/* lease_out.c - leased-KMS output for mirage (see lease_out.h).
 *
 * Adapted from the proven POC (tools/lease_kms_poc.c): bind wp_drm_lease, lease
 * DP-1, then GBM + EGL + ATOMIC page-flips straight to the leased Asahi DCP
 * connector. The DCP is atomic-only (legacy drmModePageFlip EBUSYs), so all
 * presentation goes through drmModeAtomicCommit. The panel supports 120 Hz;
 * launch scripts currently choose a stable 60 Hz render cadence by default. */
#include "mirage.h"
#include "lease_out.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include <unistd.h>
#include <poll.h>
#include <signal.h>
#include <time.h>

#include <xf86drm.h>
#include <xf86drmMode.h>
#include <gbm.h>
#include <EGL/eglext.h>

#include "drm-lease-v1-client-protocol.h"

/* All lease/KMS state lives here; mirage only carries the EGL fields + m->lease. */
static struct {
    struct wp_drm_lease_device_v1 *dev;
    struct wp_drm_lease_connector_v1 *conn;   /* the DP-1 lease connector object */
    struct wp_drm_lease_v1 *lease;
    uint32_t conn_id;                          /* DRM connector id (DP-1)         */
    int      done;                             /* lease handshake settled         */
    int      fd;                               /* leased DRM master fd            */

    drmModeModeInfo mode;
    uint32_t crtc_id, plane_id, blob;
    /* atomic property ids */
    uint32_t p_fb, p_cr, p_sx, p_sy, p_sw, p_sh, p_cx, p_cy, p_cw, p_ch;
    uint32_t c_mode, c_active, k_crtc;

    struct gbm_device  *gbm;
    struct gbm_surface *gs;
    struct gbm_bo      *prev;
    int      first;                            /* first commit does the modeset   */
    int      flip_pending;

    struct timespec deadline;                  /* next-frame target for the pacer */
    /* flip-wait diagnostic: is the page-flip event actually vblank-fenced? */
    double   fw_sum, fw_worst; long fw_n; struct timespec fw_t0;
} L;

/* Cap the present loop to a steady cadence (panel refresh by default, or
 * MIRAGE_FPS_CAP Hz; 0 = uncapped). This is a FLOOR on frame time, not a second
 * vsync: if the DCP's page-flip event is properly vblank-fenced the flip-wait
 * already consumed part of the period; if the DCP fires the event early, this
 * holds the configured cadence. clock_nanosleep(ABSTIME) against an advancing
 * deadline avoids drift. */
static void pace_to_cadence(void) {
    static double hz = -1.0;
    if (hz < 0.0) {
        const char *c = getenv("MIRAGE_FPS_CAP");
        hz = c ? atof(c) : (L.mode.vrefresh ? (double)L.mode.vrefresh : 120.0);
        if (hz > 0.0) fprintf(stderr, "lease: pacing to %.1f Hz%s\n",
                              hz, c ? " (MIRAGE_FPS_CAP)" : " (panel refresh)");
        else fprintf(stderr, "lease: pacing DISABLED (MIRAGE_FPS_CAP=0)\n");
    }
    if (hz <= 0.0) return;
    long period = (long)(1e9 / hz);
    struct timespec now; clock_gettime(CLOCK_MONOTONIC, &now);
    if (L.deadline.tv_sec == 0) L.deadline = now;
    L.deadline.tv_nsec += period;
    while (L.deadline.tv_nsec >= 1000000000L) { L.deadline.tv_nsec -= 1000000000L; L.deadline.tv_sec++; }
    /* fell a whole frame behind (slow frame): resync, don't burst to catch up */
    double behind = (now.tv_sec - L.deadline.tv_sec) + (now.tv_nsec - L.deadline.tv_nsec) / 1e9;
    if (behind > 0.0) { L.deadline = now; return; }
    clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &L.deadline, NULL);
}

/* ---- wp_drm_lease discovery (find + acquire DP-1) ---- */
static void c_name(void *d, struct wp_drm_lease_connector_v1 *c, const char *s) {
    (void)d; if (!strcmp(s, "DP-1")) L.conn = c;
}
static void c_desc(void *d, struct wp_drm_lease_connector_v1 *c, const char *s){(void)d;(void)c;(void)s;}
static void c_id(void *d, struct wp_drm_lease_connector_v1 *c, uint32_t i){(void)d; if (c==L.conn) L.conn_id=i;}
static void c_done(void *d, struct wp_drm_lease_connector_v1 *c){(void)d;(void)c;}
static void c_wd(void *d, struct wp_drm_lease_connector_v1 *c){(void)d;(void)c;}
static const struct wp_drm_lease_connector_v1_listener CL = {c_name,c_desc,c_id,c_done,c_wd};

static void le_fd(void *d, struct wp_drm_lease_v1 *l, int fd){(void)d;(void)l; L.fd=fd; L.done=1;}
static void le_fin(void *d, struct wp_drm_lease_v1 *l){(void)d;(void)l; L.done=1;}  /* denied/revoked */
static const struct wp_drm_lease_v1_listener LEL = {le_fd, le_fin};

static void d_fd(void *d, struct wp_drm_lease_device_v1 *dv, int fd){(void)d;(void)dv; close(fd);}
static void d_conn(void *d, struct wp_drm_lease_device_v1 *dv, struct wp_drm_lease_connector_v1 *c){
    (void)d;(void)dv; wp_drm_lease_connector_v1_add_listener(c, &CL, NULL);
}
static void d_done(void *d, struct wp_drm_lease_device_v1 *dv){(void)d;(void)dv;}
static void d_rel(void *d, struct wp_drm_lease_device_v1 *dv){(void)d;(void)dv;}
static const struct wp_drm_lease_device_v1_listener DL = {d_fd,d_conn,d_done,d_rel};

static void reg_global(void *d, struct wl_registry *r, uint32_t n, const char *i, uint32_t v){
    (void)d;(void)v;
    if (!strcmp(i, wp_drm_lease_device_v1_interface.name)) {
        L.dev = wl_registry_bind(r, n, &wp_drm_lease_device_v1_interface, 1);
        wp_drm_lease_device_v1_add_listener(L.dev, &DL, NULL);
    }
}
static void reg_remove(void *d, struct wl_registry *r, uint32_t n){(void)d;(void)r;(void)n;}
static const struct wl_registry_listener REG = {reg_global, reg_remove};

/* ---- KMS helpers ---- */
static uint32_t prop_id(int fd, uint32_t obj, uint32_t type, const char *name){
    drmModeObjectProperties *p = drmModeObjectGetProperties(fd, obj, type);
    uint32_t id = 0;
    if (p) {
        for (uint32_t i=0;i<p->count_props;i++){
            drmModePropertyRes *pr = drmModeGetProperty(fd, p->props[i]);
            if (pr){ if (!strcmp(pr->name, name)) id = pr->prop_id; drmModeFreeProperty(pr); }
        }
        drmModeFreeObjectProperties(p);
    }
    return id;
}
struct fbwrap { uint32_t fb; int fd; };
static void bo_destroy(struct gbm_bo *bo, void *data){
    (void)bo; struct fbwrap *f = data;
    if (f){ if (f->fb) drmModeRmFB(f->fd, f->fb); free(f); }
}
static uint32_t fb_for_bo(int fd, struct gbm_bo *bo){
    struct fbwrap *f = gbm_bo_get_user_data(bo);
    if (f) return f->fb;
    uint32_t h[4]={0}, s[4]={0}, o[4]={0};
    h[0] = gbm_bo_get_handle(bo).u32; s[0] = gbm_bo_get_stride(bo);
    f = calloc(1, sizeof *f); f->fd = fd;
    if (drmModeAddFB2(fd, gbm_bo_get_width(bo), gbm_bo_get_height(bo),
                      gbm_bo_get_format(bo), h, s, o, &f->fb, 0)) {
        fprintf(stderr, "lease: AddFB2: %s\n", strerror(errno)); free(f); return 0;
    }
    gbm_bo_set_user_data(bo, f, bo_destroy);
    return f->fb;
}

bool lease_out_init(struct mirage *m){
    memset(&L, 0, sizeof L); L.fd = -1; L.first = 1;

    /* 1. discover + lease DP-1 over the existing Wayland connection */
    struct wl_registry *reg = wl_display_get_registry(m->display);
    wl_registry_add_listener(reg, &REG, NULL);
    wl_display_roundtrip(m->display);
    if (!L.dev) { fprintf(stderr, "lease: no wp_drm_lease_device (compositor support?)\n"); return false; }
    wl_display_roundtrip(m->display);    /* connector offers */
    wl_display_roundtrip(m->display);    /* names/ids        */
    if (!L.conn) { fprintf(stderr, "lease: DP-1 not offered for lease (kernel non-desktop patch missing?)\n"); return false; }

    struct wp_drm_lease_request_v1 *req = wp_drm_lease_device_v1_create_lease_request(L.dev);
    wp_drm_lease_request_v1_request_connector(req, L.conn);
    L.lease = wp_drm_lease_request_v1_submit(req);
    wp_drm_lease_v1_add_listener(L.lease, &LEL, NULL);
    while (!L.done) if (wl_display_dispatch(m->display) < 0) break;
    if (L.fd < 0) { fprintf(stderr, "lease: lease denied\n"); return false; }
    fprintf(stderr, "lease: acquired DP-1 (fd=%d)\n", L.fd);

    /* 2. KMS: mode, crtc, primary plane, atomic property ids */
    drmSetClientCap(L.fd, DRM_CLIENT_CAP_UNIVERSAL_PLANES, 1);
    drmSetClientCap(L.fd, DRM_CLIENT_CAP_ATOMIC, 1);
    drmModeRes *res = drmModeGetResources(L.fd);
    drmModeConnector *conn = drmModeGetConnector(L.fd, L.conn_id);
    if (!res || !conn || conn->count_modes == 0) { fprintf(stderr, "lease: no modes\n"); return false; }
    L.mode = conn->modes[0];
    L.crtc_id = res->crtcs[0];
    int crtc_idx = 0;
    for (int i=0;i<res->count_crtcs;i++) if (res->crtcs[i]==L.crtc_id){ crtc_idx=i; break; }
    drmModePlaneRes *pres = drmModeGetPlaneResources(L.fd);
    for (uint32_t i=0;i<pres->count_planes && !L.plane_id;i++){
        drmModePlane *pl = drmModeGetPlane(L.fd, pres->planes[i]);
        if (pl){
            if (pl->possible_crtcs & (1u<<crtc_idx)){
                uint32_t tprop = prop_id(L.fd, pl->plane_id, DRM_MODE_OBJECT_PLANE, "type");
                drmModeObjectProperties *op = drmModeObjectGetProperties(L.fd, pl->plane_id, DRM_MODE_OBJECT_PLANE);
                for (uint32_t j=0;j<op->count_props;j++)
                    if (op->props[j]==tprop && op->prop_values[j]==DRM_PLANE_TYPE_PRIMARY) L.plane_id = pl->plane_id;
                drmModeFreeObjectProperties(op);
                if (!L.plane_id) L.plane_id = pl->plane_id;   /* fallback */
            }
            drmModeFreePlane(pl);
        }
    }
    drmModeFreePlaneResources(pres);
    L.p_fb=prop_id(L.fd,L.plane_id,DRM_MODE_OBJECT_PLANE,"FB_ID");
    L.p_cr=prop_id(L.fd,L.plane_id,DRM_MODE_OBJECT_PLANE,"CRTC_ID");
    L.p_sx=prop_id(L.fd,L.plane_id,DRM_MODE_OBJECT_PLANE,"SRC_X");
    L.p_sy=prop_id(L.fd,L.plane_id,DRM_MODE_OBJECT_PLANE,"SRC_Y");
    L.p_sw=prop_id(L.fd,L.plane_id,DRM_MODE_OBJECT_PLANE,"SRC_W");
    L.p_sh=prop_id(L.fd,L.plane_id,DRM_MODE_OBJECT_PLANE,"SRC_H");
    L.p_cx=prop_id(L.fd,L.plane_id,DRM_MODE_OBJECT_PLANE,"CRTC_X");
    L.p_cy=prop_id(L.fd,L.plane_id,DRM_MODE_OBJECT_PLANE,"CRTC_Y");
    L.p_cw=prop_id(L.fd,L.plane_id,DRM_MODE_OBJECT_PLANE,"CRTC_W");
    L.p_ch=prop_id(L.fd,L.plane_id,DRM_MODE_OBJECT_PLANE,"CRTC_H");
    L.c_mode=prop_id(L.fd,L.crtc_id,DRM_MODE_OBJECT_CRTC,"MODE_ID");
    L.c_active=prop_id(L.fd,L.crtc_id,DRM_MODE_OBJECT_CRTC,"ACTIVE");
    L.k_crtc=prop_id(L.fd,L.conn_id,DRM_MODE_OBJECT_CONNECTOR,"CRTC_ID");
    drmModeCreatePropertyBlob(L.fd, &L.mode, sizeof L.mode, &L.blob);
    drmModeFreeConnector(conn); drmModeFreeResources(res);

    m->glasses_w = L.mode.hdisplay;
    m->glasses_h = L.mode.vdisplay;
    fprintf(stderr, "lease: %s %ux%u@%u, crtc %u plane %u\n",
            L.mode.name, L.mode.hdisplay, L.mode.vdisplay, L.mode.vrefresh, L.crtc_id, L.plane_id);

    /* 3. GBM + EGL into mirage's EGL fields */
    L.gbm = gbm_create_device(L.fd);
    L.gs  = gbm_surface_create(L.gbm, L.mode.hdisplay, L.mode.vdisplay,
                               GBM_FORMAT_XRGB8888, GBM_BO_USE_SCANOUT|GBM_BO_USE_RENDERING);
    if (!L.gbm || !L.gs) { fprintf(stderr, "lease: gbm setup failed\n"); return false; }

    m->edpy = eglGetPlatformDisplay(EGL_PLATFORM_GBM_KHR, L.gbm, NULL);
    EGLint maj, min;
    if (m->edpy == EGL_NO_DISPLAY || !eglInitialize(m->edpy, &maj, &min)) { fprintf(stderr, "lease: eglInitialize failed\n"); return false; }
    eglBindAPI(EGL_OPENGL_ES_API);
    EGLint at[] = { EGL_SURFACE_TYPE, EGL_WINDOW_BIT, EGL_RED_SIZE,8, EGL_GREEN_SIZE,8,
                    EGL_BLUE_SIZE,8, EGL_RENDERABLE_TYPE, EGL_OPENGL_ES2_BIT, EGL_NONE };
    EGLConfig cfgs[32]; EGLint ncfg; eglChooseConfig(m->edpy, at, cfgs, 32, &ncfg);
    m->ecfg = cfgs[0];
    for (int i=0;i<ncfg;i++){ EGLint vid; eglGetConfigAttrib(m->edpy, cfgs[i], EGL_NATIVE_VISUAL_ID, &vid);
        if (vid==(EGLint)GBM_FORMAT_XRGB8888){ m->ecfg=cfgs[i]; break; } }
    EGLint ca[] = { EGL_CONTEXT_CLIENT_VERSION, 2, EGL_NONE };
    m->ectx = eglCreateContext(m->edpy, m->ecfg, EGL_NO_CONTEXT, ca);
    m->esurf = eglCreateWindowSurface(m->edpy, m->ecfg, (EGLNativeWindowType)L.gs, NULL);
    if (m->ectx==EGL_NO_CONTEXT || m->esurf==EGL_NO_SURFACE) { fprintf(stderr, "lease: EGL surface/context failed (0x%x)\n", eglGetError()); return false; }
    eglMakeCurrent(m->edpy, m->esurf, m->esurf, m->ectx);
    fprintf(stderr, "lease: GL_RENDERER=%s\n", glGetString(GL_RENDERER));
    return true;
}

static void on_flip(int fd, unsigned seq, unsigned tv_s, unsigned tv_us, unsigned crtc, void *u) {
    (void)fd; (void)seq; (void)tv_s; (void)tv_us; (void)crtc; *(int*)u = 0;
}

void lease_out_present(struct mirage *m){
    eglSwapBuffers(m->edpy, m->esurf);
    struct gbm_bo *bo = gbm_surface_lock_front_buffer(L.gs);
    uint32_t fb = fb_for_bo(L.fd, bo);
    if (!fb) { if (bo) gbm_surface_release_buffer(L.gs, bo); return; }

    bool modeset = L.first;
    drmModeAtomicReq *a = drmModeAtomicAlloc();
    uint32_t flags;
    if (modeset) {
        /* Bring the display UP. On the Asahi DCP the modeset powers the external
         * display's controller + enables its timings, and that's ASYNC (~2s link
         * training - the firmware "swallows" swaps with fControllerPowerState 0
         * until it's done). So commit this BLOCKING (no NONBLOCK, no flip event):
         * drmModeAtomicCommit returns only once the DCP has actually enabled the
         * display, so the FIRST flip never races an unpowered controller. */
        drmModeAtomicAddProperty(a, L.crtc_id, L.c_mode, L.blob);
        drmModeAtomicAddProperty(a, L.crtc_id, L.c_active, 1);
        drmModeAtomicAddProperty(a, L.conn_id, L.k_crtc, L.crtc_id);
        drmModeAtomicAddProperty(a, L.plane_id, L.p_cr, L.crtc_id);
        drmModeAtomicAddProperty(a, L.plane_id, L.p_sx, 0);
        drmModeAtomicAddProperty(a, L.plane_id, L.p_sy, 0);
        drmModeAtomicAddProperty(a, L.plane_id, L.p_sw, (uint64_t)L.mode.hdisplay<<16);
        drmModeAtomicAddProperty(a, L.plane_id, L.p_sh, (uint64_t)L.mode.vdisplay<<16);
        drmModeAtomicAddProperty(a, L.plane_id, L.p_cx, 0);
        drmModeAtomicAddProperty(a, L.plane_id, L.p_cy, 0);
        drmModeAtomicAddProperty(a, L.plane_id, L.p_cw, L.mode.hdisplay);
        drmModeAtomicAddProperty(a, L.plane_id, L.p_ch, L.mode.vdisplay);
        drmModeAtomicAddProperty(a, L.plane_id, L.p_fb, fb);
        flags = DRM_MODE_ATOMIC_ALLOW_MODESET;                 /* blocking */
    } else {
        drmModeAtomicAddProperty(a, L.plane_id, L.p_fb, fb);
        flags = DRM_MODE_ATOMIC_NONBLOCK | DRM_MODE_PAGE_FLIP_EVENT;
        L.flip_pending = 1;
    }
    /* Modeset is BLOCKING (above) so the DCP fully powers the display before we ever
     * flip. Flips are NONBLOCK + page-flip event: that hits the panel's 120Hz when
     * the link is healthy, and fails FAST (EBUSY) if it stalls - rather than a
     * blocking flip hanging ~17s on a vblank that never comes. */
    int rc = drmModeAtomicCommit(L.fd, a, flags, modeset ? NULL : &L.flip_pending);
    drmModeAtomicFree(a);

    static int fail_streak = 0;
    static struct timespec t0 = {0, 0};
    if (rc) {
        L.flip_pending = 0;
        if (modeset) {
            /* DCP still bringing the link up: keep retrying the blocking modeset for
             * up to ~12s (paced, so we don't busy-spin) before giving up. */
            struct timespec now; clock_gettime(CLOCK_MONOTONIC, &now);
            if (t0.tv_sec == 0) { t0 = now; fprintf(stderr, "lease: bringing the display up (waiting on the DCP)...\n"); }
            double el = (now.tv_sec - t0.tv_sec) + (now.tv_nsec - t0.tv_nsec) / 1e9;
            if (el > 12.0) {
                fprintf(stderr, "lease: display never powered on after %.0fs (DCP timings) - exiting.\n", el);
                raise(SIGTERM);
            }
            usleep(120 * 1000);
        } else {
            /* Tell a real disconnect apart from a transient EBUSY. The Asahi DCP
             * wedges after lease churn and rejects ~every other flip with EBUSY
             * (glasses still present; a reboot resets it) - that must NOT count as
             * a disconnect, and must NOT spew thousands of log lines/sec. Only a
             * SUSTAINED hard error means the glasses are actually gone. */
            int e = errno;
            static struct timespec last_log = {0, 0};
            struct timespec now; clock_gettime(CLOCK_MONOTONIC, &now);
            double since = (now.tv_sec - last_log.tv_sec) + (now.tv_nsec - last_log.tv_nsec) / 1e9;
            if (last_log.tv_sec == 0 || since >= 1.0) {
                fprintf(stderr, "lease: page-flip rejected (%s)%s\n", strerror(e),
                        e == EBUSY ? " - DCP wedged; REBOOT to reset (rate still capped)"
                                   : " - glasses unplugged?");
                last_log = now;
            }
            if (e == EBUSY) fail_streak = 0;          /* transient: never exit on this */
            else if (++fail_streak > 180) {
                fprintf(stderr, "lease: lost the glasses connection - exiting.\n");
                raise(SIGTERM);
            }
        }
        gbm_surface_release_buffer(L.gs, bo);    /* not shown; recycle */
        pace_to_cadence();   /* keep the loop at 120 even while the DCP rejects flips */
        return;
    }
    fail_streak = 0;
    if (modeset) {
        L.first = 0;
        fprintf(stderr, "lease: display up (blocking modeset OK).\n");
    } else {
        /* Time the flip-wait: how long the page-flip-done event takes to arrive.
         * ~8.3 ms (at 120 Hz) => the DCP fences the flip to vblank => real,
         * tear-free vsync. ~0 ms => it fires the event immediately => the flip
         * lands mid-scanout (tearing) and the loop would free-run if not paced. */
        struct timespec fa; clock_gettime(CLOCK_MONOTONIC, &fa);
        struct pollfd pfd = { .fd = L.fd, .events = POLLIN };
        while (L.flip_pending) {
            if (poll(&pfd, 1, 100) <= 0) { L.flip_pending = 0; break; }
            drmEventContext ev = { .version = 3, .page_flip_handler2 = on_flip };
            drmHandleEvent(L.fd, &ev);
        }
        struct timespec fb; clock_gettime(CLOCK_MONOTONIC, &fb);
        double waited = (fb.tv_sec - fa.tv_sec) * 1000.0 + (fb.tv_nsec - fa.tv_nsec) / 1e6;
        L.fw_sum += waited; L.fw_n++; if (waited > L.fw_worst) L.fw_worst = waited;
        if (L.fw_t0.tv_sec == 0) L.fw_t0 = fb;
        double el = (fb.tv_sec - L.fw_t0.tv_sec) + (fb.tv_nsec - L.fw_t0.tv_nsec) / 1e9;
        if (el >= 1.0 && L.fw_n > 0) {
            double avg = L.fw_sum / (double)L.fw_n;
            fprintf(stderr, "lease: flip-wait avg %.2f ms worst %.2f ms (%ld flips) - %s\n",
                    avg, L.fw_worst, L.fw_n,
                    avg < 2.0 ? "NOT vblank-fenced (tearing source; pacer holds the rate)"
                              : "vblank-fenced (vsync is real)");
            L.fw_sum = 0; L.fw_worst = 0; L.fw_n = 0; L.fw_t0 = fb;
        }
    }
    if (L.prev) gbm_surface_release_buffer(L.gs, L.prev);
    L.prev = bo;
    pace_to_cadence();   /* hold a steady 120 even if the DCP free-runs the flips */
}

void lease_out_finish(struct mirage *m){
    /* EGL (incl. the surface wrapping L.gs) is already torn down by render_finish;
     * here we only release the front buffer, GBM, the lease, and the fd. */
    if (L.prev) gbm_surface_release_buffer(L.gs, L.prev);
    if (L.gs)  gbm_surface_destroy(L.gs);
    if (L.gbm) gbm_device_destroy(L.gbm);
    if (L.lease) wp_drm_lease_v1_destroy(L.lease);
    if (L.fd >= 0) close(L.fd);
    if (m->display) wl_display_flush(m->display);
}
