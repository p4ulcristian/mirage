#include "mirage.h"
#include "pose.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <signal.h>
#include <unistd.h>
#include <time.h>
#include <poll.h>

#include "wlr-layer-shell-unstable-v1-client-protocol.h"
#include "wlr-screencopy-unstable-v1-client-protocol.h"
#include "linux-dmabuf-unstable-v1-client-protocol.h"
#include "xdg-shell-client-protocol.h"
#include "pointer-constraints-unstable-v1-client-protocol.h"
#include "relative-pointer-unstable-v1-client-protocol.h"
#include "wlr-virtual-pointer-unstable-v1-client-protocol.h"
#include <wayland-egl.h>

static struct mirage M;
static volatile sig_atomic_t g_stop = 0;
static volatile sig_atomic_t g_recenter = 0;
static volatile sig_atomic_t g_grab_toggle = 0;
static void on_sig(int s) { (void)s; g_stop = 1; }
static void on_recenter(int s) { (void)s; g_recenter = 1; }     /* SIGUSR1 */
static void on_grab(int s) { (void)s; g_grab_toggle = 1; }      /* SIGUSR2 */

/* override which output is the render target (default: auto by glasses_match) */
static const char *opt_output = NULL;
/* per-axis pose inversion (calibrate to your IMU driver's convention) */
static float opt_sign_yaw = 1.0f, opt_sign_pitch = 1.0f, opt_sign_roll = 1.0f;
/* head-tracked 3D arc (off for now: capture-only flat view by default) */
static bool opt_3d = false;
/* windowed scene-setup mode: render into a normal xdg-shell window */
static bool opt_windowed = false;
static bool opt_preview  = false;          /* laptop preview window (--preview) */
static int  opt_pv_w = 1280, opt_pv_h = 480;
static int  opt_win_w = 1280, opt_win_h = 720;
static struct xdg_wm_base  *g_wm_base  = NULL;
static struct xdg_toplevel *g_toplevel = NULL;
static int32_t g_win_cfg_w = 0, g_win_cfg_h = 0;
static bool    g_pv_configured = false;

static void wm_ping(void *d, struct xdg_wm_base *b, uint32_t serial) {
    (void)d; xdg_wm_base_pong(b, serial);
}
static const struct xdg_wm_base_listener WM_BASE_LISTENER = { .ping = wm_ping };

static void xsurf_configure(void *d, struct xdg_surface *xs, uint32_t serial) {
    (void)d; xdg_surface_ack_configure(xs, serial); M.configured = true;
}
static const struct xdg_surface_listener XSURF_LISTENER = { .configure = xsurf_configure };

static void xtop_configure(void *d, struct xdg_toplevel *t, int32_t w, int32_t h,
                           struct wl_array *states) {
    (void)d;(void)t;(void)states;
    if (w > 0 && h > 0) { g_win_cfg_w = w; g_win_cfg_h = h; }
}
static void xtop_close(void *d, struct xdg_toplevel *t) {
    (void)d;(void)t; M.running = false; g_stop = 1;
}
static const struct xdg_toplevel_listener XTOP_LISTENER = {
    .configure = xtop_configure, .close = xtop_close,
};

/* ---- laptop preview window: its own xdg listeners (separate config state) ---- */
static void pv_xsurf_configure(void *d, struct xdg_surface *xs, uint32_t serial) {
    (void)d; xdg_surface_ack_configure(xs, serial); g_pv_configured = true;
}
static const struct xdg_surface_listener PV_XSURF_LISTENER = { .configure = pv_xsurf_configure };

static void pv_xtop_configure(void *d, struct xdg_toplevel *t, int32_t w, int32_t h,
                              struct wl_array *states) {
    (void)d;(void)t;(void)states;
    if (w > 0 && h > 0) { M.pv_cfg_w = w; M.pv_cfg_h = h; }
}
static void pv_xtop_close(void *d, struct xdg_toplevel *t) {
    (void)d;(void)t; M.pv_enabled = false;   /* closing the preview != quitting */
}
static const struct xdg_toplevel_listener PV_XTOP_LISTENER = {
    .configure = pv_xtop_configure, .close = pv_xtop_close,
};

/* ---- wl_output discovery ---- */
static void out_geometry(void *d, struct wl_output *o, int32_t x, int32_t y,
        int32_t pw, int32_t ph, int32_t sp, const char *make, const char *model,
        int32_t tr) {
    (void)o;(void)x;(void)y;(void)pw;(void)ph;(void)sp;(void)tr;
    int idx = (int)(intptr_t)d;
    snprintf(M.pending[idx].desc, sizeof M.pending[idx].desc, "%s %s",
             make ? make : "", model ? model : "");
}
static void out_mode(void *d, struct wl_output *o, uint32_t flags,
        int32_t w, int32_t h, int32_t refresh) {
    (void)o;(void)refresh;
    int idx = (int)(intptr_t)d;
    if (flags & WL_OUTPUT_MODE_CURRENT) { M.pending[idx].w = w; M.pending[idx].h = h; }
}
static void out_done(void *d, struct wl_output *o) {
    (void)o; int idx = (int)(intptr_t)d; M.pending[idx].done = true;
}
static void out_scale(void *d, struct wl_output *o, int32_t s) { (void)d;(void)o;(void)s; }
static void out_name(void *d, struct wl_output *o, const char *name) {
    (void)o; int idx = (int)(intptr_t)d;
    snprintf(M.pending[idx].name, sizeof M.pending[idx].name, "%s", name);
}
static void out_description(void *d, struct wl_output *o, const char *desc) {
    (void)o; int idx = (int)(intptr_t)d;
    if (desc) snprintf(M.pending[idx].desc, sizeof M.pending[idx].desc, "%s", desc);
}
static const struct wl_output_listener OUTPUT_LISTENER = {
    .geometry = out_geometry, .mode = out_mode, .done = out_done,
    .scale = out_scale, .name = out_name, .description = out_description,
};

/* ---- registry ---- */
static void reg_global(void *d, struct wl_registry *r, uint32_t name,
                       const char *iface, uint32_t ver) {
    (void)d;
    if (!strcmp(iface, wl_compositor_interface.name)) {
        M.compositor = wl_registry_bind(r, name, &wl_compositor_interface, 4);
    } else if (!strcmp(iface, zwlr_layer_shell_v1_interface.name)) {
        M.layer_shell = wl_registry_bind(r, name, &zwlr_layer_shell_v1_interface,
                                         ver < 4 ? ver : 4);
    } else if (!strcmp(iface, zwlr_screencopy_manager_v1_interface.name)) {
        M.screencopy = wl_registry_bind(r, name, &zwlr_screencopy_manager_v1_interface,
                                        ver < 3 ? ver : 3);
    } else if (!strcmp(iface, zwp_linux_dmabuf_v1_interface.name)) {
        M.dmabuf = wl_registry_bind(r, name, &zwp_linux_dmabuf_v1_interface,
                                    ver < 3 ? ver : 3);
    } else if (!strcmp(iface, wl_shm_interface.name)) {
        M.shm = wl_registry_bind(r, name, &wl_shm_interface, 1);
    } else if (!strcmp(iface, wl_seat_interface.name)) {
        M.seat = wl_registry_bind(r, name, &wl_seat_interface, ver < 5 ? ver : 5);
    } else if (!strcmp(iface, zwp_pointer_constraints_v1_interface.name)) {
        M.pointer_constraints = wl_registry_bind(r, name,
                                  &zwp_pointer_constraints_v1_interface, 1);
    } else if (!strcmp(iface, zwp_relative_pointer_manager_v1_interface.name)) {
        M.rel_pointer_mgr = wl_registry_bind(r, name,
                                  &zwp_relative_pointer_manager_v1_interface, 1);
    } else if (!strcmp(iface, zwlr_virtual_pointer_manager_v1_interface.name)) {
        /* v2 required for create_virtual_pointer_with_output */
        M.vpointer_mgr = wl_registry_bind(r, name,
                                  &zwlr_virtual_pointer_manager_v1_interface,
                                  ver < 2 ? ver : 2);
    } else if (!strcmp(iface, xdg_wm_base_interface.name)) {
        g_wm_base = wl_registry_bind(r, name, &xdg_wm_base_interface, ver < 4 ? ver : 4);
        xdg_wm_base_add_listener(g_wm_base, &WM_BASE_LISTENER, NULL);
    } else if (!strcmp(iface, wl_output_interface.name)) {
        if (M.n_pending < 16) {
            int idx = M.n_pending++;
            struct wl_output *o = wl_registry_bind(r, name, &wl_output_interface, 4);
            M.pending[idx].wl = o;
            wl_output_add_listener(o, &OUTPUT_LISTENER, (void*)(intptr_t)idx);
        }
    }
}
static void reg_remove(void *d, struct wl_registry *r, uint32_t name) {
    (void)d;(void)r;(void)name;
}
static const struct wl_registry_listener REGISTRY_LISTENER = {
    .global = reg_global, .global_remove = reg_remove,
};

/* ---- layer surface ---- */
static void ls_configure(void *d, struct zwlr_layer_surface_v1 *ls,
                         uint32_t serial, uint32_t w, uint32_t h) {
    (void)d;
    zwlr_layer_surface_v1_ack_configure(ls, serial);
    if (w > 0 && h > 0) { M.glasses_w = (int32_t)w; M.glasses_h = (int32_t)h; }
    M.configured = true;
}
static void ls_closed(void *d, struct zwlr_layer_surface_v1 *ls) {
    (void)d;(void)ls; M.running = false;
}
static const struct zwlr_layer_surface_v1_listener LS_LISTENER = {
    .configure = ls_configure, .closed = ls_closed,
};

static int classify_outputs(void) {
    /* glasses output */
    int glasses = -1;
    for (int i = 0; i < M.n_pending; i++) {
        const char *n = M.pending[i].name, *de = M.pending[i].desc;
        if (opt_output) {
            if (!strcmp(n, opt_output)) { glasses = i; break; }
        } else if (strstr(de, M.cfg.glasses_match) || strstr(n, M.cfg.glasses_match)
                   || strstr(de, "RayNeo")) {
            glasses = i; break;
        }
    }
    if (glasses < 0 && !opt_windowed) {
        fprintf(stderr, "mirage: glasses output not found (match='%s'%s%s)\n",
                M.cfg.glasses_match, opt_output ? ", --output=" : "",
                opt_output ? opt_output : "");
        fprintf(stderr, "  available outputs:\n");
        for (int i = 0; i < M.n_pending; i++)
            fprintf(stderr, "    %-10s %dx%d  [%s]\n", M.pending[i].name,
                    M.pending[i].w, M.pending[i].h, M.pending[i].desc);
        return -1;
    }
    if (glasses >= 0) {
        M.glasses_out = M.pending[glasses].wl;
        snprintf(M.glasses_name, sizeof M.glasses_name, "%s", M.pending[glasses].name);
        snprintf(M.glasses_desc, sizeof M.glasses_desc, "%s", M.pending[glasses].desc);
        M.glasses_w = M.pending[glasses].w;
        M.glasses_h = M.pending[glasses].h;
    } else {
        snprintf(M.glasses_name, sizeof M.glasses_name, "windowed");
        M.glasses_w = opt_win_w; M.glasses_h = opt_win_h;
    }

    /* virtual screens: outputs named VIRT*, in name order */
    for (int pass = 1; pass <= MIRAGE_MAX_SCREENS; pass++) {
        char want[16]; snprintf(want, sizeof want, "VIRT%d", pass);
        for (int i = 0; i < M.n_pending; i++) {
            if (!strcmp(M.pending[i].name, want) && M.n_screen < MIRAGE_MAX_SCREENS) {
                screen_t *s = &M.screen[M.n_screen];
                memset(s, 0, sizeof *s);
                s->wl = M.pending[i].wl;
                snprintf(s->name, sizeof s->name, "%s", want);
                s->width  = M.pending[i].w;
                s->height = M.pending[i].h;
                s->index  = M.n_screen;
                s->image  = EGL_NO_IMAGE_KHR;
                s->dmabuf_fd = -1;
                M.n_screen++;
                break;
            }
        }
    }
    fprintf(stderr, "mirage: glasses=%s (%dx%d) [%s]; %d virtual screens\n",
            M.glasses_name, M.glasses_w, M.glasses_h, M.glasses_desc, M.n_screen);
    return M.n_screen > 0 ? 0 : -1;
}

static void usage(const char *p) {
    printf("usage: %s [options]\n"
           "  --output NAME     render target output (default: auto-detect glasses)\n"
           "  --port N          OpenTrack UDP port (default 4242)\n"
           "  --fov DEG         glasses vertical FOV (default 26)\n"
           "  --distance M      screen distance in metres (default 2.0)\n"
           "  --spacing DEG     extra gap between screens (default 0)\n"
           "  --arc DEG         angular width of each curved screen (default 40)\n"
           "  --mincutoff HZ    One-Euro steadiness at rest (default 0.5; lower = steadier)\n"
           "  --beta F          One-Euro responsiveness in motion (default 1.0; higher = less lag)\n"
           "  --yaw-gain F      amplify head yaw (default 2.5; >1 = reach side screens with less turn)\n"
           "  --pitch-gain F    amplify head pitch (default 3.0; >1 = reach the top row with less look-up)\n"
           "  --roll-damp F     fraction of head roll kept (default 0.25; 0 = full horizon lock)\n"
           "  --read-deadband D freeze camera tremor below D deg for steady text (default 0.22; 0 = off)\n"
           "  --sharpen S       contrast-adaptive text sharpen strength (default 0.35; 0 = off)\n"
           "  --flat/--cylinder  screen surface: flat panels (default) or curved strips\n"
           "  --smooth F        use the legacy fixed nlerp instead of One-Euro (0..1)\n"
           "  --screens N       expected virtual screen count (default 3)\n"
           "  --invert-yaw      flip yaw if turning your head feels reversed\n"
           "  --invert-pitch    flip pitch\n"
           "  --invert-roll     flip roll\n"
           "  --windowed [WxH]  render into a normal window (scene setup; default 1280x720)\n"
           "  --preview [WxH]   also open a laptop window mirroring the screens (default 1280x480)\n"
           "  --3d              enable head-tracked 3D arc (default: flat capture-only)\n", p);
}

int main(int argc, char **argv) {
    mirage_config_defaults(&M.cfg);
    M.zoom = 1.0f;
    M.lens_power = M.lens_target = 1.0f;   /* loupe off until Cmd is held */
    M.view_focus = (M.cfg.screen_cols > 0 ? M.cfg.screen_cols - 1 : 2) / 2;  /* centre screen */
    for (int i = 1; i < argc; i++) {
        if      (!strcmp(argv[i], "--output")   && i+1<argc) opt_output = argv[++i];
        else if (!strcmp(argv[i], "--port")     && i+1<argc) M.cfg.pose_port = atoi(argv[++i]);
        else if (!strcmp(argv[i], "--fov")      && i+1<argc) M.cfg.fov_deg = atof(argv[++i]);
        else if (!strcmp(argv[i], "--distance") && i+1<argc) M.cfg.screen_distance_m = atof(argv[++i]);
        else if (!strcmp(argv[i], "--spacing")  && i+1<argc) M.cfg.arc_spacing_deg = atof(argv[++i]);
        else if (!strcmp(argv[i], "--arc")      && i+1<argc) M.cfg.screen_arc_deg = atof(argv[++i]);
        else if (!strcmp(argv[i], "--mincutoff") && i+1<argc) M.cfg.pose_mincutoff = atof(argv[++i]);
        else if (!strcmp(argv[i], "--beta")      && i+1<argc) M.cfg.pose_beta = atof(argv[++i]);
        else if (!strcmp(argv[i], "--yaw-gain")  && i+1<argc) M.cfg.yaw_gain = atof(argv[++i]);
        else if (!strcmp(argv[i], "--pitch-gain")&& i+1<argc) M.cfg.pitch_gain = atof(argv[++i]);
        else if (!strcmp(argv[i], "--roll-damp") && i+1<argc) M.cfg.roll_damp = atof(argv[++i]);
        else if (!strcmp(argv[i], "--read-deadband") && i+1<argc) M.cfg.read_deadband_deg = atof(argv[++i]);
        else if (!strcmp(argv[i], "--sharpen") && i+1<argc) M.cfg.sharpen = atof(argv[++i]);
        else if (!strcmp(argv[i], "--flat"))     M.cfg.geometry = GEOM_FLAT;
        else if (!strcmp(argv[i], "--cylinder")) M.cfg.geometry = GEOM_CYLINDER;
        else if (!strcmp(argv[i], "--smooth")   && i+1<argc) {
            M.cfg.pose_smoothing = atof(argv[++i]);
            M.cfg.pose_oneeuro = false;   /* --smooth opts into the legacy filter */
        }
        else if (!strcmp(argv[i], "--screens")  && i+1<argc) M.cfg.screen_count = atoi(argv[++i]);
        else if (!strcmp(argv[i], "--invert-yaw"))   opt_sign_yaw   = -1.0f;
        else if (!strcmp(argv[i], "--invert-pitch")) opt_sign_pitch = -1.0f;
        else if (!strcmp(argv[i], "--invert-roll"))  opt_sign_roll  = -1.0f;
        else if (!strcmp(argv[i], "--3d")) opt_3d = true;
        else if (!strcmp(argv[i], "--windowed")) {
            opt_windowed = true;
            if (i+1 < argc && argv[i+1][0] != '-' &&
                sscanf(argv[i+1], "%dx%d", &opt_win_w, &opt_win_h) == 2) i++;
        }
        else if (!strcmp(argv[i], "--preview")) {
            opt_preview = true;
            if (i+1 < argc && argv[i+1][0] != '-' &&
                sscanf(argv[i+1], "%dx%d", &opt_pv_w, &opt_pv_h) == 2) i++;
        }
        else if (!strcmp(argv[i], "-h") || !strcmp(argv[i], "--help")) { usage(argv[0]); return 0; }
    }

    signal(SIGINT, on_sig);
    signal(SIGTERM, on_sig);
    signal(SIGUSR1, on_recenter);   /* recenter head pose on demand */
    signal(SIGUSR2, on_grab);       /* Super+G: toggle pointer capture */

    M.display = wl_display_connect(NULL);
    if (!M.display) { fprintf(stderr, "mirage: cannot connect to wayland\n"); return 1; }
    M.registry = wl_display_get_registry(M.display);
    wl_registry_add_listener(M.registry, &REGISTRY_LISTENER, NULL);
    wl_display_roundtrip(M.display);   /* globals */
    wl_display_roundtrip(M.display);   /* output name/desc/mode events */

    if (!M.compositor || !M.layer_shell || !M.screencopy || !M.dmabuf) {
        fprintf(stderr, "mirage: missing required wayland globals "
                "(compositor=%p layer_shell=%p screencopy=%p dmabuf=%p)\n",
                (void*)M.compositor, (void*)M.layer_shell,
                (void*)M.screencopy, (void*)M.dmabuf);
        return 1;
    }
    if (classify_outputs() != 0) return 1;

    M.surface = wl_compositor_create_surface(M.compositor);
    if (opt_windowed) {
        /* normal resizable window for tuning the scene without the glasses */
        if (!g_wm_base) { fprintf(stderr, "mirage: no xdg_wm_base\n"); return 1; }
        struct xdg_surface *xs = xdg_wm_base_get_xdg_surface(g_wm_base, M.surface);
        xdg_surface_add_listener(xs, &XSURF_LISTENER, NULL);
        g_toplevel = xdg_surface_get_toplevel(xs);
        xdg_toplevel_add_listener(g_toplevel, &XTOP_LISTENER, NULL);
        xdg_toplevel_set_title(g_toplevel, "mirage (scene setup)");
        xdg_toplevel_set_app_id(g_toplevel, "mirage");
        wl_surface_commit(M.surface);
    } else {
        /* layer-shell fullscreen overlay on the glasses */
        M.layer_surface = zwlr_layer_shell_v1_get_layer_surface(
            M.layer_shell, M.surface, M.glasses_out,
            ZWLR_LAYER_SHELL_V1_LAYER_OVERLAY, "mirage");
        zwlr_layer_surface_v1_add_listener(M.layer_surface, &LS_LISTENER, NULL);
        zwlr_layer_surface_v1_set_anchor(M.layer_surface,
            ZWLR_LAYER_SURFACE_V1_ANCHOR_TOP | ZWLR_LAYER_SURFACE_V1_ANCHOR_BOTTOM |
            ZWLR_LAYER_SURFACE_V1_ANCHOR_LEFT | ZWLR_LAYER_SURFACE_V1_ANCHOR_RIGHT);
        zwlr_layer_surface_v1_set_exclusive_zone(M.layer_surface, -1);
        zwlr_layer_surface_v1_set_keyboard_interactivity(M.layer_surface, 0);
        zwlr_layer_surface_v1_set_size(M.layer_surface, 0, 0);
        wl_surface_commit(M.surface);
    }

    while (!M.configured && wl_display_dispatch(M.display) >= 0) { /* wait config */ }
    if (opt_windowed && g_win_cfg_w > 0) { M.glasses_w = g_win_cfg_w; M.glasses_h = g_win_cfg_h; }
    if (M.glasses_w <= 0 || M.glasses_h <= 0) {
        fprintf(stderr, "mirage: bad glasses size %dx%d\n", M.glasses_w, M.glasses_h);
        return 1;
    }

    if (!render_init(&M))  { fprintf(stderr, "mirage: render_init failed\n");  return 1; }
    if (!capture_init(&M)) { fprintf(stderr, "mirage: capture_init failed\n"); return 1; }
    if (M.seat) M.pointer = wl_seat_get_pointer(M.seat);
    grab_init(&M);

    /* optional laptop preview: a normal toplevel window mirroring the flat view,
     * so the virtual screens stay visible/usable without the glasses. Separate
     * surface, drawn each frame from the same captured textures. */
    if (opt_preview && !opt_windowed && g_wm_base) {
        M.pv_surface = wl_compositor_create_surface(M.compositor);
        M.pv_xsurf = xdg_wm_base_get_xdg_surface(g_wm_base, M.pv_surface);
        xdg_surface_add_listener(M.pv_xsurf, &PV_XSURF_LISTENER, NULL);
        M.pv_xtop = xdg_surface_get_toplevel(M.pv_xsurf);
        xdg_toplevel_add_listener(M.pv_xtop, &PV_XTOP_LISTENER, NULL);
        xdg_toplevel_set_title(M.pv_xtop, "mirage preview");
        xdg_toplevel_set_app_id(M.pv_xtop, "mirage-preview");
        wl_surface_commit(M.pv_surface);
        while (!g_pv_configured && wl_display_dispatch(M.display) >= 0) { /* wait */ }
        M.pv_w = M.pv_cfg_w > 0 ? M.pv_cfg_w : opt_pv_w;
        M.pv_h = M.pv_cfg_h > 0 ? M.pv_cfg_h : opt_pv_h;
        if (render_preview_init(&M)) M.pv_enabled = true;
    }

    /* head-tracking is disabled for now (capture-only flat view).
     * pass --3d to bring back pose input + the 3D arc. */
    if (opt_3d) {
        pose_config pc = { .backend = POSE_OPENTRACK_UDP, .udp_port = M.cfg.pose_port,
                           .smoothing = M.cfg.pose_smoothing,
                           .use_oneeuro = M.cfg.pose_oneeuro,
                           .oe_mincutoff = M.cfg.pose_mincutoff,
                           .oe_beta = M.cfg.pose_beta, .oe_dcutoff = 1.0f,
                           .sign_yaw = opt_sign_yaw, .sign_pitch = opt_sign_pitch,
                           .sign_roll = opt_sign_roll };
        if (pose_start(&pc) != 0)
            fprintf(stderr, "mirage: pose backend failed to start (rendering without tracking)\n");
    }

    fprintf(stderr, "mirage: running (%s mode). Ctrl-C to quit.\n",
            opt_3d ? "3D head-tracked" : "flat capture-only");
    M.running = true;
    struct timespec fps_t0; clock_gettime(CLOCK_MONOTONIC, &fps_t0);
    long fps_frames = 0;
    /* Capture desktop content at ~60 Hz, independent of the (up-to-120 Hz)
     * camera redraw: re-blitting 3 outputs every vsync starves presentation and
     * costs frames, and screen content past 60 Hz isn't perceptible anyway. */
    struct timespec cap_t = fps_t0;
    const double CAP_PERIOD = 1.0 / 62.0;   /* a hair over 60 for headroom */
    while (M.running && !g_stop) {
        /* drain pending events first (xdg ping/pong, resizes) so the
         * compositor never flags us as unresponsive */
        wl_display_dispatch_pending(M.display);

        /* windowed mode: follow compositor-driven resizes */
        if (opt_windowed && g_win_cfg_w > 0 &&
            (g_win_cfg_w != M.glasses_w || g_win_cfg_h != M.glasses_h)) {
            M.glasses_w = g_win_cfg_w; M.glasses_h = g_win_cfg_h;
            wl_egl_window_resize(M.egl_window, M.glasses_w, M.glasses_h, 0, 0);
        }
        /* Decoupled capture: fire a screencopy for any idle screen, then pump
         * the Wayland socket NON-blocking and consume whatever has landed. We
         * never block waiting for content here — the glasses vsync (eglSwapBuffers
         * below) is the only pacer, so the head-pose camera redraws every panel
         * refresh (120 Hz) while desktop content refreshes opportunistically.
         * Blocking on capture here used to stack a second 120 Hz wait in series
         * with vsync, halving the rate to 60. */
        {   /* fire a fresh capture batch only when the ~60 Hz period elapses */
            struct timespec ct; clock_gettime(CLOCK_MONOTONIC, &ct);
            double since = (ct.tv_sec - cap_t.tv_sec) + (ct.tv_nsec - cap_t.tv_nsec) * 1e-9;
            if (since >= CAP_PERIOD) { capture_begin_frame(&M); cap_t = ct; }
        }
        while (wl_display_prepare_read(M.display) != 0)
            wl_display_dispatch_pending(M.display);
        wl_display_flush(M.display);
        struct pollfd pfd = { .fd = wl_display_get_fd(M.display), .events = POLLIN };
        if (poll(&pfd, 1, 0) > 0 && (pfd.revents & POLLIN))
            wl_display_read_events(M.display);
        else
            wl_display_cancel_read(M.display);
        wl_display_dispatch_pending(M.display);
        capture_poll(&M);   /* bind any ready frames into textures; never blocks */

        /* A protocol error tears down the connection but leaves eglSwapBuffers
         * spinning unthrottled into an unmapped surface (invisible, 1000s of
         * fps). Catch it loudly instead of pretending to render. */
        int derr = wl_display_get_error(M.display);
        if (derr) {
            fprintf(stderr, "mirage: wayland connection error (%d) - surface is "
                    "no longer presented; exiting.\n", derr);
            M.running = false;
            break;
        }

        if (g_grab_toggle) { grab_toggle(&M); g_grab_toggle = 0; }
        grab_pump(&M);   /* drain trackpad motion/buttons while captured */

        if (opt_3d) {
            /* first time we get tracking, treat the current look direction as
             * "straight ahead" so the centre screen lands in front of you. */
            static bool centered = false;
            if (!centered && pose_has_signal()) { pose_recenter(); centered = true; }
            if (g_recenter) { pose_recenter(); g_recenter = 0;
                              fprintf(stderr, "mirage: recentered\n"); }
            quat head = pose_has_signal() ? pose_latest() : q_identity();
            render_frame(&M, head);
        } else {
            render_frame_flat(&M);   /* capture-only, no pose */
        }
        if (M.pv_enabled) render_preview(&M);   /* laptop mirror window */
        wl_display_flush(M.display);

        /* once-a-second render-rate readout (gated by capture + glasses vsync) */
        if (++fps_frames >= 60) {
            struct timespec now; clock_gettime(CLOCK_MONOTONIC, &now);
            double dt = (now.tv_sec - fps_t0.tv_sec) + (now.tv_nsec - fps_t0.tv_nsec) * 1e-9;
            if (dt >= 1.0) {
                fprintf(stderr, "mirage: %.1f fps\n", fps_frames / dt);
                fps_t0 = now; fps_frames = 0;
            }
        }
    }

    fprintf(stderr, "\nmirage: shutting down\n");
    pose_stop();
    capture_finish(&M);
    grab_destroy(&M);
    render_finish(&M);
    wl_display_disconnect(M.display);
    return 0;
}
