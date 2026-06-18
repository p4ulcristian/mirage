#include "mirage.h"
#include "handle.hpp"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <fcntl.h>
#include <unistd.h>
#include <time.h>

#include <gbm.h>
#include <drm_fourcc.h>

#include <print>
#include <string_view>

#include "ext-image-copy-capture-v1-client-protocol.h"
#include "ext-image-capture-source-v1-client-protocol.h"
#include "linux-dmabuf-unstable-v1-client-protocol.h"

/* ext-image-copy-capture-v1 capture path. Replaces the deprecated wlr-screencopy:
 * the compositor copies only DAMAGED regions into our persistent dmabuf each
 * frame (no full-output re-blit when the desktop is mostly static), and the
 * import is zero-copy via EGLImage exactly as before. One persistent session
 * per output advertises the buffer constraints (size/format/modifiers); we
 * allocate one matching dmabuf and re-arm a frame against it every tick. */

/* EGL/GL extension entrypoints (resolved at init) */
static PFNEGLCREATEIMAGEKHRPROC            p_eglCreateImageKHR;
static PFNEGLDESTROYIMAGEKHRPROC           p_eglDestroyImageKHR;
static PFNGLEGLIMAGETARGETTEXTURE2DOESPROC p_glEGLImageTargetTexture2DOES;

#ifndef GL_TEXTURE_MAX_ANISOTROPY_EXT
#define GL_TEXTURE_MAX_ANISOTROPY_EXT     0x84FE
#define GL_MAX_TEXTURE_MAX_ANISOTROPY_EXT 0x84FF
#endif
static float g_aniso = 1.0f;

/* Buffer constraints advertised by each output's capture session. Kept here so
 * mirage.h stays free of protocol-specific layout. Keyed by screen index. */
#define MAX_MODS 64
static struct con {
    uint32_t cap_w, cap_h;          /* buffer size the compositor wants     */
    uint32_t fmt;                   /* chosen DRM fourcc                     */
    uint64_t mods[MAX_MODS];        /* modifiers offered for that fourcc     */
    int      n_mods;
    bool     got_size, got_fmt;
    /* Startup race: we create all sessions at once, but an output whose swapchain
     * isn't composited yet advertises an empty format. Rather than give up on it,
     * re-create the session a few times until a real format arrives. */
    bool     empty_fmt;             /* last `done` had a 0 fourcc            */
    int      retry_ticks, retries;  /* throttle + cap the re-session loop    */
} g_con[MIRAGE_MAX_SCREENS];

static bool has_gl_ext(const char *name) {
    const char *e = (const char*)glGetString(GL_EXTENSIONS);
    return e && strstr(e, name) != NULL;
}

static int open_render_node(void) {
    for (int n = 128; n < 140; n++) {
        char path[64];
        snprintf(path, sizeof path, "/dev/dri/renderD%d", n);
        int fd = open(path, O_RDWR | O_CLOEXEC);
        if (fd >= 0) { std::print(stderr, "capture: using {}\n", path); return fd; }
    }
    return -1;
}

/* ---- session listener: collect buffer constraints ---- */
static void se_buffer_size(void *d, struct ext_image_copy_capture_session_v1 *s,
                           uint32_t w, uint32_t h) {
    (void)s; struct con *c = &g_con[((screen_t*)d)->index];
    c->cap_w = w; c->cap_h = h; c->got_size = true;
}
static void se_shm_format(void *d, struct ext_image_copy_capture_session_v1 *s, uint32_t f) {
    (void)d;(void)s;(void)f;   /* we only use the dmabuf path */
}
static void se_dmabuf_device(void *d, struct ext_image_copy_capture_session_v1 *s,
                             struct wl_array *dev) {
    (void)d;(void)s;(void)dev; /* single-GPU: our render node is the same device */
}
static void se_dmabuf_format(void *d, struct ext_image_copy_capture_session_v1 *s,
                             uint32_t format, struct wl_array *mods) {
    (void)s; struct con *c = &g_con[((screen_t*)d)->index];
    /* Prefer opaque XRGB8888; otherwise take the first format offered. The
     * compositor may send this event once per supported format. */
    bool prefer = (format == DRM_FORMAT_XRGB8888);
    if (c->got_fmt && c->fmt == DRM_FORMAT_XRGB8888 && !prefer) return;
    if (c->got_fmt && !prefer) return;
    c->fmt = format;
    c->n_mods = 0;
    const uint64_t *p = (const uint64_t*)mods->data;
    int n = (int)(mods->size / sizeof(uint64_t));
    for (int i = 0; i < n && c->n_mods < MAX_MODS; i++) c->mods[c->n_mods++] = p[i];
    c->got_fmt = true;
}
static void se_done(void *d, struct ext_image_copy_capture_session_v1 *s) {
    (void)s; screen_t *sc = (screen_t*)d; struct con *c = &g_con[sc->index];
    /* Ready only with a size AND a usable (non-zero) dmabuf format. Some headless
     * outputs occasionally come up advertising an empty format (fourcc 0, no
     * modifiers) - it can't be imported as a dmabuf, and arming it would attach an
     * invalid buffer, which the compositor punishes with a fatal protocol error
     * that kills the WHOLE client. So we leave that one screen un-ready (it shows
     * its placeholder); the others, and the rest of the app, carry on. */
    if (c->got_size && c->got_fmt && c->fmt != 0) {
        sc->session_ready = true;
        c->empty_fmt = false;
        std::print(stderr, "capture[{}]: session ready {}x{} fmt {} ({} modifiers)\n",
                sc->name, c->cap_w, c->cap_h,
                std::string_view((const char*)&c->fmt, 4), c->n_mods);
    } else if (c->got_size && c->got_fmt && c->fmt == 0) {
        /* Empty format - flag it so capture_begin_frame re-creates the session
         * once the output has actually composited a frame (see resession). */
        c->empty_fmt = true;
        if (c->retries == 0)
            std::print(stderr, "capture[{}]: empty dmabuf format (output not composited "
                    "yet); will retry the session\n", sc->name);
    }
}
static void se_stopped(void *d, struct ext_image_copy_capture_session_v1 *s) {
    (void)s; screen_t *sc = (screen_t*)d; sc->session_ready = false;
    std::print(stderr, "capture[{}]: session stopped\n", sc->name);
}
static const struct ext_image_copy_capture_session_v1_listener SESSION_LISTENER = {
    .buffer_size   = se_buffer_size,
    .shm_format    = se_shm_format,
    .dmabuf_device = se_dmabuf_device,
    .dmabuf_format = se_dmabuf_format,
    .done          = se_done,
    .stopped       = se_stopped,
};

/* Allocate the persistent dmabuf-backed capture target once the session has
 * advertised size + format + modifiers, and wrap it as wl_buffer + GL texture. */
static bool ensure_buffer(struct mirage *m, screen_t *s) {
    if (s->buffer) return true;
    struct con *c = &g_con[s->index];

    /* Allocate honouring the compositor's advertised modifiers (so it can import
     * our buffer for a tiled, zero-copy damage copy). Fall back to a plain
     * RENDERING bo, then LINEAR, if the modifier set can't be satisfied. */
    struct gbm_bo *raw_bo = NULL;
    if (c->n_mods > 0) {
        /* drop DRM_FORMAT_MOD_INVALID from the list - mixing it with real
         * modifiers is rejected by gbm. */
        uint64_t real[MAX_MODS]; int nr = 0;
        for (int i = 0; i < c->n_mods; i++)
            if (c->mods[i] != DRM_FORMAT_MOD_INVALID) real[nr++] = c->mods[i];
        if (nr > 0)
            raw_bo = gbm_bo_create_with_modifiers(m->gbm, c->cap_w, c->cap_h, c->fmt,
                                                  real, nr);
    }
    if (!raw_bo) raw_bo = gbm_bo_create(m->gbm, c->cap_w, c->cap_h, c->fmt, GBM_BO_USE_RENDERING);
    if (!raw_bo) raw_bo = gbm_bo_create(m->gbm, c->cap_w, c->cap_h, c->fmt,
                                        GBM_BO_USE_RENDERING | GBM_BO_USE_LINEAR);
    if (!raw_bo) { std::print(stderr, "capture[{}]: gbm_bo_create failed\n", s->name); return false; }

    /* From here on every resource lives in a move-only RAII local: any early
     * return below frees the partial buffer automatically, so the reuse guard at
     * the top can never hand back a half-built buffer (the bug that once attached
     * an invalid buffer and killed the whole client). They're released into the
     * screen only on the success path; capture_finish still frees them there. */
    own::GbmBo bo(raw_bo);
    own::Fd    fd(gbm_bo_get_fd(bo.get()));

    s->y_invert   = false;  /* upright by default; fr_transform flips only for 180 */
    s->width      = (int32_t)c->cap_w;     /* buffer size is authoritative */
    s->height     = (int32_t)c->cap_h;
    s->drm_format = c->fmt;
    s->stride     = gbm_bo_get_stride(bo.get());
    s->modifier   = gbm_bo_get_modifier(bo.get());

    /* wl_buffer via linux-dmabuf */
    struct zwp_linux_buffer_params_v1 *params =
        zwp_linux_dmabuf_v1_create_params(m->dmabuf);
    zwp_linux_buffer_params_v1_add(params, fd.get(), 0, 0, s->stride,
                                   (uint32_t)(s->modifier >> 32),
                                   (uint32_t)(s->modifier & 0xffffffff));
    own::WlBuffer buffer(zwp_linux_buffer_params_v1_create_immed(
        params, s->width, s->height, s->drm_format, 0));
    zwp_linux_buffer_params_v1_destroy(params);
    if (!buffer) {
        std::print(stderr, "capture[{}]: create wl_buffer failed\n", s->name);
        return false;   /* bo + fd auto-freed */
    }

    /* EGLImage from the same dmabuf, bound to a GL texture */
    EGLint attrs[32]; int a = 0;
    attrs[a++] = EGL_WIDTH;                     attrs[a++] = s->width;
    attrs[a++] = EGL_HEIGHT;                    attrs[a++] = s->height;
    attrs[a++] = EGL_LINUX_DRM_FOURCC_EXT;      attrs[a++] = (EGLint)s->drm_format;
    attrs[a++] = EGL_DMA_BUF_PLANE0_FD_EXT;     attrs[a++] = fd.get();
    attrs[a++] = EGL_DMA_BUF_PLANE0_OFFSET_EXT; attrs[a++] = 0;
    attrs[a++] = EGL_DMA_BUF_PLANE0_PITCH_EXT;  attrs[a++] = (EGLint)s->stride;
    if (s->modifier != DRM_FORMAT_MOD_INVALID) {
        attrs[a++] = EGL_DMA_BUF_PLANE0_MODIFIER_LO_EXT;
        attrs[a++] = (EGLint)(s->modifier & 0xffffffff);
        attrs[a++] = EGL_DMA_BUF_PLANE0_MODIFIER_HI_EXT;
        attrs[a++] = (EGLint)(s->modifier >> 32);
    }
    attrs[a++] = EGL_NONE;

    s->image = p_eglCreateImageKHR(m->edpy, EGL_NO_CONTEXT,
                                   EGL_LINUX_DMA_BUF_EXT, NULL, attrs);
    if (s->image == EGL_NO_IMAGE_KHR) {
        std::print(stderr, "capture[{}]: eglCreateImageKHR failed (fmt {} mod {:x})\n",
                s->name, std::string_view((const char*)&s->drm_format, 4),
                (unsigned long long)s->modifier);
        return false;   /* bo + fd + buffer auto-freed */
    }

    /* Success: hand the now-complete buffer set to the screen (capture_finish
     * frees it). release() relinquishes ownership so the locals' destructors
     * leave it alone. */
    s->bo        = bo.release();
    s->dmabuf_fd = fd.release();
    s->buffer    = buffer.release();

    glGenTextures(1, &s->tex);
    glBindTexture(GL_TEXTURE_2D, s->tex);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
    if (g_aniso > 1.0f)
        glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_MAX_ANISOTROPY_EXT, g_aniso);
    p_glEGLImageTargetTexture2DOES(GL_TEXTURE_2D, s->image);

    std::print(stderr, "capture[{}]: {}x{} fmt {} mod {:x} stride {} -> tex {}\n",
            s->name, s->width, s->height,
            std::string_view((const char*)&s->drm_format, 4),
            (unsigned long long)s->modifier, s->stride, s->tex);
    return true;
}

/* ---- frame listener ---- */
static void fr_transform(void *d, struct ext_image_copy_capture_frame_v1 *f, uint32_t t) {
    (void)f; screen_t *s = (screen_t*)d;
    /* ext-image-copy-capture delivers the buffer already upright (NORMAL), unlike
     * wlr-screencopy's bottom-up Y_INVERT convention - so DON'T flip for NORMAL;
     * only a 180 / flipped-180 transform needs the vertical flip. */
    s->y_invert = (t == WL_OUTPUT_TRANSFORM_180 || t == WL_OUTPUT_TRANSFORM_FLIPPED_180);
}
static void fr_damage(void *d, struct ext_image_copy_capture_frame_v1 *f,
                      int32_t x, int32_t y, int32_t w, int32_t h) {
    (void)d;(void)f;(void)x;(void)y;(void)w;(void)h;   /* compositor wrote it for us */
}
static void fr_presentation_time(void *d, struct ext_image_copy_capture_frame_v1 *f,
                                 uint32_t shi, uint32_t slo, uint32_t ns) {
    (void)d;(void)f;(void)shi;(void)slo;(void)ns;
}
static void fr_ready(void *d, struct ext_image_copy_capture_frame_v1 *f) {
    (void)f; ((screen_t*)d)->frame_state = 2;
}
static void fr_failed(void *d, struct ext_image_copy_capture_frame_v1 *f, uint32_t reason) {
    (void)f; screen_t *s = (screen_t*)d; s->frame_state = 3;
    static int logged[MIRAGE_MAX_SCREENS] = {0};
    if (s->index >= 0 && s->index < MIRAGE_MAX_SCREENS && !logged[s->index]++)
        std::print(stderr, "capture[{}]: frame FAILED reason={}\n", s->name, reason);
    if (reason == EXT_IMAGE_COPY_CAPTURE_FRAME_V1_FAILURE_REASON_BUFFER_CONSTRAINTS) {
        /* our buffer no longer matches; drop it so it's re-allocated from fresh
         * constraints on the next session done. */
        std::print(stderr, "capture[{}]: buffer constraints changed, re-allocating\n", s->name);
        s->session_ready = false;
    }
}
static const struct ext_image_copy_capture_frame_v1_listener FRAME_LISTENER = {
    .transform          = fr_transform,
    .damage             = fr_damage,
    .presentation_time  = fr_presentation_time,
    .ready              = fr_ready,
    .failed             = fr_failed,
};

/* Arm a fresh capture frame for one screen against its persistent buffer. */
static void arm_frame(struct mirage *m, screen_t *s) {
    if (!ensure_buffer(m, s)) { s->frame_state = 3; return; }
    s->frame = ext_image_copy_capture_session_v1_create_frame(s->session);
    ext_image_copy_capture_frame_v1_add_listener(s->frame, &FRAME_LISTENER, s);
    ext_image_copy_capture_frame_v1_attach_buffer(s->frame, s->buffer);
    /* Damage the whole buffer every frame: a headless output Hyprland renders
     * lazily only delivers content when we declare our buffer fully stale (else
     * the first un-rendered capture stays black forever). This matches what
     * wlr-screencopy did with a full copy each frame; cheap on 0.55. */
    ext_image_copy_capture_frame_v1_damage_buffer(s->frame, 0, 0, s->width, s->height);
    ext_image_copy_capture_frame_v1_capture(s->frame);
    s->frame_state = 1;
}

mirage_status capture_init(struct mirage *m) {
    m->drm_fd = open_render_node();
    if (m->drm_fd < 0) return std::unexpected("no DRM render node");
    m->gbm = gbm_create_device(m->drm_fd);
    if (!m->gbm) return std::unexpected("gbm_create_device failed");

    p_eglCreateImageKHR  = (PFNEGLCREATEIMAGEKHRPROC)eglGetProcAddress("eglCreateImageKHR");
    p_eglDestroyImageKHR = (PFNEGLDESTROYIMAGEKHRPROC)eglGetProcAddress("eglDestroyImageKHR");
    p_glEGLImageTargetTexture2DOES =
        (PFNGLEGLIMAGETARGETTEXTURE2DOESPROC)eglGetProcAddress("glEGLImageTargetTexture2DOES");
    if (!p_eglCreateImageKHR || !p_glEGLImageTargetTexture2DOES)
        return std::unexpected("dmabuf EGLImage import not available");
    if (!m->capture_src_mgr || !m->copy_capture_mgr)
        return std::unexpected("ext-image-copy-capture not advertised "
                               "(needs Hyprland 0.52+/KWin 6.6+/Mutter 49+)");

    if (has_gl_ext("GL_EXT_texture_filter_anisotropic")) {
        GLfloat amax = 1.0f;
        glGetFloatv(GL_MAX_TEXTURE_MAX_ANISOTROPY_EXT, &amax);
        /* Use the hardware max, not a 2x cap. The desktops are minified hard onto
         * the arc (a 2160-tall portrait into ~23deg especially), and anisotropic
         * sampling averages along that steep minification axis - the single biggest
         * lever against the shimmer/sparkle on text as the head moves. It works off
         * the base level here (these are EGLImage externals with no mip chain), so a
         * higher cap = more taps = less aliasing, for free. */
        g_aniso = amax;
        std::print(stderr, "capture: anisotropic filtering up to {:.0f}x\n", g_aniso);
    }

    /* One persistent capture session per virtual output. Constraints arrive
     * asynchronously (buffer_size/dmabuf_format/done); the main loop's dispatch
     * fills them in and flips session_ready. paint_cursors composites the desktop
     * pointer into the capture so it's visible on the wall. */
    for (int i = 0; i < m->n_screen; i++) {
        screen_t *s = &m->screen[i];
        struct ext_image_capture_source_v1 *src =
            ext_output_image_capture_source_manager_v1_create_source(m->capture_src_mgr, s->wl);
        s->session = ext_image_copy_capture_manager_v1_create_session(
            m->copy_capture_mgr, src,
            EXT_IMAGE_COPY_CAPTURE_MANAGER_V1_OPTIONS_PAINT_CURSORS);
        ext_image_capture_source_v1_destroy(src);   /* session keeps its own ref */
        ext_image_copy_capture_session_v1_add_listener(s->session, &SESSION_LISTENER, s);
    }
    return {};
}

/* Tear down and re-create one output's capture session from scratch. Used when a
 * session came up with an empty format (its output wasn't composited yet at
 * startup): a fresh session re-queries the constraints, and by now the swapchain
 * exists, so the compositor advertises a real format. Resets the cached con. */
static void resession(struct mirage *m, screen_t *s) {
    struct con *c = &g_con[s->index];
    if (s->frame)   { ext_image_copy_capture_frame_v1_destroy(s->frame);     s->frame = NULL; }
    if (s->session) { ext_image_copy_capture_session_v1_destroy(s->session); s->session = NULL; }
    c->got_size = c->got_fmt = c->empty_fmt = false;
    c->fmt = 0; c->n_mods = 0;
    s->frame_state = 0;

    struct ext_image_capture_source_v1 *src =
        ext_output_image_capture_source_manager_v1_create_source(m->capture_src_mgr, s->wl);
    s->session = ext_image_copy_capture_manager_v1_create_session(
        m->copy_capture_mgr, src,
        EXT_IMAGE_COPY_CAPTURE_MANAGER_V1_OPTIONS_PAINT_CURSORS);
    ext_image_capture_source_v1_destroy(src);
    ext_image_copy_capture_session_v1_add_listener(s->session, &SESSION_LISTENER, s);
}

void capture_begin_frame(struct mirage *m) {
    for (int i = 0; i < m->n_screen; i++) {
        screen_t *s = &m->screen[i];
        struct con *c = &g_con[i];
        /* An output that advertised an empty format: re-create its session every
         * ~half second (this runs on the throttled capture tick), up to a cap, until
         * a real format arrives. Cheap, and it self-heals the startup race. */
        if (c->empty_fmt && !s->session_ready) {
            if (++c->retry_ticks >= 30 && c->retries < 40) {
                c->retry_ticks = 0; c->retries++;
                std::print(stderr, "capture[{}]: re-creating session (retry {})\n",
                        s->name, c->retries);
                resession(m, s);
            }
            continue;
        }
        if (!s->session_ready || s->frame) continue;   /* not ready / in flight */
        arm_frame(m, s);
    }
}

bool capture_poll(struct mirage *m) {
    bool all_settled = true;
    for (int i = 0; i < m->n_screen; i++) {
        screen_t *s = &m->screen[i];
        if (s->frame_state == 1) { all_settled = false; continue; }
        if (s->frame) {
            bool ready = s->frame_state == 2;
            if (ready) {
                s->have_tex = true;
                s->tex_dirty = true;   /* render rebuilds the mip chain from the new frame */
                if (s->image != EGL_NO_IMAGE_KHR) {
                    glBindTexture(GL_TEXTURE_2D, s->tex);
                    p_glEGLImageTargetTexture2DOES(GL_TEXTURE_2D, s->image);
                }
            }
            ext_image_copy_capture_frame_v1_destroy(s->frame);
            s->frame = NULL;
            s->frame_state = 0;
            /* Re-arm immediately after a successful capture so the next damage is
             * caught without a tick of latency. On failure leave it un-armed so
             * capture_begin_frame retries on the throttled tick (no busy loop). */
            if (ready) arm_frame(m, s);
        }
    }
    return all_settled;
}

void capture_finish(struct mirage *m) {
    for (int i = 0; i < m->n_screen; i++) {
        screen_t *s = &m->screen[i];
        if (s->frame)   ext_image_copy_capture_frame_v1_destroy(s->frame);
        if (s->session) ext_image_copy_capture_session_v1_destroy(s->session);
        if (s->image && p_eglDestroyImageKHR) p_eglDestroyImageKHR(m->edpy, s->image);
        if (s->tex)     glDeleteTextures(1, &s->tex);
        if (s->mip_tex) glDeleteTextures(1, &s->mip_tex);
        if (s->buffer)  wl_buffer_destroy(s->buffer);
        if (s->bo)      gbm_bo_destroy(s->bo);
        if (s->dmabuf_fd > 0) close(s->dmabuf_fd);
    }
    if (m->gbm)    gbm_device_destroy(m->gbm);
    if (m->drm_fd >= 0) close(m->drm_fd);
}
