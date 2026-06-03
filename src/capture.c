#include "mirage.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <fcntl.h>
#include <unistd.h>
#include <time.h>

#include <gbm.h>
#include <drm_fourcc.h>

#include "wlr-screencopy-unstable-v1-client-protocol.h"
#include "linux-dmabuf-unstable-v1-client-protocol.h"

/* EGL/GL extension entrypoints (resolved at init) */
static PFNEGLCREATEIMAGEKHRPROC            p_eglCreateImageKHR;
static PFNEGLDESTROYIMAGEKHRPROC           p_eglDestroyImageKHR;
static PFNGLEGLIMAGETARGETTEXTURE2DOESPROC p_glEGLImageTargetTexture2DOES;

/* Max anisotropy the driver supports (1 = unsupported/off). Captured screens are
 * minified ~1.13x head-on and far more on the side/top screens viewed at a
 * glancing angle, where an anisotropic footprint sharpens text the most. */
#ifndef GL_TEXTURE_MAX_ANISOTROPY_EXT
#define GL_TEXTURE_MAX_ANISOTROPY_EXT     0x84FE
#define GL_MAX_TEXTURE_MAX_ANISOTROPY_EXT 0x84FF
#endif
static float g_aniso = 1.0f;

static bool has_gl_ext(const char *name) {
    const char *e = (const char*)glGetString(GL_EXTENSIONS);
    return e && strstr(e, name) != NULL;
}

static int open_render_node(void) {
    /* render nodes are typically /dev/dri/renderD12N */
    for (int n = 128; n < 140; n++) {
        char path[64];
        snprintf(path, sizeof path, "/dev/dri/renderD%d", n);
        int fd = open(path, O_RDWR | O_CLOEXEC);
        if (fd >= 0) { fprintf(stderr, "capture: using %s\n", path); return fd; }
    }
    return -1;
}

bool capture_init(struct mirage *m) {
    m->drm_fd = open_render_node();
    if (m->drm_fd < 0) { fprintf(stderr, "capture: no DRM render node\n"); return false; }
    m->gbm = gbm_create_device(m->drm_fd);
    if (!m->gbm) { fprintf(stderr, "capture: gbm_create_device failed\n"); return false; }

    p_eglCreateImageKHR  = (void*)eglGetProcAddress("eglCreateImageKHR");
    p_eglDestroyImageKHR = (void*)eglGetProcAddress("eglDestroyImageKHR");
    p_glEGLImageTargetTexture2DOES =
        (void*)eglGetProcAddress("glEGLImageTargetTexture2DOES");
    if (!p_eglCreateImageKHR || !p_glEGLImageTargetTexture2DOES) {
        fprintf(stderr, "capture: dmabuf EGLImage import not available\n");
        return false;
    }

    if (has_gl_ext("GL_EXT_texture_filter_anisotropic")) {
        GLfloat amax = 1.0f;
        glGetFloatv(GL_MAX_TEXTURE_MAX_ANISOTROPY_EXT, &amax);
        g_aniso = amax > 8.0f ? 8.0f : amax;   /* 8x is plenty; diminishing past it */
        fprintf(stderr, "capture: anisotropic filtering up to %.0fx\n", g_aniso);
    }
    return true;
}

/* Allocate the persistent dmabuf-backed capture target for a screen, once we
 * know the format/size, and wrap it as a wl_buffer + GL texture. */
static bool ensure_buffer(struct mirage *m, screen_t *s) {
    if (s->buffer) return true;

    struct gbm_bo *bo = gbm_bo_create(m->gbm, s->width, s->height, s->drm_format,
                                      GBM_BO_USE_RENDERING | GBM_BO_USE_LINEAR);
    if (!bo) bo = gbm_bo_create(m->gbm, s->width, s->height, s->drm_format,
                                GBM_BO_USE_RENDERING);
    if (!bo) { fprintf(stderr, "capture[%s]: gbm_bo_create failed\n", s->name); return false; }

    s->bo        = bo;
    s->dmabuf_fd = gbm_bo_get_fd(bo);
    s->stride    = gbm_bo_get_stride(bo);
    s->modifier  = gbm_bo_get_modifier(bo);

    /* wl_buffer via linux-dmabuf */
    struct zwp_linux_buffer_params_v1 *params =
        zwp_linux_dmabuf_v1_create_params(m->dmabuf);
    zwp_linux_buffer_params_v1_add(params, s->dmabuf_fd, 0, 0, s->stride,
                                   (uint32_t)(s->modifier >> 32),
                                   (uint32_t)(s->modifier & 0xffffffff));
    s->buffer = zwp_linux_buffer_params_v1_create_immed(
        params, s->width, s->height, s->drm_format, 0);
    zwp_linux_buffer_params_v1_destroy(params);
    if (!s->buffer) { fprintf(stderr, "capture[%s]: create wl_buffer failed\n", s->name); return false; }

    /* EGLImage from the same dmabuf, bound to a GL texture */
    EGLint attrs[32]; int a = 0;
    attrs[a++] = EGL_WIDTH;                     attrs[a++] = s->width;
    attrs[a++] = EGL_HEIGHT;                    attrs[a++] = s->height;
    attrs[a++] = EGL_LINUX_DRM_FOURCC_EXT;      attrs[a++] = (EGLint)s->drm_format;
    attrs[a++] = EGL_DMA_BUF_PLANE0_FD_EXT;     attrs[a++] = s->dmabuf_fd;
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
        fprintf(stderr, "capture[%s]: eglCreateImageKHR failed (fmt %.4s mod %llx)\n",
                s->name, (char*)&s->drm_format, (unsigned long long)s->modifier);
        return false;
    }
    glGenTextures(1, &s->tex);
    glBindTexture(GL_TEXTURE_2D, s->tex);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
    if (g_aniso > 1.0f)
        glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_MAX_ANISOTROPY_EXT, g_aniso);
    p_glEGLImageTargetTexture2DOES(GL_TEXTURE_2D, s->image);

    fprintf(stderr, "capture[%s]: %dx%d fmt %.4s mod %llx stride %u -> tex %u\n",
            s->name, s->width, s->height, (char*)&s->drm_format,
            (unsigned long long)s->modifier, s->stride, s->tex);
    return true;
}

/* ---- screencopy frame listener ---- */
static void on_buffer(void *data, struct zwlr_screencopy_frame_v1 *f,
                      uint32_t format, uint32_t w, uint32_t h, uint32_t stride) {
    (void)f; (void)format; (void)stride;
    screen_t *s = data;
    if (s->width == 0)  s->width = (int32_t)w;
    if (s->height == 0) s->height = (int32_t)h;
}

static void on_dmabuf(void *data, struct zwlr_screencopy_frame_v1 *f,
                      uint32_t format, uint32_t w, uint32_t h) {
    (void)f;
    screen_t *s = data;
    s->drm_format = format;
    s->width  = (int32_t)w;
    s->height = (int32_t)h;
}

/* mirage instance, stashed so the frame listener can reach gbm/egl */
struct mirage *g_capture_mirage = NULL;

/* "Hot" screens: ones that changed in the last CAPTURE_HOT_MS. They get a
 * full-rate plain copy so the screen you're typing on is always current -
 * copy_with_damage reveals a change one damage-event late (the "one character
 * late" lag). Idle screens (no recent damage) fall back to damage copies and
 * cost ~0. Keyed by screen index, refreshed from the damage callback, so it
 * works whether or not you're in Super+G grab mode. */
#define CAPTURE_HOT_MS 800
static uint64_t g_hot_until_ms[MIRAGE_MAX_SCREENS];

static uint64_t now_ms(void) {
    struct timespec ts; clock_gettime(CLOCK_MONOTONIC, &ts);
    return (uint64_t)ts.tv_sec * 1000 + ts.tv_nsec / 1000000;
}
static bool screen_hot(const screen_t *s) {
    return s->index >= 0 && s->index < MIRAGE_MAX_SCREENS
           && now_ms() < g_hot_until_ms[s->index];
}

static void on_buffer_done(void *data, struct zwlr_screencopy_frame_v1 *f) {
    screen_t *s = data;
    if (!s->drm_format) {           /* compositor offered no dmabuf format */
        s->frame_state = 3;
        return;
    }
    if (!ensure_buffer(g_capture_mirage, s)) { s->frame_state = 3; return; }
    /* A screen that changed recently (hot), or the one the captured cursor is
     * on, gets a plain copy EVERY frame so the active window is always perfectly
     * current - copy_with_damage can only reveal a change one damage-event
     * later, which shows as the "one character late" lag on whatever you're
     * typing into. Plain copy always completes, so with the immediate re-arm a
     * hot screen captures at full rate. Damage events keep it hot while you type
     * and stop ~CAPTURE_HOT_MS after, so it auto-promotes with NO dependence on
     * grab mode.
     *
     * Every other (idle) screen uses copy_with_damage: it only completes (ready)
     * when the output actually changes, so static background screens cost zero
     * compositor render + zero blit. GPU load scales with on-screen activity. */
    bool active = screen_hot(s) || s->index == grab_cursor_screen(g_capture_mirage);
    if (s->have_tex && !active)
        zwlr_screencopy_frame_v1_copy_with_damage(f, s->buffer);
    else
        zwlr_screencopy_frame_v1_copy(f, s->buffer);
}

static void on_flags(void *data, struct zwlr_screencopy_frame_v1 *f, uint32_t flags) {
    (void)f;
    screen_t *s = data;
    s->y_invert = (flags & ZWLR_SCREENCOPY_FRAME_V1_FLAGS_Y_INVERT) != 0;
}

static void on_ready(void *data, struct zwlr_screencopy_frame_v1 *f,
                     uint32_t sec_hi, uint32_t sec_lo, uint32_t nsec) {
    (void)f; (void)sec_hi; (void)sec_lo; (void)nsec;
    screen_t *s = data;
    s->frame_state = 2;
}

static void on_failed(void *data, struct zwlr_screencopy_frame_v1 *f) {
    (void)f;
    screen_t *s = data;
    s->frame_state = 3;
}

static void on_damage(void *d, struct zwlr_screencopy_frame_v1 *f,
                      uint32_t x, uint32_t y, uint32_t w, uint32_t h) {
    (void)f;(void)x;(void)y;(void)w;(void)h;
    /* this screen just changed -> keep it on full-rate plain copy briefly, so
     * the next change shows immediately instead of one damage-event late. */
    screen_t *s = d;
    if (s->index >= 0 && s->index < MIRAGE_MAX_SCREENS)
        g_hot_until_ms[s->index] = now_ms() + CAPTURE_HOT_MS;
}

static const struct zwlr_screencopy_frame_v1_listener FRAME_LISTENER = {
    .buffer       = on_buffer,
    .flags        = on_flags,
    .ready        = on_ready,
    .failed       = on_failed,
    .damage       = on_damage,
    .linux_dmabuf = on_dmabuf,
    .buffer_done  = on_buffer_done,
};

/* Arm a fresh screencopy frame for one screen. overlay_cursor=1 composites the
 * cursor into the copy, so the pointer mirage injects onto a virtual screen is
 * visible on the glasses. Once a frame is armed it sits in flight (costing
 * nothing for copy_with_damage) until the output next changes. */
static void arm_frame(struct mirage *m, screen_t *s) {
    g_capture_mirage = m;
    s->frame_state = 1;
    s->frame = zwlr_screencopy_manager_v1_capture_output(m->screencopy, 1, s->wl);
    zwlr_screencopy_frame_v1_add_listener(s->frame, &FRAME_LISTENER, s);
}

void capture_begin_frame(struct mirage *m) {
    g_capture_mirage = m;
    for (int i = 0; i < m->n_screen; i++) {
        screen_t *s = &m->screen[i];
        if (s->frame) continue;              /* still in flight / re-armed */
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
            if (ready) {                     /* ready: contents updated */
                s->have_tex = true;
                if (s->image != EGL_NO_IMAGE_KHR) {
                    glBindTexture(GL_TEXTURE_2D, s->tex);
                    p_glEGLImageTargetTexture2DOES(GL_TEXTURE_2D, s->image);
                }
            }
            zwlr_screencopy_frame_v1_destroy(s->frame);
            s->frame = NULL;
            s->frame_state = 0;
            /* Re-arm immediately after a successful capture so a copy_with_damage
             * frame is always in flight to catch the NEXT change. Leaving the
             * screen un-armed until the next ~16ms tick opened a blind spot where
             * a keystroke's damage was missed and only surfaced on the following
             * keystroke - the "one letter behind" lag. A re-armed damage frame is
             * free while idle, so this keeps cc7b371's savings. On failure we
             * leave it NULL so capture_begin_frame re-arms on the throttled tick,
             * avoiding a busy loop on a persistently failing output. */
            if (ready) arm_frame(m, s);
        }
    }
    return all_settled;
}

void capture_finish(struct mirage *m) {
    for (int i = 0; i < m->n_screen; i++) {
        screen_t *s = &m->screen[i];
        if (s->frame)  zwlr_screencopy_frame_v1_destroy(s->frame);
        if (s->image && p_eglDestroyImageKHR) p_eglDestroyImageKHR(m->edpy, s->image);
        if (s->tex)    glDeleteTextures(1, &s->tex);
        if (s->buffer) wl_buffer_destroy(s->buffer);
        if (s->bo)     gbm_bo_destroy(s->bo);
        if (s->dmabuf_fd > 0) close(s->dmabuf_fd);
    }
    if (m->gbm)    gbm_device_destroy(m->gbm);
    if (m->drm_fd >= 0) close(m->drm_fd);
}
