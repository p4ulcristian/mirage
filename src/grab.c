/* grab.c - Super+G pointer capture for mirage (libinput / evdev based).
 *
 * The pure-Wayland approach (lock the pointer + relative-pointer) can't work
 * here: Wayland has one cursor, and the moment we drive it onto a virtual
 * screen it leaves our surface and the lock breaks. So instead we grab the
 * trackpad at the device level:
 *
 *   1. open the trackpad via libinput (which does the messy multitouch ->
 *      pointer processing: motion, tap, two-finger scroll, acceleration),
 *   2. EVIOCGRAB it in open_restricted so the compositor stops seeing it
 *      (no more double cursor / no fighting over the seat cursor),
 *   3. accumulate the processed motion into one continuous strip spanning all
 *      the arc screens, and
 *   4. inject onto the right VIRT output via a per-output virtual pointer.
 *
 * Keyboard needs no capture: once the injected cursor lands on a window,
 * Hyprland focus-follows-mouse gives it keyboard focus, so typing just works.
 * We never grab the keyboard, so Super+G / Super+SHIFT+Q keep firing.
 */
#include "mirage.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include <math.h>
#include <fcntl.h>
#include <unistd.h>
#include <time.h>
#include <sys/ioctl.h>
#include <linux/input.h>
#include <libinput.h>

#include "wlr-virtual-pointer-unstable-v1-client-protocol.h"

#define GRAB_MAX MIRAGE_MAX_SCREENS

typedef struct {
    struct mirage *m;           /* back-ref for zoom etc.                 */
    bool active;
    bool super;                 /* Super held? (from the un-grabbed kbd)  */

    int   n;                    /* screen count                          */
    int   x0[GRAB_MAX];         /* cumulative left edge of each screen    */
    int   w[GRAB_MAX], h[GRAB_MAX];
    int   strip_w, strip_h;

    double gx, gy;              /* cursor position in strip space         */
    int    cur;                 /* screen the cursor is currently on      */
    float  sens;               /* motion scale                          */

    struct zwlr_virtual_pointer_v1 *vp[GRAB_MAX];
    struct libinput *li;        /* input, live only while active          */
    char   dev[64];             /* trackpad device path (grabbed)         */
    char   kbd[64];             /* keyboard device path (observed, not grabbed) */
} grab_state;

static uint32_t now_ms(void) {
    struct timespec ts; clock_gettime(CLOCK_MONOTONIC, &ts);
    return (uint32_t)(ts.tv_sec * 1000 + ts.tv_nsec / 1000000);
}
static grab_state *GS(struct mirage *m) { return (grab_state*)m->grab; }

/* ---- libinput device interface ----
 * The trackpad is opened with an exclusive grab (taken from the compositor);
 * the keyboard is opened WITHOUT a grab - we only observe it for the Super
 * modifier, so typing and compositor binds keep working. */
static int li_open(const char *path, int flags, void *u) {
    grab_state *g = u;
    int fd = open(path, flags);
    if (fd < 0) return -errno;
    bool is_kbd = g && !strcmp(path, g->kbd);
    if (!is_kbd) ioctl(fd, EVIOCGRAB, (void*)1);
    return fd;
}
static void li_close(int fd, void *u) {
    (void)u;
    ioctl(fd, EVIOCGRAB, (void*)0);
    close(fd);
}
static const struct libinput_interface LI_IFACE = {
    .open_restricted = li_open, .close_restricted = li_close,
};

/* Map the strip cursor to a screen + output-local pixel and inject it. */
static void push_cursor(grab_state *g) {
    if (g->gx < 0) g->gx = 0;
    if (g->gy < 0) g->gy = 0;
    if (g->gx > g->strip_w - 1) g->gx = g->strip_w - 1;
    if (g->gy > g->strip_h - 1) g->gy = g->strip_h - 1;

    int s = 0;
    while (s < g->n - 1 && g->gx >= g->x0[s + 1]) s++;
    g->cur = s;
    if (!g->vp[s]) return;

    uint32_t lx = (uint32_t)(g->gx - g->x0[s]);
    uint32_t ly = (uint32_t)g->gy;
    if (lx > (uint32_t)g->w[s] - 1) lx = g->w[s] - 1;
    if (ly > (uint32_t)g->h[s] - 1) ly = g->h[s] - 1;

    zwlr_virtual_pointer_v1_motion_absolute(g->vp[s], now_ms(), lx, ly,
                                            (uint32_t)g->w[s], (uint32_t)g->h[s]);
    zwlr_virtual_pointer_v1_frame(g->vp[s]);
}

static void do_zoom(grab_state *g, double scroll_v) {
    /* scroll up (negative v) zooms in; multiplicative for even feel */
    float z = g->m->zoom > 0.0f ? g->m->zoom : 1.0f;
    z *= expf((float)(-scroll_v) * 0.0015f);
    if (z < MIRAGE_ZOOM_MIN) z = MIRAGE_ZOOM_MIN;
    if (z > MIRAGE_ZOOM_MAX) z = MIRAGE_ZOOM_MAX;
    g->m->zoom = z;
}

static void handle_event(grab_state *g, struct libinput_event *ev) {
    enum libinput_event_type t = libinput_event_get_type(ev);
    switch (t) {
    case LIBINPUT_EVENT_KEYBOARD_KEY: {
        struct libinput_event_keyboard *k = libinput_event_get_keyboard_event(ev);
        uint32_t key = libinput_event_keyboard_get_key(k);
        if (key == KEY_LEFTMETA || key == KEY_RIGHTMETA)
            g->super = libinput_event_keyboard_get_key_state(k)
                       == LIBINPUT_KEY_STATE_PRESSED;
        break;
    }
    case LIBINPUT_EVENT_POINTER_MOTION: {
        struct libinput_event_pointer *p = libinput_event_get_pointer_event(ev);
        g->gx += libinput_event_pointer_get_dx(p) * g->sens;
        g->gy += libinput_event_pointer_get_dy(p) * g->sens;
        push_cursor(g);
        break;
    }
    case LIBINPUT_EVENT_POINTER_BUTTON: {
        struct libinput_event_pointer *p = libinput_event_get_pointer_event(ev);
        uint32_t btn = libinput_event_pointer_get_button(p);
        uint32_t st  = libinput_event_pointer_get_button_state(p)
                       == LIBINPUT_BUTTON_STATE_PRESSED ? 1u : 0u;
        if (g->vp[g->cur]) {
            zwlr_virtual_pointer_v1_button(g->vp[g->cur], now_ms(), btn, st);
            zwlr_virtual_pointer_v1_frame(g->vp[g->cur]);
        }
        break;
    }
    case LIBINPUT_EVENT_POINTER_SCROLL_FINGER:
    case LIBINPUT_EVENT_POINTER_SCROLL_WHEEL:
    case LIBINPUT_EVENT_POINTER_SCROLL_CONTINUOUS: {
        struct libinput_event_pointer *p = libinput_event_get_pointer_event(ev);
        enum libinput_pointer_axis ax = LIBINPUT_POINTER_AXIS_SCROLL_VERTICAL;
        if (!libinput_event_pointer_has_axis(p, ax)) break;
        double v = libinput_event_pointer_get_scroll_value(p, ax);
        if (g->super) {                  /* Super+scroll -> zoom (not forwarded) */
            do_zoom(g, v);
        } else if (g->vp[g->cur]) {      /* plain scroll -> forward to the window */
            zwlr_virtual_pointer_v1_axis(g->vp[g->cur], now_ms(),
                WL_POINTER_AXIS_VERTICAL_SCROLL, wl_fixed_from_double(v));
            zwlr_virtual_pointer_v1_frame(g->vp[g->cur]);
        }
        break;
    }
    default: break;
    }
}

/* Called every frame from the render loop: drain processed trackpad events. */
void grab_pump(struct mirage *m) {
    grab_state *g = GS(m);
    if (!g || !g->active || !g->li) return;
    if (libinput_dispatch(g->li) != 0) return;
    struct libinput_event *ev;
    while ((ev = libinput_get_event(g->li))) {
        handle_event(g, ev);
        libinput_event_destroy(ev);
    }
    wl_display_flush(m->display);
}

/* ---- lifecycle ---- */
bool grab_init(struct mirage *m) {
    if (!m->vpointer_mgr || !m->seat) {
        fprintf(stderr, "grab: virtual-pointer unavailable; Super+G disabled\n");
        return true;
    }
    grab_state *g = calloc(1, sizeof *g);
    m->grab = g;
    g->m = m;
    g->sens = 1.0f;
    const char *td = getenv("MIRAGE_TRACKPAD");
    const char *kb = getenv("MIRAGE_KEYBOARD");
    snprintf(g->dev, sizeof g->dev, "%s", td ? td : "/dev/input/event0");
    snprintf(g->kbd, sizeof g->kbd, "%s", kb ? kb : "/dev/input/event1");

    int x = 0;
    g->n = m->n_screen > GRAB_MAX ? GRAB_MAX : m->n_screen;
    for (int i = 0; i < g->n; i++) {
        screen_t *s = &m->screen[i];
        g->x0[i] = x; g->w[i] = s->width; g->h[i] = s->height;
        x += s->width;
        if (s->height > g->strip_h) g->strip_h = s->height;
        g->vp[i] = zwlr_virtual_pointer_manager_v1_create_virtual_pointer_with_output(
                       m->vpointer_mgr, m->seat, s->wl);
    }
    g->strip_w = x;
    g->gx = g->strip_w / 2.0; g->gy = g->strip_h / 2.0;
    fprintf(stderr, "grab: ready (%d screens, strip %dx%d, trackpad %s). "
            "Super+G to capture.\n", g->n, g->strip_w, g->strip_h, g->dev);
    return true;
}

void grab_toggle(struct mirage *m) {
    grab_state *g = GS(m);
    if (!g) return;
    if (!g->active) {
        g->li = libinput_path_create_context(&LI_IFACE, g);
        if (!g->li) { fprintf(stderr, "grab: libinput context failed\n"); return; }
        if (!libinput_path_add_device(g->li, g->dev)) {
            fprintf(stderr, "grab: cannot open trackpad %s (permission? wrong "
                    "device? set MIRAGE_TRACKPAD)\n", g->dev);
            libinput_unref(g->li); g->li = NULL; return;
        }
        /* observe (don't grab) the keyboard, for the Super+scroll zoom modifier */
        if (!libinput_path_add_device(g->li, g->kbd))
            fprintf(stderr, "grab: note - keyboard %s not observed; Super+scroll "
                    "zoom disabled\n", g->kbd);
        g->super = false;
        g->active = true;
        push_cursor(g);
        fprintf(stderr, "grab: capture ON (trackpad grabbed)\n");
    } else {
        if (g->li) { libinput_unref(g->li); g->li = NULL; }   /* ungrabs device */
        g->active = false;
        fprintf(stderr, "grab: capture OFF\n");
    }
    wl_display_flush(m->display);
}

bool grab_active(struct mirage *m) { grab_state *g = GS(m); return g && g->active; }

void grab_destroy(struct mirage *m) {
    grab_state *g = GS(m);
    if (!g) return;
    if (g->li) { libinput_unref(g->li); g->li = NULL; }
    for (int i = 0; i < g->n; i++)
        if (g->vp[i]) zwlr_virtual_pointer_v1_destroy(g->vp[i]);
    free(g);
    m->grab = NULL;
}
