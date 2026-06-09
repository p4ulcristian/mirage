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
 * Scroll is the exception: Hyprland does NOT reliably deliver zwlr_virtual_
 * pointer *axis* events to clients (Qt/Electron/Firefox ignore them, though
 * clicks/motion from the same virtual pointer work - see Hyprland #8931). So
 * scroll is injected as a REAL wheel via a kernel uinput device, which every
 * app honours like a physical mouse, at the cursor the virtual pointer placed.
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
#include <linux/uinput.h>
#include <libinput.h>

#include "wlr-virtual-pointer-unstable-v1-client-protocol.h"

#define GRAB_MAX MIRAGE_MAX_SCREENS
#define MAX_KBD  8           /* how many keyboards we observe for the Super key */

typedef struct {
    struct mirage *m;           /* back-ref for zoom etc.                 */
    bool active;
    bool super;                 /* Super held? (from the un-grabbed kbd)  */

    int   n;                    /* screen count                          */
    int   x0[GRAB_MAX], y0[GRAB_MAX];  /* top-left of each screen's grid cell */
    int   w[GRAB_MAX], h[GRAB_MAX];
    int   cols, rows, cellw, cellh;    /* cursor grid (matches the visual wall) */
    int   strip_w, strip_h;

    double gx, gy;              /* cursor position in strip space         */
    int    cur;                 /* screen the cursor is currently on      */
    float  sens;               /* motion scale                          */
    float  gaze_gain;          /* scales gaze delta -> cursor delta (1 = 1:1)     */
    double gaze_prev_gx, gaze_prev_gy;  /* last frame's gaze strip point          */
    bool   gaze_prev_valid;    /* false until the first gaze sample is seeded     */
    uint32_t last_alt_ms;      /* last Alt press (for double-tap gaze toggle) */
    double scroll_acc;         /* accumulated trackpad delta -> wheel notches */
    int    swipe_fingers;      /* finger count of the in-progress swipe gesture */
    double swipe_dx, swipe_dy; /* accumulated swipe travel (3-finger workspace swipe) */

    struct zwlr_virtual_pointer_v1 *vp[GRAB_MAX];
    struct libinput *li;        /* input, live only while active          */
    char   dev[64];             /* trackpad device path (grabbed)         */
    char   kbd[MAX_KBD][64];    /* keyboard device paths (observed, not grabbed) */
    int    n_kbd;               /* how many entries of kbd[] are valid    */
    int    uifd;                /* uinput wheel device fd (-1 if unavailable) */
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
    bool is_kbd = false;
    if (g) for (int i = 0; i < g->n_kbd; i++)
        if (!strcmp(path, g->kbd[i])) { is_kbd = true; break; }
    if (!is_kbd) ioctl(fd, EVIOCGRAB, (void*)1);   /* grab the trackpad, observe keyboards */
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

/* ---- uinput wheel device ----
 * A minimal virtual mouse that emits only wheel events. We route scroll here
 * instead of through the Wayland virtual pointer because Hyprland drops virtual-
 * pointer axis events for many clients (see the file header). A real wheel event
 * lands on whatever surface the seat cursor is over - which our virtual pointer
 * has already positioned on the target window. */
static int uinput_open(void) {
    int fd = open("/dev/uinput", O_WRONLY | O_NONBLOCK);
    if (fd < 0) {
        fprintf(stderr, "grab: scroll disabled - open /dev/uinput: %s "
                "(add yourself to the device ACL / 'input' group)\n", strerror(errno));
        return -1;
    }
    ioctl(fd, UI_SET_EVBIT, EV_KEY);   /* a button bit so it registers as a mouse */
    ioctl(fd, UI_SET_KEYBIT, BTN_LEFT);
    ioctl(fd, UI_SET_EVBIT, EV_REL);
    ioctl(fd, UI_SET_RELBIT, REL_WHEEL);
    ioctl(fd, UI_SET_RELBIT, REL_WHEEL_HI_RES);
    struct uinput_setup us = {0};
    us.id.bustype = BUS_USB;
    us.id.vendor  = 0x1d6b;            /* arbitrary; "Linux Foundation" */
    us.id.product = 0x5c01;
    snprintf(us.name, sizeof us.name, "mirage virtual scroll");
    if (ioctl(fd, UI_DEV_SETUP, &us) < 0 || ioctl(fd, UI_DEV_CREATE) < 0) {
        fprintf(stderr, "grab: scroll disabled - uinput setup: %s\n", strerror(errno));
        close(fd);
        return -1;
    }
    return fd;
}

static void uinput_close(int fd) {
    if (fd < 0) return;
    ioctl(fd, UI_DEV_DESTROY);
    close(fd);
}

/* Emit `notches` wheel detents (sign = direction). Sends both classic REL_WHEEL
 * and the hi-res form so old and new (wl_pointer v8+) clients both scroll. */
static void uinput_wheel(int fd, int notches) {
    if (fd < 0 || notches == 0) return;
    struct input_event ev[4] = {0};
    ev[0].type = EV_REL; ev[0].code = REL_WHEEL;        ev[0].value = notches;
    ev[1].type = EV_REL; ev[1].code = REL_WHEEL_HI_RES; ev[1].value = notches * 120;
    ev[2].type = EV_SYN; ev[2].code = SYN_REPORT;       ev[2].value = 0;
    if (write(fd, ev, 3 * sizeof ev[0]) < 0) { /* device may have vanished */ }
}

/* Map the strip cursor to a screen + output-local pixel and inject it. */
static void push_cursor(grab_state *g) {
    if (g->gx < 0) g->gx = 0;
    if (g->gy < 0) g->gy = 0;
    if (g->gx > g->strip_w - 1) g->gx = g->strip_w - 1;
    if (g->gy > g->strip_h - 1) g->gy = g->strip_h - 1;

    /* which grid cell -> which screen. Top canvas band = top visual row. */
    int col  = (int)(g->gx / g->cellw);  if (col  >= g->cols) col  = g->cols - 1;
    int band = (int)(g->gy / g->cellh);  if (band >= g->rows) band = g->rows - 1;
    int lay_row = (g->rows - 1) - band;
    int s = lay_row * g->cols + col;
    if (s >= g->n) return;               /* empty cell in a partial last row */

    g->cur = s;
    if (!g->vp[s]) return;

    uint32_t lx = (uint32_t)(g->gx - g->x0[s]);
    uint32_t ly = (uint32_t)(g->gy - g->y0[s]);
    if (lx > (uint32_t)g->w[s] - 1) lx = g->w[s] - 1;
    if (ly > (uint32_t)g->h[s] - 1) ly = g->h[s] - 1;

    zwlr_virtual_pointer_v1_motion_absolute(g->vp[s], now_ms(), lx, ly,
                                            (uint32_t)g->w[s], (uint32_t)g->h[s]);
    zwlr_virtual_pointer_v1_frame(g->vp[s]);
}

/* Inverse of layout_focus_angles + push_cursor: given the camera look angles
 * render_frame published (m->gaze_yaw/pitch), find the strip position (gx,gy)
 * the eye is pointing at. We pick the screen whose centre is angularly nearest
 * the gaze, then offset within it by the leftover angle: a screen spans
 * screen_arc_deg horizontally and ~the same * aspect vertically, mapped linearly
 * to its pixel box. +yaw is the viewer's left (see layout.c), so we subtract.
 * Out-of-screen gaze (the gaps between panels) clamps to the nearest edge. */
static void gaze_target(grab_state *g, double *out_gx, double *out_gy) {
    struct mirage *m = g->m;
    const mirage_config *c = &m->cfg;
    float gy_a = m->gaze_yaw, gp_a = m->gaze_pitch;

    int best = -1; float bestd = 1e30f, bty = 0, btp = 0;
    for (int i = 0; i < g->n; i++) {
        float ty, tp; layout_focus_angles(m, i, &ty, &tp);
        float dy = gy_a - ty, dp = gp_a - tp;
        float dd = dy*dy + dp*dp;
        if (dd < bestd) { bestd = dd; best = i; bty = ty; btp = tp; }
    }
    if (best < 0) { *out_gx = g->gx; *out_gy = g->gy; return; }

    float ang_w = c->screen_arc_deg * (float)M_PI/180.0f;     /* horizontal span */
    float aspect = (g->w[best] > 0 && g->h[best] > 0)
                   ? (float)g->h[best] / (float)g->w[best] : 9.0f/16.0f;
    float h_m = (c->geometry == GEOM_FLAT)
                ? 2.0f * c->screen_distance_m * tanf(ang_w * 0.5f) * aspect
                : c->screen_distance_m * ang_w * aspect;
    float vspan = 2.0f * atan2f(0.5f * h_m, c->screen_distance_m);  /* vertical span */

    float lx = g->w[best] * 0.5f - ((gy_a - bty) / ang_w)  * g->w[best];
    float ly = g->h[best] * 0.5f - ((gp_a - btp) / vspan)  * g->h[best];
    if (lx < 0) lx = 0;
    if (lx > g->w[best] - 1) lx = g->w[best] - 1;
    if (ly < 0) ly = 0;
    if (ly > g->h[best] - 1) ly = g->h[best] - 1;

    *out_gx = g->x0[best] + lx;
    *out_gy = g->y0[best] + ly;
}

static void do_zoom(grab_state *g, double scroll_v) {
    /* scroll up (negative v) zooms in; multiplicative for even feel */
    float z = g->m->zoom > 0.0f ? g->m->zoom : 1.0f;
    z *= expf((float)(-scroll_v) * 0.0015f);
    if (z < MIRAGE_ZOOM_MIN) z = MIRAGE_ZOOM_MIN;
    if (z > MIRAGE_ZOOM_MAX) z = MIRAGE_ZOOM_MAX;
    g->m->zoom = z;
}

/* Change the workspace on the wall via a Hyprland dispatch. We own the trackpad
 * grab, so Hyprland never sees the 3-finger swipe - we run the switch ourselves.
 * Hyprland 0.55's Lua API: focus the VIRT output the cursor is on, then step the
 * workspace with "m+1"/"m-1" - the MONITOR-relative form, so it only ever cycles
 * the wall's own workspaces and never crosses onto the glasses (which would drop
 * you out to the real desktop). Backgrounded so it never stalls the render loop. */
static void workspace_swipe(grab_state *g, int dir) {
    const char *out = (g->cur >= 0 && g->cur < g->n)
                      ? g->m->screen[g->cur].name : "VIRT1";
    char cmd[256];
    snprintf(cmd, sizeof cmd,
             "hyprctl eval 'hl.dispatch(hl.dsp.focus({ monitor = \"%s\" })); "
             "hl.dispatch(hl.dsp.focus({ workspace = \"m%c1\" }))' >/dev/null 2>&1 &",
             out, dir > 0 ? '+' : '-');
    if (system(cmd) < 0) { /* best-effort; nothing useful to do on failure */ }
}

static void handle_event(grab_state *g, struct libinput_event *ev) {
    enum libinput_event_type t = libinput_event_get_type(ev);
    switch (t) {
    case LIBINPUT_EVENT_KEYBOARD_KEY: {
        struct libinput_event_keyboard *k = libinput_event_get_keyboard_event(ev);
        uint32_t key = libinput_event_keyboard_get_key(k);
        bool down = libinput_event_keyboard_get_key_state(k)
                    == LIBINPUT_KEY_STATE_PRESSED;
        if (key == KEY_LEFTMETA || key == KEY_RIGHTMETA)
            g->super = down;               /* Cmd gates scroll zoom */
        if (key == KEY_LEFTALT || key == KEY_RIGHTALT) {
            /* Double-tap Alt toggles the gaze-follow cursor. Two presses within
             * 350 ms flips it; a single Alt tap (e.g. the Alt+C recenter bind)
             * does nothing here. */
            if (down) {
                uint32_t t = now_ms();
                if (t - g->last_alt_ms < 350u) {
                    g->m->cfg.gaze_cursor = !g->m->cfg.gaze_cursor;
                    g->gaze_prev_valid = false;   /* reseed delta on next frame */
                    g->last_alt_ms = 0;    /* consume, so a third tap re-arms */
                    fprintf(stderr, "grab: gaze cursor %s\n",
                            g->m->cfg.gaze_cursor ? "ON" : "OFF");
                } else {
                    g->last_alt_ms = t;
                }
            }
        }
        break;
    }
    case LIBINPUT_EVENT_POINTER_MOTION: {
        struct libinput_event_pointer *p = libinput_event_get_pointer_event(ev);
        /* RAW (unaccelerated) deltas: libinput's accelerated get_dx() shrinks
         * slow motion toward zero, so a slow drag stalls at a screen edge and
         * can't cross the 1920px cell into the next screen - you had to flick.
         * The unaccelerated delta is linear in finger travel regardless of
         * speed (and regardless of whether the device exposes an accel config),
         * so boundaries cross at any speed. Our g->sens scales it. */
        g->gx += libinput_event_pointer_get_dx_unaccelerated(p) * g->sens;
        g->gy += libinput_event_pointer_get_dy_unaccelerated(p) * g->sens;
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
        if (g->super) {                  /* Cmd+scroll -> telephoto zoom (not forwarded) */
            do_zoom(g, v);
            break;
        }
        /* Inject as real wheel detents via uinput (Hyprland drops virtual-
         * pointer axis for many apps). Accumulate the smooth trackpad delta and
         * emit one notch per NOTCH units; the wheel lands on the window the
         * virtual pointer already moved the cursor over. */
        const double NOTCH = 8.0;        /* trackpad units per wheel detent */
        g->scroll_acc += v;
        while (g->scroll_acc >= NOTCH || g->scroll_acc <= -NOTCH) {
            int step = g->scroll_acc > 0 ? 1 : -1;
            g->scroll_acc -= step * NOTCH;
            uinput_wheel(g->uifd, -step);   /* -step: match natural scroll dir */
        }
        if (v == 0.0) g->scroll_acc = 0.0;   /* reset on finger lift */
        break;
    }
    case LIBINPUT_EVENT_GESTURE_SWIPE_BEGIN: {
        struct libinput_event_gesture *gst = libinput_event_get_gesture_event(ev);
        g->swipe_fingers = libinput_event_gesture_get_finger_count(gst);
        g->swipe_dx = g->swipe_dy = 0.0;
        break;
    }
    case LIBINPUT_EVENT_GESTURE_SWIPE_UPDATE: {
        struct libinput_event_gesture *gst = libinput_event_get_gesture_event(ev);
        g->swipe_dx += libinput_event_gesture_get_dx(gst);
        g->swipe_dy += libinput_event_gesture_get_dy(gst);
        break;
    }
    case LIBINPUT_EVENT_GESTURE_SWIPE_END: {
        struct libinput_event_gesture *gst = libinput_event_get_gesture_event(ev);
        /* 3-finger horizontal swipe -> change workspace on the wall (create a new
         * one past the end), replicating Hyprland's workspace_swipe - which can't
         * see the trackpad while we hold the exclusive grab. One switch per swipe. */
        if (!libinput_event_gesture_get_cancelled(gst) && g->swipe_fingers == 3 &&
            fabs(g->swipe_dx) > 60.0 && fabs(g->swipe_dx) > fabs(g->swipe_dy))
            workspace_swipe(g, g->swipe_dx > 0 ? +1 : -1);
        g->swipe_fingers = 0;
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

    /* Gaze cursor (toggled by double-tap Cmd): move the cursor by HOW MUCH the
     * gaze shifted since last frame, not toward where it points. A still head
     * (the camera read-deadband freezes gaze_yaw/pitch) yields a zero delta, so
     * the cursor stays exactly where your hand left it - no magnet, no jump-back.
     * Move your head and the cursor travels the same amount, keeping the hand's
     * offset. The trackpad deltas above already landed this frame; gaze just adds
     * its own on top. */
    if (m->cfg.gaze_cursor && m->gaze_have) {
        double tx, ty;
        gaze_target(g, &tx, &ty);
        if (!g->gaze_prev_valid) {
            g->gaze_prev_gx = tx; g->gaze_prev_gy = ty;
            g->gaze_prev_valid = true;
        } else {
            double dx = tx - g->gaze_prev_gx, dy = ty - g->gaze_prev_gy;
            g->gaze_prev_gx = tx; g->gaze_prev_gy = ty;
            /* Drop implausible single-frame jumps: those come from gaze_target
             * reclassifying the nearest screen (a position discontinuity), not
             * real head motion. Smooth turns stay well under half a cell/frame. */
            double cap = (g->cellw > g->cellh ? g->cellw : g->cellh) * 0.5;
            if ((dx != 0.0 || dy != 0.0) && fabs(dx) < cap && fabs(dy) < cap) {
                g->gx += dx * g->gaze_gain;
                g->gy += dy * g->gaze_gain;
                push_cursor(g);
            }
        }
    }

    wl_display_flush(m->display);
}

/* True if evdev device `fd` advertises capability `code` of `type` (EV_KEY /
 * EV_ABS). We detect devices by what they CAN emit rather than by name or event
 * number: names vary and event numbers aren't stable across boots/replug. The
 * capability bitmap is readable even when another client (keyd) holds an
 * exclusive grab on the device. */
static bool ev_has(int fd, int type, int code) {
    unsigned long bits[KEY_MAX / (8 * sizeof(long)) + 1];   /* KEY_MAX is the largest map */
    memset(bits, 0, sizeof bits);
    if (ioctl(fd, EVIOCGBIT(type, sizeof bits), bits) < 0) return false;
    return (bits[code / (8 * sizeof(long))] >> (code % (8 * sizeof(long)))) & 1UL;
}

/* True if the device carries the Super (Meta) key - a keyboard that can deliver
 * the Cmd modifier we gate zoom/pan on. */
static bool dev_has_meta(const char *path) {
    int fd = open(path, O_RDONLY);
    if (fd < 0) return false;
    bool ok = ev_has(fd, EV_KEY, KEY_LEFTMETA);
    close(fd);
    return ok;
}

/* True if the device is a multitouch trackpad: BTN_TOOL_FINGER + multitouch X.
 * (A plain mouse / the keyd virtual pointer has REL/ABS but neither of these.) */
static bool dev_is_trackpad(const char *path) {
    int fd = open(path, O_RDONLY);
    if (fd < 0) return false;
    bool ok = ev_has(fd, EV_KEY, BTN_TOOL_FINGER) && ev_has(fd, EV_ABS, ABS_MT_POSITION_X);
    close(fd);
    return ok;
}

/* Collect every keyboard that carries the Super key into g->kbd[]. A keyd setup
 * exposes two (the raw keyboard and a keyd virtual one), and which one actually
 * DELIVERS events flips with keyd's grab state - so we read them ALL, ungrabbed,
 * and take the Super modifier from whichever fires. Skips the trackpad. */
static void detect_keyboards(grab_state *g) {
    g->n_kbd = 0;
    for (int i = 0; i < 32 && g->n_kbd < MAX_KBD; i++) {
        char path[64];
        snprintf(path, sizeof path, "/dev/input/event%d", i);
        if (!strcmp(path, g->dev)) continue;      /* that's the grabbed trackpad */
        if (dev_has_meta(path))
            snprintf(g->kbd[g->n_kbd++], sizeof g->kbd[0], "%s", path);
    }
}

/* Pick the first multitouch trackpad into g->dev. Falls back to event0 (left as
 * set by the caller) if none is found. */
static void detect_trackpad(grab_state *g) {
    for (int i = 0; i < 32; i++) {
        char path[64];
        snprintf(path, sizeof path, "/dev/input/event%d", i);
        if (dev_is_trackpad(path)) { snprintf(g->dev, sizeof g->dev, "%s", path); return; }
    }
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
    /* Trackpad cursor speed. Tuned for the RAW (unaccelerated) deltas we read in
     * handle_event: those run ~8x larger than libinput's accelerated deltas, so
     * ~0.3 reproduces a usable mid-speed feel, linearly. */
    g->sens = 0.3f;
    /* Gaze-follow gain: cursor travel per unit of gaze travel; 1.0 = the cursor
     * moves exactly as far as the point you're looking at. */
    g->gaze_gain = 1.0f;
    g->uifd = -1;               /* opened lazily on capture (fd 0 is valid!) */
    /* Auto-detect input devices by capability, not name or event number (neither
     * is stable across boots/replug): the multitouch trackpad, then every
     * Super-capable keyboard. Trackpad falls back to event0 if none is found. */
    snprintf(g->dev, sizeof g->dev, "%s", "/dev/input/event0");
    detect_trackpad(g);
    detect_keyboards(g);

    g->n = m->n_screen > GRAB_MAX ? GRAB_MAX : m->n_screen;

    /* Lay the cursor canvas out as the SAME grid as the visual wall: screen_cols
     * columns wide, ceil(n/cols) rows tall. So the cursor moves right across the
     * columns and up/down between the rows, instead of a single 1xN row where it
     * could only ever travel right. Uniform cell = the largest screen. */
    g->cols = m->cfg.screen_cols > 0 ? m->cfg.screen_cols : 3;
    g->rows = (g->n + g->cols - 1) / g->cols;
    for (int i = 0; i < g->n; i++) {
        if (m->screen[i].width  > g->cellw) g->cellw = m->screen[i].width;
        if (m->screen[i].height > g->cellh) g->cellh = m->screen[i].height;
    }
    for (int i = 0; i < g->n; i++) {
        screen_t *s = &m->screen[i];
        int col       = i % g->cols;
        int lay_row   = i / g->cols;            /* layout row: 0 = bottom (eye level) */
        int band      = (g->rows - 1) - lay_row; /* canvas band: 0 = top (small y) */
        g->w[i]  = s->width;  g->h[i] = s->height;
        g->x0[i] = col  * g->cellw;
        g->y0[i] = band * g->cellh;
        g->vp[i] = zwlr_virtual_pointer_manager_v1_create_virtual_pointer_with_output(
                       m->vpointer_mgr, m->seat, s->wl);
    }
    g->strip_w = g->cols * g->cellw;
    g->strip_h = g->rows * g->cellh;
    g->gx = g->strip_w / 2.0; g->gy = g->strip_h / 2.0;
    fprintf(stderr, "grab: ready (%d screens, %dx%d grid, strip %dx%d, trackpad %s, %d keyboard(s):",
            g->n, g->cols, g->rows, g->strip_w, g->strip_h, g->dev, g->n_kbd);
    for (int i = 0; i < g->n_kbd; i++) fprintf(stderr, " %s", g->kbd[i]);
    fprintf(stderr, ").\n");
    /* Always-on capture: the trackpad drives the arc cursor and Cmd+scroll zooms
     * from the first frame - no Super+G toggle needed. */
    grab_toggle(m);
    return true;
}

void grab_toggle(struct mirage *m) {
    grab_state *g = GS(m);
    if (!g) return;
    if (!g->active) {
        g->li = libinput_path_create_context(&LI_IFACE, g);
        if (!g->li) { fprintf(stderr, "grab: libinput context failed\n"); return; }
        struct libinput_device *dev = libinput_path_add_device(g->li, g->dev);
        if (!dev) {
            fprintf(stderr, "grab: cannot open trackpad %s (permission? or "
                    "name-match auto-detect picked the wrong device)\n", g->dev);
            libinput_unref(g->li); g->li = NULL; return;
        }
        /* Flat (constant) acceleration: map finger travel linearly to cursor
         * travel. libinput's default adaptive profile DECELERATES slow motion
         * toward zero, so a slow drag parks at a screen edge and can't push the
         * cursor across the 1920px-wide cell boundary into the next screen -
         * only a fast flick accumulates enough delta. Flat removes that "wall",
         * so boundaries cross smoothly at any speed. Our own g->sens scales it. */
        if (libinput_device_config_accel_is_available(dev))
            libinput_device_config_accel_set_profile(
                dev, LIBINPUT_CONFIG_ACCEL_PROFILE_FLAT);
        /* observe (don't grab) every Super-capable keyboard, for the Cmd+scroll
         * zoom / Cmd+H-pan modifier. We add them all so the modifier is seen
         * whether keyd is grabbing the raw keyboard or passing it through. */
        int kbd_ok = 0;
        for (int i = 0; i < g->n_kbd; i++)
            if (libinput_path_add_device(g->li, g->kbd[i])) kbd_ok++;
        if (kbd_ok == 0)
            fprintf(stderr, "grab: note - no keyboard observed; Cmd+scroll zoom disabled\n");
        if (g->uifd < 0) g->uifd = uinput_open();   /* real wheel for scroll */
        g->scroll_acc = 0.0;
        g->super = false;
        g->active = true;
        push_cursor(g);
        fprintf(stderr, "grab: capture ON (trackpad grabbed%s)\n",
                g->uifd >= 0 ? ", scroll via uinput" : ", scroll unavailable");
    } else {
        if (g->li) { libinput_unref(g->li); g->li = NULL; }   /* ungrabs device */
        if (g->uifd >= 0) { uinput_close(g->uifd); g->uifd = -1; }
        g->active = false;
        fprintf(stderr, "grab: capture OFF\n");
    }
    wl_display_flush(m->display);
}

bool grab_active(struct mirage *m) { grab_state *g = GS(m); return g && g->active; }

/* Screen the captured cursor is currently on (== the focused window's screen
 * under focus-follows-mouse), or -1 when not capturing. Capture uses this to
 * grab the active screen at full rate so the focused window never lags. */
int grab_cursor_screen(struct mirage *m) {
    grab_state *g = GS(m);
    return (g && g->active) ? g->cur : -1;
}

void grab_destroy(struct mirage *m) {
    grab_state *g = GS(m);
    if (!g) return;
    if (g->li) { libinput_unref(g->li); g->li = NULL; }
    if (g->uifd >= 0) { uinput_close(g->uifd); g->uifd = -1; }
    for (int i = 0; i < g->n; i++)
        if (g->vp[i]) zwlr_virtual_pointer_v1_destroy(g->vp[i]);
    free(g);
    m->grab = NULL;
}
