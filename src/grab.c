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
    float  gaze_ease;          /* per-frame lerp of cursor toward gaze (mode on) */
    uint32_t gaze_hold_ms;     /* trackpad use suppresses gaze for this long      */
    uint32_t last_motion_ms;   /* last trackpad motion (gaze yields to the hand)  */
    uint32_t last_cmd_ms;      /* last Cmd press (for double-tap toggle detection) */
    double scroll_acc;         /* accumulated trackpad delta -> wheel notches */
    double hscroll_acc;        /* accumulated H delta -> Cmd-pan display steps */

    struct zwlr_virtual_pointer_v1 *vp[GRAB_MAX];
    struct libinput *li;        /* input, live only while active          */
    char   dev[64];             /* trackpad device path (grabbed)         */
    char   kbd[64];             /* keyboard device path (observed, not grabbed) */
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

    /* Opt-in trace (MIRAGE_GRAB_DEBUG=1, off by default): log every screen change
     * plus a throttled position sample, to diagnose cursor traversal. */
    static int dbg = -1;
    static unsigned tick = 0;
    if (dbg < 0) dbg = getenv("MIRAGE_GRAB_DEBUG") ? 1 : 0;
    if (dbg && s != g->cur)
        fprintf(stderr, "grab: cursor -> screen %d (gx=%.0f gy=%.0f col=%d band=%d "
                "strip=%dx%d cell=%dx%d)\n",
                s, g->gx, g->gy, col, band, g->strip_w, g->strip_h, g->cellw, g->cellh);
    else if (dbg && (++tick % 30u) == 0u)
        fprintf(stderr, "grab: pos gx=%.0f gy=%.0f (screen %d col=%d band=%d)\n",
                g->gx, g->gy, s, col, band);

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

static void handle_event(grab_state *g, struct libinput_event *ev) {
    enum libinput_event_type t = libinput_event_get_type(ev);
    switch (t) {
    case LIBINPUT_EVENT_KEYBOARD_KEY: {
        struct libinput_event_keyboard *k = libinput_event_get_keyboard_event(ev);
        uint32_t key = libinput_event_keyboard_get_key(k);
        bool down = libinput_event_keyboard_get_key_state(k)
                    == LIBINPUT_KEY_STATE_PRESSED;
        if (key == KEY_LEFTMETA || key == KEY_RIGHTMETA) {
            g->super = down;               /* Cmd gates scroll zoom + H-pan */
            /* Double-tap Cmd toggles gaze mode. Two presses within 350 ms and
             * we flip it; a normal Cmd hold (one press) just scrolls/pans. */
            if (down) {
                uint32_t t = now_ms();
                if (t - g->last_cmd_ms < 350u) {
                    g->m->cfg.gaze_cursor = !g->m->cfg.gaze_cursor;
                    g->last_cmd_ms = 0;    /* consume, so a third tap re-arms */
                    fprintf(stderr, "grab: gaze cursor %s\n",
                            g->m->cfg.gaze_cursor ? "ON" : "OFF");
                } else {
                    g->last_cmd_ms = t;
                }
            }
        }
        if (key == KEY_LEFTALT || key == KEY_RIGHTALT) {
            /* Alt held raises the gaze-centre loupe; release springs it back.
             * render_frame eases lens_power toward this each frame. Kept off
             * Cmd so zoom/pan can run without the fisheye warp. */
            g->m->lens_target = down ? g->m->cfg.lens_max : 1.0f;
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
         * so boundaries cross at any speed. Our MIRAGE_SENS scales it. */
        g->gx += libinput_event_pointer_get_dx_unaccelerated(p) * g->sens;
        g->gy += libinput_event_pointer_get_dy_unaccelerated(p) * g->sens;
        g->last_motion_ms = now_ms();    /* hand owns the cursor; gaze backs off */
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

        /* Super+horizontal -> pan the wall across the display ring, snapping one
         * screen per swipe (VIRT1..N as a loop). Independent of the vertical
         * (zoom) axis, so a diagonal swipe can do both. */
        enum libinput_pointer_axis hax = LIBINPUT_POINTER_AXIS_SCROLL_HORIZONTAL;
        if (g->super && libinput_event_pointer_has_axis(p, hax)) {
            double hv = libinput_event_pointer_get_scroll_value(p, hax);
            const double PAN_NOTCH = 30.0;   /* trackpad units per display step */
            int nscr = g->n > 0 ? g->n : 1;
            int cols = g->m->cfg.screen_cols > 0 ? g->m->cfg.screen_cols : 3;
            g->hscroll_acc += hv;
            while (g->hscroll_acc >= PAN_NOTCH || g->hscroll_acc <= -PAN_NOTCH) {
                int dir = g->hscroll_acc > 0 ? 1 : -1;  /* +1 = scroll right -> next screen */
                g->hscroll_acc -= dir * PAN_NOTCH;
                /* Step in snake (boustrophedon) order so every notch lands on a
                 * physical neighbour: bottom row L->R, up a row, top row R->L.
                 * Odd rows reverse, so the raw 2->3 / 5->0 cross-wall leaps go
                 * away. Convert screen idx -> snake pos, step, convert back. */
                int idx = g->m->view_focus;
                int row = idx / cols, col = idx % cols;
                int pos = row * cols + ((row & 1) ? (cols - 1 - col) : col);
                pos = (pos + dir + nscr) % nscr;
                row = pos / cols; col = pos % cols;
                int next = row * cols + ((row & 1) ? (cols - 1 - col) : col);
                g->m->view_focus = (next >= 0 && next < nscr) ? next : pos;
            }
            if (hv == 0.0) g->hscroll_acc = 0.0;   /* reset on finger lift */
        }

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

    /* Gaze cursor (toggled by double-tap Cmd): when the mode is on, ease the
     * cursor toward where the eye points - but only once the hand has been idle
     * for gaze_hold_ms. Any trackpad motion stamps last_motion_ms and suppresses
     * the pull, so fine placement is always the hand's and never fights the eye;
     * let go and after the hold window the cursor re-homes to your gaze. */
    if (m->cfg.gaze_cursor && m->gaze_have &&
        now_ms() - g->last_motion_ms >= g->gaze_hold_ms) {
        double tx, ty;
        gaze_target(g, &tx, &ty);
        g->gx += (tx - g->gx) * g->gaze_ease;
        g->gy += (ty - g->gy) * g->gaze_ease;
        push_cursor(g);
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
    const char *sv = getenv("MIRAGE_SENS");   /* trackpad cursor speed (both axes) */
    /* Tuned for the RAW (unaccelerated) deltas we now read in handle_event:
     * those run ~8x larger than libinput's accelerated deltas, so the old 2.0
     * (set for accelerated motion) made the cursor hypersensitive and overshoot
     * whole screens. ~0.3 reproduces the old usable mid-speed feel, linearly. */
    g->sens = sv ? (float)atof(sv) : 0.3f;
    /* How fast the cursor chases the gaze while Cmd is held (per frame lerp).
     * ~0.25 @120 Hz settles in ~80 ms: snappy but not a hard teleport. */
    const char *ge = getenv("MIRAGE_GAZE_EASE");
    g->gaze_ease = ge ? (float)atof(ge) : 0.25f;
    /* How long a trackpad touch suppresses the gaze pull (ms): the hand owns the
     * cursor while moving and for this long after, so fine placement never fights
     * the eye. After it lapses the cursor re-homes to your gaze. */
    const char *gh = getenv("MIRAGE_GAZE_HOLD");
    g->gaze_hold_ms = gh ? (uint32_t)atoi(gh) : 500u;
    g->uifd = -1;               /* opened lazily on capture (fd 0 is valid!) */
    const char *td = getenv("MIRAGE_TRACKPAD");
    const char *kb = getenv("MIRAGE_KEYBOARD");
    snprintf(g->dev, sizeof g->dev, "%s", td ? td : "/dev/input/event0");
    snprintf(g->kbd, sizeof g->kbd, "%s", kb ? kb : "/dev/input/event1");

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
    fprintf(stderr, "grab: ready (%d screens, %dx%d grid, strip %dx%d, trackpad %s). "
            "Super+G to capture.\n", g->n, g->cols, g->rows, g->strip_w, g->strip_h, g->dev);
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
            fprintf(stderr, "grab: cannot open trackpad %s (permission? wrong "
                    "device? set MIRAGE_TRACKPAD)\n", g->dev);
            libinput_unref(g->li); g->li = NULL; return;
        }
        /* Flat (constant) acceleration: map finger travel linearly to cursor
         * travel. libinput's default adaptive profile DECELERATES slow motion
         * toward zero, so a slow drag parks at a screen edge and can't push the
         * cursor across the 1920px-wide cell boundary into the next screen -
         * only a fast flick accumulates enough delta. Flat removes that "wall",
         * so boundaries cross smoothly at any speed. Our own MIRAGE_SENS scales. */
        if (libinput_device_config_accel_is_available(dev))
            libinput_device_config_accel_set_profile(
                dev, LIBINPUT_CONFIG_ACCEL_PROFILE_FLAT);
        /* observe (don't grab) the keyboard, for the Super+scroll zoom modifier */
        if (!libinput_path_add_device(g->li, g->kbd))
            fprintf(stderr, "grab: note - keyboard %s not observed; Super+scroll "
                    "zoom disabled\n", g->kbd);
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
