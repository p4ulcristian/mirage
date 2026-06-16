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
#include "pose.h"

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

#include <print>

#define GRAB_MAX MIRAGE_MAX_SCREENS
#define MAX_KBD  8           /* how many keyboards we observe for the Super key */
#define MAX_PAD  8           /* how many trackpads we grab at once (built-in + external) */

typedef struct {
    struct mirage *m;           /* back-ref for zoom etc.                 */
    bool active;
    bool super;                 /* Super held? (from the un-grabbed kbd)  */
    bool alt;                   /* Alt held? (gates the Alt+N layout combo) */
    bool world_drag;            /* left-press on empty space: drag spins the world */
    bool sens_drag;             /* left-press on the sensitivity handle: drag sets gain */
    bool sens_click;            /* consumed a left-press on the widget: swallow its release */
    bool bri_drag;              /* left-press on the brightness handle: drag sets env_brightness */
    double shake_pdx, shake_pdy;  /* previous motion vector (shake-to-gaze detector) */
    int      shake_count;         /* recent direction reversals                      */
    uint32_t shake_last_ms;       /* time of the last reversal                       */

    int   n;                    /* screen count                          */

    /* Cursor = a wall-space look direction (rad); layout_pick turns it into a
     * screen + output-local pixel. No flat-strip projection: we point at the same
     * cylinder render draws, so the cursor can't disagree with the picture. */
    double cyaw, cpitch;       /* pointer direction in wall space (rad)  */
    int    cur;                 /* screen the cursor is currently on      */
    float  sens;               /* trackpad delta -> angular motion (rad/unit) */
    uint32_t last_super_ms;    /* last Cmd press (for double-tap recenter)    */
    double scroll_acc;         /* accumulated trackpad delta -> wheel notches */

    struct zwlr_virtual_pointer_v1 *vp[GRAB_MAX];
    struct libinput *li;        /* input, live only while active          */
    char   pad[MAX_PAD][64];    /* trackpad device paths (all grabbed)    */
    int    n_pad;               /* how many entries of pad[] are valid    */
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
    grab_state *g = (grab_state*)u;
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
        std::print(stderr, "grab: scroll disabled - open /dev/uinput: {} "
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
        std::print(stderr, "grab: scroll disabled - uinput setup: {}\n", strerror(errno));
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

/* Pick the screen the cursor direction lands on (layout_pick snaps to the nearest
 * edge in a gap and clamps cyaw/cpitch onto the wall), then inject the cursor at
 * that screen's output-local pixel. One geometry source for input and render. */
static void push_cursor(grab_state *g) {
    struct mirage *m = g->m;
    /* The cursor roams the WHOLE cylinder. Yaw WRAPS a full turn, so you can carry
     * it all the way around behind you; pitch clamps just shy of the poles to keep
     * d*tan(pitch) finite. Everything off the panels is gap, where the 3D arrow
     * shows - so the pointer is free in the entire 3D world around you. */
    const double PITCH_MAX = 1.30;                 /* ~75 deg up/down */
    while (g->cyaw >  M_PI) g->cyaw -= 2.0 * M_PI;
    while (g->cyaw < -M_PI) g->cyaw += 2.0 * M_PI;
    if (g->cpitch >  PITCH_MAX) g->cpitch =  PITCH_MAX;
    if (g->cpitch < -PITCH_MAX) g->cpitch = -PITCH_MAX;

    float u, v; bool inside = false;
    int s = layout_pick(m, (float)g->cyaw, (float)g->cpitch, &u, &v, &inside);
    /* publish the cursor's real direction so render draws the 3D arrow there. Only
     * in the gaps (!inside): over a screen the painted desktop cursor already shows,
     * so the arrow would just duplicate it. */
    m->cursor_yaw = (float)g->cyaw; m->cursor_pitch = (float)g->cpitch;
    m->cursor_have = true; m->cursor_in_gap = !inside;
    if (s < 0) return;
    g->cur = s;
    if (!g->vp[s]) return;

    const screen_t *sc = &m->screen[s];
    int W = sc->width  > 0 ? sc->width  : 1;
    int H = sc->height > 0 ? sc->height : 1;
    int px = (int)(u * W); if (px < 0) px = 0; else if (px > W - 1) px = W - 1;
    int py = (int)(v * H); if (py < 0) py = 0; else if (py > H - 1) py = H - 1;

    zwlr_virtual_pointer_v1_motion_absolute(g->vp[s], now_ms(),
                                            (uint32_t)px, (uint32_t)py,
                                            (uint32_t)W, (uint32_t)H);
    zwlr_virtual_pointer_v1_frame(g->vp[s]);
}

/* Sensitivity slider helpers. The handle and DEFAULT button live in the centre
 * screen's local frame (sens_panel_compute); a cursor (cyaw,cpitch) maps in via
 * x = d*tan(yaw_c - cyaw), y = d*tan(cpitch) - lift_c - the inverse of how render
 * places the panel, so a click lands where the picture shows it. */
static void sens_cursor_local(grab_state *g, const sens_panel *sp, float *lx, float *ly) {
    *lx = sp->d * tanf(sp->yaw_c - (float)g->cyaw);
    *ly = sp->d * tanf((float)g->cpitch) - sp->lift_c;
}

/* Persist the linked yaw/pitch gain into profile.toml, same path calib.cpp uses, so
 * the dialed-in look sensitivity survives a restart (and a layout switch re-stamps
 * it via profile_apply). */
static void sens_persist(struct mirage *m) {
    profile_apply(&m->calib_cfg, &m->cfg);
    std::string p = profile_default_path();
    profile_save(p.c_str(), &m->calib_cfg);
    m->have_profile = true;
}

/* Persist the HUD selections (geometry toggle, brightness, environment, active
 * layout) to ui_state.toml so the wall comes back as the user left it. Called from
 * the widget hit-tests below whenever one of those choices changes at runtime. */
static void ui_persist(struct mirage *m) {
    std::string p = ui_state_default_path();
    ui_state_save(p.c_str(), m);
}

/* Map the handle's x (clamped to the track) to a gain and set BOTH yaw and pitch
 * (one knob: look sensitivity). */
static void sens_set_from_local(struct mirage *m, const sens_panel *sp, float lx) {
    float frac = (lx - sp->track_x0) / (sp->track_x1 - sp->track_x0);
    if (frac < 0.0f) frac = 0.0f;
    if (frac > 1.0f) frac = 1.0f;
    float gain = SENS_GAIN_MIN + frac * (SENS_GAIN_MAX - SENS_GAIN_MIN);
    m->cfg.yaw_gain = gain;
    m->cfg.pitch_gain = gain;
}

/* Map the brightness handle's x (clamped to its track) to env_brightness. */
static void bri_set_from_local(struct mirage *m, const sens_panel *sp, float lx) {
    float frac = (lx - sp->bri_x0) / (sp->bri_x1 - sp->bri_x0);
    if (frac < 0.0f) frac = 0.0f;
    if (frac > 1.0f) frac = 1.0f;
    m->env_brightness = BRI_MIN + frac * (BRI_MAX - BRI_MIN);
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
            g->super = down;               /* Cmd gates scroll zoom */
            /* Double-tap Cmd recenters the head pose (current look = straight
             * ahead). Two presses within 350 ms; a single Cmd press/hold (zoom)
             * is unaffected. */
            if (down) {
                uint32_t t = now_ms();
                if (t - g->last_super_ms < 350u) {
                    pose_recenter();
                    g->last_super_ms = 0;  /* consume, so a third tap re-arms */
                    std::print(stderr, "grab: recentered\n");
                } else {
                    g->last_super_ms = t;
                }
            }
        }
        if (down && key != KEY_LEFTMETA && key != KEY_RIGHTMETA) {
            /* Any other key (e.g. the V in Cmd+V) breaks the Cmd double-tap: the two
             * presses must be consecutive with nothing between them. */
            g->last_super_ms = 0;
        }
        /* Alt+1 / Alt+2 / Alt+3 switch to the Nth named layout (layouts.conf). */
        if (down && g->alt && (key == KEY_1 || key == KEY_2 || key == KEY_3)) {
            int idx = key == KEY_1 ? 0 : key == KEY_2 ? 1 : 2;
            layouts_switch(g->m, idx);
            ui_persist(g->m);            /* remember the active layout across restarts */
        }
        if (key == KEY_LEFTALT || key == KEY_RIGHTALT)
            g->alt = down;               /* track Alt for the Alt+N layout combo */
        break;
    }
    case LIBINPUT_EVENT_POINTER_MOTION: {
        struct libinput_event_pointer *p = libinput_event_get_pointer_event(ev);
        if (calib_active(g->m)) break;   /* freeze the cursor during calibration */
        double rdx = libinput_event_pointer_get_dx_unaccelerated(p);
        double rdy = libinput_event_pointer_get_dy_unaccelerated(p);

        /* Shake-to-gaze ("find my cursor"): shake the cursor back and forth and it
         * warps to where you're LOOKING (head gaze). We count rapid direction
         * REVERSALS - the dot product of consecutive motion vectors going negative -
         * so an ordinary fast move never triggers it; only a deliberate there-and-
         * back-and-there shake does. Needs SHAKE_N reversals, each within GAP ms. */
        const double MIN2 = 6.0*6.0;     /* ignore tiny jitter moves (raw units^2) */
        const uint32_t GAP = 400;        /* max ms between reversals to keep counting */
        const int SHAKE_N = 3;           /* reversals needed to fire */
        if ((rdx*rdx + rdy*rdy) > MIN2) {
            bool had_prev = (g->shake_pdx*g->shake_pdx + g->shake_pdy*g->shake_pdy) > MIN2;
            if (had_prev && (rdx*g->shake_pdx + rdy*g->shake_pdy) < 0.0) {   /* reversed */
                uint32_t t = now_ms();
                g->shake_count = (t - g->shake_last_ms > GAP) ? 1 : g->shake_count + 1;
                g->shake_last_ms = t;
                if (g->shake_count >= SHAKE_N && !g->world_drag && g->m->gaze_have) {
                    g->cyaw = g->m->gaze_yaw; g->cpitch = g->m->gaze_pitch;
                    g->shake_count = 0;
                    g->shake_pdx = rdx; g->shake_pdy = rdy;
                    push_cursor(g);
                    break;
                }
            }
            g->shake_pdx = rdx; g->shake_pdy = rdy;
        }

        /* RAW (unaccelerated) deltas: libinput's accelerated get_dx() shrinks
         * slow motion toward zero, so a slow drag stalls at a screen edge and
         * can't cross the 1920px cell into the next screen - you had to flick.
         * The unaccelerated delta is linear in finger travel regardless of
         * speed (and regardless of whether the device exposes an accel config),
         * so boundaries cross at any speed. Our g->sens scales it. */
        double dx = rdx * g->sens;
        double dy = rdy * g->sens;
        if (g->world_drag) {
            /* Grabbed empty space: horizontal drag spins the whole world about the
             * eye (yaw only - vertical is ignored, so the wall stays level). The
             * cursor itself holds still while the screens sweep past under it. */
            g->m->world_yaw -= (float)dx;
            while (g->m->world_yaw >  (float)M_PI) g->m->world_yaw -= 2.0f*(float)M_PI;
            while (g->m->world_yaw < -(float)M_PI) g->m->world_yaw += 2.0f*(float)M_PI;
        } else {
            /* +yaw = viewer's left (see layout.c), so finger-right (dx>0) lowers yaw;
             * finger-down (dy>0) lowers pitch. Equal finger travel = equal angle
             * anywhere on the wall, so the cursor crosses panels at any speed. */
            g->cyaw   -= dx;
            g->cpitch -= dy;
            if (g->cpitch >  1.4) g->cpitch =  1.4;   /* keep d*tan(pitch) finite */
            if (g->cpitch < -1.4) g->cpitch = -1.4;
        }
        push_cursor(g);
        /* dragging the sensitivity handle: the cursor moved above, so slide the
         * gain to wherever it now sits along the track (handle follows the cursor). */
        if (g->sens_drag || g->bri_drag) {
            sens_panel sp;
            if (sens_panel_compute(g->m, &sp)) {
                float lx, ly; sens_cursor_local(g, &sp, &lx, &ly);
                if (g->sens_drag) sens_set_from_local(g->m, &sp, lx);
                else              bri_set_from_local(g->m, &sp, lx);
            }
        }
        break;
    }
    case LIBINPUT_EVENT_POINTER_BUTTON: {
        struct libinput_event_pointer *p = libinput_event_get_pointer_event(ev);
        uint32_t btn = libinput_event_pointer_get_button(p);
        uint32_t st  = libinput_event_pointer_get_button_state(p)
                       == LIBINPUT_BUTTON_STATE_PRESSED ? 1u : 0u;
        if (calib_active(g->m)) {        /* calibration: a click advances the wizard */
            if (btn == BTN_LEFT && st) calib_click(g->m);
            break;
        }
        /* Sensitivity slider: a left-press on the handle/track grabs it (drag in
         * MOTION sets the gain), a press on DEFAULT resets to 8x. Checked before the
         * world-grab below so a click on the widget never spins the world or reaches
         * a screen. The release that ends a drag is swallowed and saves the profile. */
        if (btn == BTN_LEFT) {
            sens_panel sp;
            if (st && sens_panel_compute(g->m, &sp)) {
                float lx, ly; sens_cursor_local(g, &sp, &lx, &ly);
                if (lx >= sp.def_x0 && lx <= sp.def_x1 && ly >= sp.def_y0 && ly <= sp.def_y1) {
                    g->m->cfg.yaw_gain = g->m->cfg.pitch_gain = SENS_GAIN_DEF;
                    sens_persist(g->m);
                    g->sens_click = true;            /* swallow the matching release */
                    std::print(stderr, "grab: sensitivity reset to {}x\n", SENS_GAIN_DEF);
                    break;
                }
                /* layout switcher: a click on the Nth box applies that named layout. */
                int lhit = -1;
                for (int k = 0; k < sp.n_layout; k++)
                    if (lx >= sp.lay_cx[k] - sp.lay_w*0.5f && lx <= sp.lay_cx[k] + sp.lay_w*0.5f &&
                        ly >= sp.lay_y0 && ly <= sp.lay_y1) { lhit = k; break; }
                if (lhit >= 0) {
                    layouts_switch(g->m, lhit);
                    ui_persist(g->m);                /* remember the active layout    */
                    g->sens_click = true;            /* swallow the matching release */
                    break;
                }
                /* environment switcher: a click on the Nth box applies that backdrop. */
                int ehit = -1;
                for (int k = 0; k < sp.n_env; k++)
                    if (lx >= sp.env_cx[k] - sp.env_w*0.5f && lx <= sp.env_cx[k] + sp.env_w*0.5f &&
                        ly >= sp.env_y0 && ly <= sp.env_y1) { ehit = k; break; }
                if (ehit >= 0) {
                    env_switch(g->m, ehit);
                    ui_persist(g->m);                /* remember the environment      */
                    g->sens_click = true;            /* swallow the matching release */
                    break;
                }
                /* flat/curved toggle: a click flips the geometry and rebuilds meshes. */
                if (lx >= sp.geo_x0 && lx <= sp.geo_x1 &&
                    ly >= sp.geo_y0 && ly <= sp.geo_y1) {
                    g->m->cfg.geometry =
                        g->m->cfg.geometry == GEOM_FLAT ? GEOM_CYLINDER : GEOM_FLAT;
                    g->m->layout_dirty = true;       /* render thread rebuilds meshes  */
                    ui_persist(g->m);                /* remember the toggle            */
                    g->sens_click = true;            /* swallow the matching release  */
                    break;
                }
                /* brightness slider: a press on its track/handle grabs it (MOTION drags). */
                float bri_band = fmaxf(sp.bri_handle_h, sp.bri_track_h) * 0.5f + 0.03f;
                if (lx >= sp.bri_x0 - 0.05f && lx <= sp.bri_x1 + 0.05f &&
                    ly >= sp.bri_row_y - bri_band && ly <= sp.bri_row_y + bri_band) {
                    g->bri_drag = g->sens_click = true;
                    bri_set_from_local(g->m, &sp, lx);   /* jump the handle to the click */
                    break;
                }
                float band = fmaxf(sp.handle_h, sp.track_h) * 0.5f + 0.03f;
                if (lx >= sp.track_x0 - 0.05f && lx <= sp.track_x1 + 0.05f &&
                    ly >= sp.row_y - band && ly <= sp.row_y + band) {
                    g->sens_drag = g->sens_click = true;
                    sens_set_from_local(g->m, &sp, lx);   /* jump the handle to the click */
                    break;
                }
            }
            if (!st && g->sens_click) {
                bool was_sens = g->sens_drag, was_bri = g->bri_drag;
                g->sens_drag = g->sens_click = g->bri_drag = false;
                if (was_sens) {
                    sens_persist(g->m);
                    std::print(stderr, "grab: sensitivity {}x\n", g->m->cfg.yaw_gain);
                } else if (was_bri) {
                    ui_persist(g->m);                /* remember the brightness        */
                    std::print(stderr, "grab: env brightness {:.2f}\n", g->m->env_brightness);
                }
                break;
            }
        }
        /* Left-press on empty space (a gap) grabs the WORLD: the drag rotates it
         * around you (handled in MOTION) and no click reaches a screen. A press on a
         * panel clicks normally; the release that ends a world-grab is swallowed too,
         * so the app under the cursor never sees a stray click. */
        if (btn == BTN_LEFT) {
            if (st && g->m->cursor_in_gap && !g->world_drag) { g->world_drag = true; break; }
            if (!st && g->world_drag) { g->world_drag = false; break; }
        }
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
        if (calib_active(g->m)) {        /* calibration: scroll resizes (FOV step) */
            calib_scroll(g->m, v);
            break;
        }
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
        bool is_pad = false;                      /* skip any grabbed trackpad */
        for (int k = 0; k < g->n_pad; k++)
            if (!strcmp(path, g->pad[k])) { is_pad = true; break; }
        if (is_pad) continue;
        if (dev_has_meta(path))
            snprintf(g->kbd[g->n_kbd++], sizeof g->kbd[0], "%s", path);
    }
}

/* Collect EVERY multitouch trackpad into g->pad[] (built-in + external, e.g. a
 * Magic Trackpad), so they all drive the cursor - no need to choose. */
static void detect_trackpad(grab_state *g) {
    g->n_pad = 0;
    for (int i = 0; i < 32 && g->n_pad < MAX_PAD; i++) {
        char path[64];
        snprintf(path, sizeof path, "/dev/input/event%d", i);
        if (dev_is_trackpad(path))
            snprintf(g->pad[g->n_pad++], sizeof g->pad[0], "%s", path);
    }
}

/* ---- lifecycle ---- */
bool grab_init(struct mirage *m) {
    if (!m->vpointer_mgr || !m->seat) {
        std::print(stderr, "grab: virtual-pointer unavailable; Super+G disabled\n");
        return true;
    }
    grab_state *g = (grab_state*)calloc(1, sizeof *g);
    m->grab = g;
    g->m = m;
    /* Trackpad cursor speed, in radians of wall-angle per RAW (unaccelerated) delta
     * unit. ~1e-4 rad/unit reproduces the old mid-speed feel (the strip ran 0.3
     * px/unit at ~3000 px/rad). Angular, so the feel is the same regardless of
     * screen distance, and motion is linear in finger travel at any speed. */
    g->sens = 1.0e-4f;
    g->uifd = -1;               /* opened lazily on capture (fd 0 is valid!) */
    /* Auto-detect input devices by capability, not name or event number (neither
     * is stable across boots/replug): the multitouch trackpad, then every
     * Super-capable keyboard. Trackpad falls back to event0 if none is found. */
    detect_trackpad(g);
    if (g->n_pad == 0)        /* none found by capability: fall back to event0 */
        snprintf(g->pad[g->n_pad++], sizeof g->pad[0], "%s", "/dev/input/event0");
    detect_keyboards(g);

    g->n = m->n_screen > GRAB_MAX ? GRAB_MAX : m->n_screen;

    /* One virtual pointer per output; the cursor itself is a wall-space look
     * direction that layout_pick maps onto whichever screen it crosses (any
     * layout, free or grid - no projected canvas to collapse). Seed it at the
     * first screen's centre so it starts on a real panel. */
    for (int i = 0; i < g->n; i++)
        g->vp[i] = zwlr_virtual_pointer_manager_v1_create_virtual_pointer_with_output(
                       m->vpointer_mgr, m->seat, m->screen[i].wl);
    /* Seed dead-ahead at eye level (yaw 0, pitch 0); layout_pick snaps it onto the
     * nearest screen, so the cursor starts on a panel in front of you, not parked
     * up on the banner (screen 0). */
    g->cyaw = 0.0; g->cpitch = 0.0;
    std::print(stderr, "grab: ready ({} panels, angular cursor, {} trackpad(s):", g->n, g->n_pad);
    for (int i = 0; i < g->n_pad; i++) std::print(stderr, " {}", g->pad[i]);
    std::print(stderr, ", {} keyboard(s):", g->n_kbd);
    for (int i = 0; i < g->n_kbd; i++) std::print(stderr, " {}", g->kbd[i]);
    std::print(stderr, ").\n");
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
        if (!g->li) { std::print(stderr, "grab: libinput context failed\n"); return; }
        /* Open + grab EVERY detected trackpad, so the built-in pad and an external
         * one (e.g. a Magic Trackpad) both drive the cursor. Flat (constant)
         * acceleration on each: map finger travel linearly to cursor travel.
         * libinput's default adaptive profile DECELERATES slow motion toward zero,
         * so a slow drag parks at a screen edge and can't push the cursor across the
         * 1920px-wide cell boundary into the next screen - only a fast flick
         * accumulates enough delta. Flat removes that "wall", so boundaries cross
         * smoothly at any speed. Our own g->sens scales it. */
        int pad_ok = 0;
        for (int i = 0; i < g->n_pad; i++) {
            struct libinput_device *dev = libinput_path_add_device(g->li, g->pad[i]);
            if (!dev) {
                std::print(stderr, "grab: cannot open trackpad {} (permission?)\n", g->pad[i]);
                continue;
            }
            pad_ok++;
            if (libinput_device_config_accel_is_available(dev))
                libinput_device_config_accel_set_profile(
                    dev, LIBINPUT_CONFIG_ACCEL_PROFILE_FLAT);
        }
        if (pad_ok == 0) {
            std::print(stderr, "grab: no trackpad opened; Super+G disabled\n");
            libinput_unref(g->li); g->li = NULL; return;
        }
        /* observe (don't grab) every Super-capable keyboard, for the Cmd+scroll
         * zoom / Cmd+H-pan modifier. We add them all so the modifier is seen
         * whether keyd is grabbing the raw keyboard or passing it through. */
        int kbd_ok = 0;
        for (int i = 0; i < g->n_kbd; i++)
            if (libinput_path_add_device(g->li, g->kbd[i])) kbd_ok++;
        if (kbd_ok == 0)
            std::print(stderr, "grab: note - no keyboard observed; Cmd+scroll zoom disabled\n");
        if (g->uifd < 0) g->uifd = uinput_open();   /* real wheel for scroll */
        g->scroll_acc = 0.0;
        g->super = false;
        g->alt   = false;
        g->active = true;
        push_cursor(g);
        std::print(stderr, "grab: capture ON (trackpad grabbed{})\n",
                g->uifd >= 0 ? ", scroll via uinput" : ", scroll unavailable");
    } else {
        if (g->li) { libinput_unref(g->li); g->li = NULL; }   /* ungrabs device */
        if (g->uifd >= 0) { uinput_close(g->uifd); g->uifd = -1; }
        g->active = false;
        std::print(stderr, "grab: capture OFF\n");
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
