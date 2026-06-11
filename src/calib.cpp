/* calib.cpp - in-glasses guided calibration overlay.
 *
 * One little state machine, two cadences:
 *   - first run  -> the FULL wizard: centre, then a tracking check, then FOV, then
 *                   save the calibration to profile.toml.
 *   - every run  -> the QUICK centre: just "look forward, hold still" -> recenter,
 *                   because "forward" depends on how the glasses sit this session
 *                   and so can't be saved (everything else is, and stays put).
 *
 * The logic lives here; render.cpp draws the head-locked panel from calib_text()
 * and calib_progress(), and grab.cpp routes a trackpad click / scroll in here.
 */
#include "mirage.h"
#include "pose.h"

#include <cmath>
#include <ctime>
#include <string>

static const float CENTER_S  = 1.2f;   /* fixed centring countdown (s)            */
static const float DONE_S    = 0.9f;   /* how long the "Ready" flash lingers      */
static const float NOSIGNAL_S = 3.0f;  /* no IMU signal this long -> skip calib    */

static double mono_s(void) {
    struct timespec ts; clock_gettime(CLOCK_MONOTONIC, &ts);
    return ts.tv_sec + ts.tv_nsec / 1e9;
}

void calib_init(struct mirage *m, bool had_profile) {
    m->calib.step      = CALIB_CENTER;     /* every launch starts by centring */
    m->calib.wizard    = !had_profile;     /* no profile yet -> full first-run wizard */
    m->calib.still_t   = 0.0f;
    m->calib.done_t    = 0.0f;
    m->calib.wait_t    = 0.0f;
    m->calib.have_prev = false;
}

void calib_start_wizard(struct mirage *m) {
    m->calib.step      = CALIB_CENTER;
    m->calib.wizard    = true;
    m->calib.still_t   = 0.0f;
    m->calib.have_prev = false;
}

bool calib_active(const struct mirage *m) { return m->calib.step != CALIB_OFF; }

/* CENTER locked in: recenter, then branch by cadence. */
static void after_center(struct mirage *m) {
    pose_recenter();
    if (m->calib.wizard) { m->calib.step = CALIB_TRACK; }
    else                 { m->calib.step = CALIB_DONE; m->calib.done_t = DONE_S; }
}

void calib_update(struct mirage *m, quat head) {
    (void)head;   /* centring is a timed countdown now, not orientation-gated */
    calib_state *c = &m->calib;
    if (c->step == CALIB_OFF) return;

    static double prev_t = 0.0;
    double t = mono_s();
    float dt = prev_t > 0.0 ? (float)(t - prev_t) : 0.0f;
    prev_t = t;
    if (dt > 0.25f) dt = 0.25f;            /* a stall shouldn't bank stillness */

    switch (c->step) {
    case CALIB_CENTER: {
        /* need a live head signal; until then, sit and wait (no false centre) - but
         * not forever. Running on the laptop without glasses there is no IMU, so the
         * signal never comes; after a short grace period give up on tracking
         * calibration entirely (you can't centre a head sensor that isn't there) and
         * drop the overlay, so the desktop look-around (3/4-finger swipes) is usable. */
        if (!pose_has_signal()) {
            c->still_t = 0.0f;
            c->wait_t += dt;
            if (c->wait_t >= NOSIGNAL_S) { c->wait_t = 0.0f; c->step = CALIB_OFF; }
            return;
        }
        c->wait_t = 0.0f;
        /* Fixed countdown, NOT a stillness gate: a 6-axis IMU's heading drifts, so
         * the orientation never reads "still" even with your head dead-still - a
         * stillness gate could never finish. Instead we pin the view to centre
         * every frame while counting down (so the pre-recenter drift never shows),
         * then lock it. You just look forward for ~1 s. */
        pose_recenter();
        c->still_t += dt;
        if (c->still_t >= CENTER_S) { c->still_t = 0.0f; after_center(m); }
        break;
    }
    case CALIB_DONE:
        c->done_t -= dt;
        if (c->done_t <= 0.0f) c->step = CALIB_OFF;
        break;
    case CALIB_TRACK:
    case CALIB_FOV:
        break;                              /* wait for the user's click */
    default: break;
    }
}

/* Trackpad click: advance the wizard. (Ignored outside the click-driven steps.) */
void calib_click(struct mirage *m) {
    calib_state *c = &m->calib;
    if (c->step == CALIB_TRACK) {
        c->step = CALIB_FOV;
    } else if (c->step == CALIB_FOV) {
        /* persist the tuned calibration: fold cfg's calib fields into the stash so
         * a later layout switch keeps them, then write profile.toml. */
        profile_apply(&m->calib_cfg, &m->cfg);
        std::string p = profile_default_path();
        profile_save(p.c_str(), &m->calib_cfg);
        m->have_profile = true;
        c->step = CALIB_DONE; c->done_t = DONE_S;
    } else if (c->step == CALIB_CENTER) {
        c->still_t = 0.0f; c->have_prev = false;   /* click = "redo, I'll hold still" */
    }
}

/* Scroll in the FOV step resizes the screens (FOV is what render projects through;
 * no mesh rebuild needed). Up = bigger. */
void calib_scroll(struct mirage *m, double v) {
    if (m->calib.step != CALIB_FOV) return;
    float f = m->cfg.fov_deg - (float)v * 0.5f;
    if (f < 30.0f) f = 30.0f;
    if (f > 110.0f) f = 110.0f;
    m->cfg.fov_deg = f;
}

const char *calib_text(const struct mirage *m) {
    switch (m->calib.step) {
    case CALIB_CENTER: return "Look at the center screen\nand hold still";
    case CALIB_TRACK:  return "Turn your head — the screens\nshould stay put.  Click to continue";
    case CALIB_FOV:    return "Scroll to resize the screens\nClick when it looks right";
    case CALIB_DONE:   return "Ready";
    default:           return "";
    }
}

float calib_progress(const struct mirage *m) {
    if (m->calib.step == CALIB_CENTER) {
        float p = m->calib.still_t / CENTER_S;
        return p < 0.0f ? 0.0f : (p > 1.0f ? 1.0f : p);
    }
    return -1.0f;
}
