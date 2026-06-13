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

static const float STILL_DEG = 1.5f;   /* head slower than this (deg/s) = "still" */
static const float HOLD_S    = 1.0f;   /* hold still this long to lock the centre */
static const float DONE_S    = 1.3f;   /* how long the "Ready" flash lingers      */

static double mono_s(void) {
    struct timespec ts; clock_gettime(CLOCK_MONOTONIC, &ts);
    return ts.tv_sec + ts.tv_nsec / 1e9;
}

/* angular speed (deg/s) between two head samples dt seconds apart */
static float head_speed_deg(quat a, quat b, float dt) {
    float d = a.w*b.w + a.x*b.x + a.y*b.y + a.z*b.z;
    if (d < 0.0f) d = -d;
    if (d > 1.0f) d = 1.0f;
    float ang = 2.0f * acosf(d) * 180.0f/(float)M_PI;   /* degrees moved */
    return ang / (dt > 1e-4f ? dt : 1e-4f);
}

void calib_init(struct mirage *m, bool had_profile) {
    m->calib.step      = CALIB_CENTER;     /* every launch starts by centring */
    m->calib.wizard    = !had_profile;     /* no profile yet -> full first-run wizard */
    m->calib.still_t   = 0.0f;
    m->calib.done_t    = 0.0f;
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
    calib_state *c = &m->calib;
    if (c->step == CALIB_OFF) return;

    static double prev_t = 0.0;
    double t = mono_s();
    float dt = prev_t > 0.0 ? (float)(t - prev_t) : 0.0f;
    prev_t = t;
    if (dt > 0.25f) dt = 0.25f;            /* a stall shouldn't bank stillness */

    switch (c->step) {
    case CALIB_CENTER: {
        /* need a live head signal; until then, sit and wait (no false centre). */
        if (!pose_has_signal()) { c->still_t = 0.0f; c->have_prev = false; return; }
        if (!c->have_prev) { c->prev = head; c->have_prev = true; return; }
        float sp = head_speed_deg(head, c->prev, dt > 0 ? dt : 1e-3f);
        c->prev = head;
        if (sp < STILL_DEG) c->still_t += dt; else c->still_t = 0.0f;
        if (c->still_t >= HOLD_S) { c->still_t = 0.0f; after_center(m); }
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
        float p = m->calib.still_t / HOLD_S;
        return p < 0.0f ? 0.0f : (p > 1.0f ? 1.0f : p);
    }
    return -1.0f;
}
