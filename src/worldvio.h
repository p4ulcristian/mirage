/* worldvio.h - lightweight visual motion from the world-facing camera (6DoF-lite).
 *
 * The IMU (VQF) gives rotation but physically cannot sense translation, so a 3DoF+
 * neck model only *guesses* the eye's arc from rotation - it's blind to you actually
 * leaning or swaying. This module watches the world camera and estimates the part of
 * the image motion that ISN'T explained by head rotation (which the IMU tells us), i.e.
 * the parallax from real head translation -> lateral/vertical "lean/sway" position.
 *
 * Deliberately dependency-free (no OpenCV): it correlates row/column intensity
 * profiles (integral projection) of consecutive downsampled frames to get the global
 * image shift, subtracts the IMU-predicted rotational shift, and integrates the
 * residual (with a leak, so monocular drift bleeds off) into an eye offset.
 *
 * Call worldvio_feed() from the render thread each frame when TRACK_CAMERA is active;
 * read worldvio_eye_offset() for the parallax translation to add to the view.
 */
#ifndef MIRAGE_WORLDVIO_H
#define MIRAGE_WORLDVIO_H

#include "math3d.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Tunables (sane defaults baked in; expose later if needed). */
typedef struct {
    float trans_gain;   /* residual flow (downsampled px) -> metres of eye translation */
    float leak_tau_s;   /* position leak time constant (s); smaller = bleeds back faster,
                         * kills monocular drift but a sustained lean decays. */
    float flow_clamp;   /* max believable per-frame shift (downsampled px); rejects blur/cuts */
    bool  invert_x, invert_y;  /* axis sign flips (tune on hardware) */
} worldvio_cfg;

void worldvio_start(const worldvio_cfg *cfg);   /* cfg NULL = defaults */
void worldvio_stop(void);

/* Feed one world-cam frame (RGB888, w*h, tightly packed) with the head orientation at
 * capture and a monotonic timestamp (s). hfov_deg = camera horizontal field of view. */
void worldvio_feed(const uint8_t *rgb, int w, int h, quat head, double t_sec, float hfov_deg);

/* Latest camera-estimated eye translation (mirage world axes, metres): +x right, +y up,
 * -z toward the scene. Transient lean/sway parallax (leaked, so it decays to rest). */
vec3 worldvio_eye_offset(void);

/* True once it has processed enough frames to give a usable estimate. */
bool worldvio_active(void);

/* 0..1 confidence in the current camera estimate (from the RANSAC inlier count). The
 * eye offset above is already scaled by this, so it auto-fades to the neck model when
 * the camera can't see well; exposed for an optional HUD indicator. */
float worldvio_confidence(void);

/* Drop history (call on recenter / mode switch / camera (re)start). */
void worldvio_reset(void);

#ifdef __cplusplus
}
#endif
#endif
