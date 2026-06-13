#include "mirage.h"
#include <string.h>
#include <math.h>

void mirage_config_defaults(mirage_config *c) {
    memset(c, 0, sizeof *c);
    /* free-placement poses default to "auto": derive yaw/lift from the column
     * grid. A named layout (layouts.c) pins a screen by setting a finite yaw. */
    for (int i = 0; i < MIRAGE_MAX_SCREENS; i++) {
        c->screen_yaw_deg[i] = NAN;
        c->screen_lift_m[i]  = NAN;
    }
    /* 5-column wall (4 Full-HD 16:9 + 3 portrait displays):
     *   col 0 : VIRT1 (top) + VIRT2 (bottom), 1920x1080, pair centred
     *   col 1 : VIRT3 (top) + VIRT4 (bottom), 1920x1080, bottom panel at eye level
     *   col 2 : VIRT5                         1080x2160 portrait, single, centred
     *   col 3 : VIRT6                         1080x2160 portrait, single, centred
     *   col 4 : VIRT7                         1080x2160 portrait, single, centred
     * Columns auto-centre on eye level (layout.c); col 1 carries a +lift nudge
     * so its lower panel (VIRT4) sits dead-centre with VIRT3 stacked above it. */
    c->screen_count     = 7;
    c->screen_distance_m = 2.0f;   /* screen sits 2 m away              */
    c->screen_width_m    = 1.60f;  /* nominal (height follows capture aspect)        */
    c->arc_spacing_deg   = 1.0f;   /* small gap between screens (deg), columns & rows */
    c->explicit_layout   = true;   /* uneven columns: place via screen_col[] below    */
    c->screen_arc_deg    = 40.0f;  /* default arc                                     */
    c->screen_arc[0]     = 40.0f;  /* VIRT1 col0 top    16:9 */
    c->screen_arc[1]     = 40.0f;  /* VIRT2 col0 bottom 16:9 */
    c->screen_arc[2]     = 40.0f;  /* VIRT3 col1 top    16:9 */
    c->screen_arc[3]     = 40.0f;  /* VIRT4 col1 bottom 16:9 */
    c->screen_arc[4]     = 23.0f;  /* VIRT5 col2 portrait (narrow, 2x1080 tall) */
    c->screen_arc[5]     = 23.0f;  /* VIRT6 col3 portrait                       */
    c->screen_arc[6]     = 23.0f;  /* VIRT7 col4 portrait                       */
    c->screen_col[0]     = 0;      /* VIRT1 -> col 0, top    */
    c->screen_col[1]     = 0;      /* VIRT2 -> col 0, bottom */
    c->screen_col[2]     = 1;      /* VIRT3 -> col 1, top    */
    c->screen_col[3]     = 1;      /* VIRT4 -> col 1, bottom */
    c->screen_col[4]     = 2;      /* VIRT5 -> col 2, single */
    c->screen_col[5]     = 3;      /* VIRT6 -> col 3, single */
    c->screen_col[6]     = 4;      /* VIRT7 -> col 4, single */
    c->center_col        = 1;      /* col 1 (VIRT3/VIRT4) sits dead ahead at yaw 0 */
    /* lift every column up by half a 40deg panel (+gap) so the landscape stacks'
     * bottom panel lands at eye level and the portraits' bottom edge lines up
     * with them (a centred portrait spans the same height as a 2-landscape
     * stack, so the same nudge aligns all five bottoms). d*arc*aspect/2 + d*spacing/2, d=2. */
    c->screen_lift_m[0]  = 0.41f;  /* VIRT1 col0 top    */
    c->screen_lift_m[1]  = 0.41f;  /* VIRT2 col0 bottom -> eye level */
    c->screen_lift_m[2]  = 0.41f;  /* VIRT3 col1 top    */
    c->screen_lift_m[3]  = 0.41f;  /* VIRT4 col1 bottom -> eye level */
    c->screen_lift_m[4]  = 0.41f;  /* VIRT5 col2 portrait, bottom aligned */
    c->screen_lift_m[5]  = 0.41f;  /* VIRT6 col3 portrait, bottom aligned */
    c->screen_lift_m[6]  = 0.41f;  /* VIRT7 col4 portrait, bottom aligned */
    c->screen_cols       = 5;      /* informational; explicit_layout drives placement */
    c->slab_depth_m      = 0.05f;  /* 5 cm thick: real edges/sides, lit from above */
    c->gaze_cursor       = false;  /* off by default; double-tap Alt toggles */
    c->fov_deg           = 26.0f;  /* glasses vertical FOV (approx)     */
    c->pose_port         = 4242;
    c->pose_smoothing    = 0.08f;  /* legacy fixed nlerp @500Hz (only if --smooth) */
    c->pose_oneeuro      = true;   /* One-Euro adaptive filter is the default      */
    c->pose_mincutoff    = 0.5f;   /* steadiness at rest (Hz); lower = steadier     */
    c->pose_beta         = 1.0f;   /* responsiveness in motion; higher = less lag   */
    c->pose_drift_tau    = 1.0f;   /* cancel 6-axis heading creep when still (s); 0 = off */
    c->pose_predict_ms   = 18.0f;  /* forward-predict head ~18 ms so the wall stays nailed mid-turn (0 = off) */
    c->yaw_gain          = 8.0f;   /* >1 amplifies head yaw: the scene swings faster than the head, so the
                                    * side screens need less neck turn. 1.0 = 1:1 world-fixed (nailed in
                                    * space like a real monitor). (Verified direction empirically via the
                                    * gaze readout: yaw_gain=-1 tracked inverted, so + is correct after the
                                    * IMU axis remap.) */
    c->pitch_gain        = 8.0f;   /* amplify up/down look too, so top/bottom rows need less neck tilt */
    c->roll_damp         = 0.0f;   /* no tilt: fully horizon-locked (head roll ignored) */
    c->read_deadband_deg = 1.1f;   /* freeze <1.1deg tremor so held text stays still (8x gain amplifies jitter hard) */
    c->neck_fwd_m        = 0.10f;  /* eye ~10cm ahead of the neck pivot: head turns translate the eye -> parallax */
    c->neck_up_m         = 0.10f;  /* eye ~10cm above the pivot */
    c->sharpen           = 0.35f;  /* contrast-adaptive sharpen: recover minified text */
    c->geometry          = GEOM_CYLINDER;  /* curved wall: every point equidistant (radius = screen_distance_m) */
    c->hdri_on           = true;   /* starfield backdrop behind the wall */
    strcpy(c->hdri_path, "hdri/starmap_2020_4k.hdr");  /* NASA Deep Star Maps 2020, 4K */
    c->hdri_exposure     = 7.0f;   /* boost the stars hard so they pop after the black point */
    c->hdri_intensity    = 1.0f;   /* full strength: with blacks crushed, only stars add light */
    c->hdri_black        = 0.025f; /* kill the faint haze floor -> true black (transparent on optics) */
    c->hdri_saturation   = 1.7f;   /* punch up star/Milky-Way colour */
    strcpy(c->glasses_match, "SmartGlasses");
    c->bg[0] = 0.0f; c->bg[1] = 0.0f; c->bg[2] = 0.0f;   /* true black = transparent on the additive optics */
}
