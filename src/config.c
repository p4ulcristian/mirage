#include "mirage.h"
#include <string.h>

void mirage_config_defaults(mirage_config *c) {
    memset(c, 0, sizeof *c);
    /* 3-column wall (4 virtual displays):
     *   col 0 (left)  : VIRT3, 1080x2160 portrait
     *   col 1 (centre): VIRT1 (top) + VIRT2 (bottom), 1920x1080 stacked
     *   col 2 (right) : VIRT4, 1080x2160 portrait
     * The portraits are 2x1080 tall, so each side panel matches the two stacked
     * 16:9 screens in the centre; all three columns are the same height. */
    c->screen_count     = 4;
    c->screen_distance_m = 2.0f;   /* screen sits 2 m away              */
    c->screen_width_m    = 1.60f;  /* nominal (height follows capture aspect)        */
    c->arc_spacing_deg   = 1.0f;   /* small gap between screens (deg), columns & rows */
    c->explicit_layout   = true;   /* uneven columns: place via screen_col[] below    */
    c->screen_arc_deg    = 40.0f;  /* default arc                                     */
    c->screen_arc[0]     = 40.0f;  /* VIRT1 centre-top    16:9 */
    c->screen_arc[1]     = 40.0f;  /* VIRT2 centre-bottom 16:9 */
    c->screen_arc[2]     = 23.0f;  /* VIRT3 left portrait  (narrow, 2x1080 tall)      */
    c->screen_arc[3]     = 23.0f;  /* VIRT4 right portrait (narrow, 2x1080 tall)      */
    c->screen_col[0]     = 1;      /* VIRT1 -> centre column, top    */
    c->screen_col[1]     = 1;      /* VIRT2 -> centre column, bottom */
    c->screen_col[2]     = 0;      /* VIRT3 -> left column           */
    c->screen_col[3]     = 2;      /* VIRT4 -> right column          */
    c->default_focus     = 1;      /* Cmd+H pan starts on the centre-bottom screen    */
    c->screen_cols       = 3;      /* informational; explicit_layout drives placement */
    c->slab_depth_m      = 0.05f;  /* 5 cm thick: real edges/sides, lit from above */
    c->gaze_cursor       = false;  /* off by default; double-tap Alt toggles */
    c->fov_deg           = 26.0f;  /* glasses vertical FOV (approx)     */
    c->pose_port         = 4242;
    c->pose_smoothing    = 0.08f;  /* legacy fixed nlerp @500Hz (only if --smooth) */
    c->pose_oneeuro      = true;   /* One-Euro adaptive filter is the default      */
    c->pose_mincutoff    = 0.5f;   /* steadiness at rest (Hz); lower = steadier     */
    c->pose_beta         = 1.0f;   /* responsiveness in motion; higher = less lag   */
    c->yaw_gain          = 8.0f;   /* >1 amplifies head yaw: the scene swings faster than the head, so the
                                    * side screens need less neck turn. 1.0 = 1:1 world-fixed (nailed in
                                    * space like a real monitor). (Verified direction empirically via the
                                    * gaze readout: yaw_gain=-1 tracked inverted, so + is correct after the
                                    * IMU axis remap.) */
    c->pitch_gain        = 8.0f;   /* amplify up/down look too, so top/bottom rows need less neck tilt */
    c->roll_damp         = 0.0f;   /* no tilt: fully horizon-locked (head roll ignored) */
    c->read_deadband_deg = 1.1f;   /* freeze <1.1deg tremor so held text stays still (8x gain amplifies jitter hard) */
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
