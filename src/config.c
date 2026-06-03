#include "mirage.h"
#include <string.h>

void mirage_config_defaults(mirage_config *c) {
    memset(c, 0, sizeof *c);
    c->screen_count     = 6;       /* 2 rows x 3 cols (VIRT1-6); auto-bound by discovery */
    c->screen_distance_m = 2.0f;   /* screens sit 2 m away              */
    c->screen_width_m    = 1.30f;  /* ~1.3 m wide each (a big monitor)  */
    c->arc_spacing_deg   = 2.0f;   /* small gap between screens (deg); 0 = touching */
    c->screen_arc_deg    = 40.0f;  /* each curved screen spans 40 deg of arc */
    c->screen_cols       = 3;      /* 3 per row; extra screens stack above   */
    c->fov_deg           = 26.0f;  /* glasses vertical FOV (approx)     */
    c->pose_port         = 4242;
    c->pose_smoothing    = 0.08f;  /* legacy fixed nlerp @500Hz (only if --smooth) */
    c->pose_oneeuro      = true;   /* One-Euro adaptive filter is the default      */
    c->pose_mincutoff    = 0.5f;   /* steadiness at rest (Hz); lower = steadier     */
    c->pose_beta         = 1.0f;   /* responsiveness in motion; higher = less lag   */
    c->yaw_gain          = 2.5f;   /* amplify head yaw: reach side screens w/ ~16deg turn */
    c->pitch_gain        = 3.0f;   /* amplify head pitch: reach the top row w/ ~7deg look-up */
    c->roll_damp         = 0.25f;  /* keep 25% of head roll: mostly horizon-locked  */
    c->read_deadband_deg = 0.22f;  /* freeze <0.22deg tremor so held text stays still */
    c->sharpen           = 0.35f;  /* contrast-adaptive sharpen: recover minified text */
    strcpy(c->glasses_match, "SmartGlasses");
    c->bg[0] = 0.02f; c->bg[1] = 0.02f; c->bg[2] = 0.035f;
}
