#include "mirage.h"
#include <string.h>

void mirage_config_defaults(mirage_config *c) {
    memset(c, 0, sizeof *c);
    c->screen_count     = 1;       /* screenshare: ONE 32:9 ultrawide wall (VIRT1) */
    c->screen_distance_m = 2.0f;   /* screen sits 2 m away              */
    c->screen_width_m    = 2.80f;  /* wide 32:9 wall (height follows capture aspect) */
    c->arc_spacing_deg   = 1.0f;   /* small gap between screens (deg), columns & rows */
    c->screen_arc_deg    = 70.0f;  /* the single wall spans 70 deg of arc (glance to edges, no neck-pan) */
    c->screen_cols       = 1;      /* one column: the wall is centred dead-ahead */
    c->slab_depth_m      = 0.05f;  /* 5 cm thick: real edges/sides, lit from above */
    c->gaze_cursor       = true;   /* Cmd-held: cursor follows head gaze */
    c->fov_deg           = 26.0f;  /* glasses vertical FOV (approx)     */
    c->pose_port         = 4242;
    c->pose_smoothing    = 0.08f;  /* legacy fixed nlerp @500Hz (only if --smooth) */
    c->pose_oneeuro      = true;   /* One-Euro adaptive filter is the default      */
    c->pose_mincutoff    = 0.5f;   /* steadiness at rest (Hz); lower = steadier     */
    c->pose_beta         = 1.0f;   /* responsiveness in motion; higher = less lag   */
    c->yaw_gain          = 1.0f;   /* 1:1 world-fixed: the wall is nailed in space like a real monitor.
                                    * (Verified direction empirically via the gaze readout: yaw_gain=-1
                                    * tracked inverted, so +1 is correct after the IMU axis remap.) */
    c->pitch_gain        = 1.0f;   /* 1:1 world-fixed vertically too */
    c->roll_damp         = 0.0f;   /* no tilt: fully horizon-locked (head roll ignored) */
    c->read_deadband_deg = 0.5f;   /* freeze <0.5deg tremor so held text stays still */
    c->sharpen           = 0.35f;  /* contrast-adaptive sharpen: recover minified text */
    c->geometry          = GEOM_CYLINDER;  /* curved wall: every point equidistant (radius = screen_distance_m) */
    strcpy(c->glasses_match, "SmartGlasses");
    c->bg[0] = 0.02f; c->bg[1] = 0.02f; c->bg[2] = 0.035f;
}
