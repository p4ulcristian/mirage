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
    c->floor_on          = false;  /* flat grass floor (replaced by terrain)       */
    c->floor_height_m    = 1.8f;   /* floor sits 1.8 m below eye (standing height) */
    c->shadows_on        = false;  /* slab drop-shadows (parked for now)           */
    c->sky_on            = false;  /* OFF by default: cloud-dome fbm is per-frame GPU we don't need (--sky to enable) */
    c->terrain_on        = false;  /* OFF by default: 219k-vert noise heightfield was the top frame cost (--terrain) */
    c->gaze_cursor       = true;   /* Cmd-held: cursor follows head gaze (--no-gaze-cursor off) */
    c->fov_deg           = 26.0f;  /* glasses vertical FOV (approx)     */
    c->pose_port         = 4242;
    c->pose_smoothing    = 0.08f;  /* legacy fixed nlerp @500Hz (only if --smooth) */
    c->pose_oneeuro      = true;   /* One-Euro adaptive filter is the default      */
    c->pose_mincutoff    = 0.5f;   /* steadiness at rest (Hz); lower = steadier     */
    c->pose_beta         = 1.0f;   /* responsiveness in motion; higher = less lag   */
    c->yaw_gain          = 2.5f;   /* amplify head yaw: reach side screens w/ ~16deg turn */
    c->pitch_gain        = 3.0f;   /* amplify head pitch: reach the top row w/ ~7deg look-up */
    c->roll_damp         = 0.25f;  /* keep 25% of head roll: mostly horizon-locked  */
    c->read_deadband_deg = 0.5f;   /* freeze <0.5deg tremor so held text stays still */
    c->sharpen           = 0.35f;  /* contrast-adaptive sharpen: recover minified text */
    c->geometry          = GEOM_FLAT;  /* flat panels in the curved arc's layout */
    c->lens_max          = 1.8f;   /* Alt-held loupe peak magnification (1.0..2.5) */
    c->lens_rin          = 0.45f;  /* magnified rect half-height (frac of half-ht) */
    c->lens_rout         = 0.72f;  /* outer edge of the soft border; eased to 1x   */
    strcpy(c->glasses_match, "SmartGlasses");
    c->bg[0] = 0.02f; c->bg[1] = 0.02f; c->bg[2] = 0.035f;
}
