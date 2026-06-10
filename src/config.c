#include "mirage.h"
#include <string.h>

void mirage_config_defaults(mirage_config *c) {
    memset(c, 0, sizeof *c);
    c->screen_count     = 2;       /* VIRT1 = 32:9 DFHD wall, VIRT2 = 16:9 monitor above it */
    c->screen_distance_m = 2.0f;   /* screen sits 2 m away              */
    c->screen_width_m    = 2.80f;  /* wide 32:9 wall (height follows capture aspect) */
    c->arc_spacing_deg   = 1.0f;   /* small gap between screens (deg), columns & rows */
    c->screen_arc_deg    = 70.0f;  /* default arc; the wide wall spans this */
    c->screen_arc[0]     = 70.0f;  /* VIRT1: the 32:9 wall, glance edge-to-edge */
    c->screen_arc[1]     = 38.0f;  /* VIRT2: narrower so the 16:9 reads like a monitor above */
    c->screen_cols       = 1;      /* one column: both centred dead-ahead, stacked vertically */
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
    c->hdri_on           = true;   /* starfield backdrop behind the wall */
    strcpy(c->hdri_path, "hdri/starmap_2020_4k.hdr");  /* NASA Deep Star Maps 2020, 4K */
    c->hdri_exposure     = 7.0f;   /* boost the stars hard so they pop after the black point */
    c->hdri_intensity    = 1.0f;   /* full strength: with blacks crushed, only stars add light */
    c->hdri_black        = 0.025f; /* kill the faint haze floor -> true black (transparent on optics) */
    c->hdri_saturation   = 1.7f;   /* punch up star/Milky-Way colour */
    strcpy(c->glasses_match, "SmartGlasses");
    c->bg[0] = 0.0f; c->bg[1] = 0.0f; c->bg[2] = 0.0f;   /* true black = transparent on the additive optics */
}
