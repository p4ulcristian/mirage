#include "mirage.h"
#include <string.h>

void mirage_config_defaults(mirage_config *c) {
    memset(c, 0, sizeof *c);
    c->screen_count     = 3;
    c->screen_distance_m = 2.0f;   /* screens sit 2 m away              */
    c->screen_width_m    = 1.30f;  /* ~1.3 m wide each (a big monitor)  */
    c->arc_spacing_deg   = 0.0f;   /* extra gap between screens; 0 = touching */
    c->screen_arc_deg    = 40.0f;  /* each curved screen spans 40 deg of arc */
    c->screen_cols       = 3;      /* 3 per row; extra screens stack above   */
    c->fov_deg           = 26.0f;  /* glasses vertical FOV (approx)     */
    c->pose_port         = 4242;
    c->pose_smoothing    = 0.08f;  /* nlerp @500Hz; lower = smoother/laggier */
    strcpy(c->glasses_match, "SmartGlasses");
    c->bg[0] = 0.02f; c->bg[1] = 0.02f; c->bg[2] = 0.035f;
}
