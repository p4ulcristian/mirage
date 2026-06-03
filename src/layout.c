#include "mirage.h"

/* Place screen `i` on a horizontal arc around the viewer at the origin.
 *
 * Screens are centred so that with `screen_count` odd, the middle one is dead
 * ahead (-Z). Each screen is a unit quad in its local XY plane (normal +Z);
 * we rotate it about Y by its arc angle and push it out to the arc radius so
 * its face points back at the viewer.
 *
 *   model = T(d*sinθ, 0, -d*cosθ) * Ry(θ) * S(w, h, 1)
 */
/* Each screen is a curved mesh sitting on a sphere of radius d centred on the
 * viewer (see render.c build_curved_mesh), centred on -Z. Placing screen i is
 * just a yaw (column) + pitch (row) rotation about the viewer, so screens meet
 * edge-to-edge horizontally and stack edge-to-edge vertically.
 *
 * Layout is a grid: `screen_cols` per row. Columns wrap around you by yaw so
 * they curve; rows are lifted STRAIGHT UP (no pitch tilt) so both rows share
 * the same vertical cylinder wall — a tall curved monitor split into rows.
 * Row 0 sits at eye level so adding screens above doesn't move the originals. */
mat4 layout_model_matrix(const struct mirage *m, int i) {
    const mirage_config *c = &m->cfg;
    int cols = c->screen_cols > 0 ? c->screen_cols : 3;
    int col = i % cols;
    int row = i / cols;

    float ang_w = c->screen_arc_deg * (float)M_PI/180.0f;       /* width  */
    float gap   = c->arc_spacing_deg * (float)M_PI/180.0f;
    /* +yaw rotates a screen to -X (the viewer's left), so the LOW column index
     * must get the HIGH yaw to land leftmost. Hence ((cols-1)/2 - col): that
     * puts col 0 (VIRT1) on the left, ascending VIRT1,VIRT2,VIRT3 to the right.
     * (Was (col - (cols-1)/2), which reversed it to VIRT3,VIRT2,VIRT1.) */
    float yaw   = ((cols - 1) * 0.5f - (float)col) * (ang_w + gap);

    /* panel height in metres - matches the mesh so rows abut. Flat panels
     * (build_flat_mesh) are 2*d*tan(ang_w/2)*aspect tall; the curved strip is
     * d*ang_w*aspect (arc length * aspect). Same yaw + straight-up lift either
     * way, so the orientation is identical - flat just isn't bent. */
    const screen_t *s = &m->screen[i];
    float aspect = (s->width > 0 && s->height > 0)
                   ? (float)s->height / (float)s->width : 9.0f/16.0f;
    float h = (c->geometry == GEOM_FLAT)
              ? 2.0f * c->screen_distance_m * tanf(ang_w * 0.5f) * aspect
              : c->screen_distance_m * ang_w * aspect;
    /* Row gap to match the column gap: columns are spaced by `gap` radians of
     * yaw, so a metric d*gap between rows subtends the same ~gap angle at the
     * screen distance - equal-looking gaps in both directions. */
    float vgap = c->screen_distance_m * gap;
    float lift = (float)row * (h + vgap);                     /* rows stack up, gapped */

    /* rotate onto the cylinder column, then lift straight up (no tilt) */
    mat4 R = m4_from_quat(q_from_euler_ypr(yaw, 0, 0));
    mat4 T = m4_translate(v3(0.0f, lift, 0.0f));
    return m4_mul(T, R);
}

/* The camera orientation that puts display `i` dead-centre. Yaw is the screen's
 * own column yaw (so q_conj of it un-rotates the screen to -Z); pitch looks up
 * at the row's lifted centre. Geometry must match layout_model_matrix above. */
void layout_focus_angles(const struct mirage *m, int i, float *yaw, float *pitch) {
    const mirage_config *c = &m->cfg;
    int cols = c->screen_cols > 0 ? c->screen_cols : 3;
    int col = i % cols;
    int row = i / cols;

    float ang_w = c->screen_arc_deg * (float)M_PI/180.0f;
    float gap   = c->arc_spacing_deg * (float)M_PI/180.0f;
    *yaw = ((cols - 1) * 0.5f - (float)col) * (ang_w + gap);

    const screen_t *s = &m->screen[i];
    float aspect = (s->width > 0 && s->height > 0)
                   ? (float)s->height / (float)s->width : 9.0f/16.0f;
    float h = (c->geometry == GEOM_FLAT)
              ? 2.0f * c->screen_distance_m * tanf(ang_w * 0.5f) * aspect
              : c->screen_distance_m * ang_w * aspect;
    float vgap = c->screen_distance_m * gap;
    float lift = (float)row * (h + vgap);
    /* screen centre is `lift` above eye at horizontal distance d; look up to it */
    *pitch = atan2f(lift, c->screen_distance_m);
}
