#include "mirage.h"

/* On-screen height (metres) of screen j: arc length * source aspect for a curved
 * strip, or 2*d*tan(arc/2)*aspect for a flat panel. Uses the screen's own arc, so
 * a wide wall and a narrow 16:9 each measure correctly. */
static float screen_height(const struct mirage *m, int j) {
    const mirage_config *c = &m->cfg;
    const screen_t *s = &m->screen[j];
    float ang_w  = s->arc_deg * (float)M_PI/180.0f;
    float aspect = (s->width > 0 && s->height > 0)
                   ? (float)s->height / (float)s->width : 9.0f/16.0f;
    return (c->geometry == GEOM_FLAT)
           ? 2.0f * c->screen_distance_m * tanf(ang_w * 0.5f) * aspect
           : c->screen_distance_m * ang_w * aspect;
}

/* Centre height (metres above eye) of screen i's row. Row 0 sits at eye level;
 * each row above abuts the previous one, accounting for the (possibly different)
 * heights of the screens stacked below it plus a uniform gap. */
static float row_lift(const struct mirage *m, int col, int row) {
    const mirage_config *c = &m->cfg;
    int cols = c->screen_cols > 0 ? c->screen_cols : 3;
    float vgap = c->screen_distance_m * c->arc_spacing_deg * (float)M_PI/180.0f;
    float lift = 0.0f;
    float h_prev = screen_height(m, col);                 /* row 0 in this column */
    for (int r = 1; r <= row; r++) {
        int j = r * cols + col;
        if (j >= m->n_screen) break;
        float h_r = screen_height(m, j);
        lift += h_prev * 0.5f + vgap + h_r * 0.5f;
        h_prev = h_r;
    }
    return lift;
}

/* Place screen `i` on the arc: a yaw (its column) + a straight-up lift (its row),
 * both about the viewer at the origin. Columns wrap around you by yaw so the wall
 * curves; rows stack vertically with no tilt, so a screen above shares the same
 * cylinder as the one below it. Each screen keeps its own arc width. */
mat4 layout_model_matrix(const struct mirage *m, int i) {
    const mirage_config *c = &m->cfg;
    int cols = c->screen_cols > 0 ? c->screen_cols : 3;
    int col = i % cols;
    int row = i / cols;

    float ang_w = m->screen[i].arc_deg * (float)M_PI/180.0f;
    float gap   = c->arc_spacing_deg * (float)M_PI/180.0f;
    /* +yaw rotates a screen to -X (the viewer's left); low column index gets high
     * yaw so col 0 lands leftmost. With one column this is 0 (dead-ahead). */
    float yaw  = ((cols - 1) * 0.5f - (float)col) * (ang_w + gap);
    float lift = row_lift(m, col, row);

    mat4 R = m4_from_quat(q_from_euler_ypr(yaw, 0, 0));
    mat4 T = m4_translate(v3(0.0f, lift, 0.0f));
    return m4_mul(T, R);
}

/* The camera orientation that puts display `i` dead-centre: its column yaw, and a
 * pitch up to its row's lifted centre. Must match layout_model_matrix. */
void layout_focus_angles(const struct mirage *m, int i, float *yaw, float *pitch) {
    const mirage_config *c = &m->cfg;
    int cols = c->screen_cols > 0 ? c->screen_cols : 3;
    int col = i % cols;
    int row = i / cols;

    float ang_w = m->screen[i].arc_deg * (float)M_PI/180.0f;
    float gap   = c->arc_spacing_deg * (float)M_PI/180.0f;
    *yaw  = ((cols - 1) * 0.5f - (float)col) * (ang_w + gap);
    float lift = row_lift(m, col, row);
    *pitch = atan2f(lift, c->screen_distance_m);
}
