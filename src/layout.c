#include "mirage.h"

/* Arc width (deg) of screen j: the per-screen override once render has resolved it
 * onto the mesh (screen.arc_deg), else the config value, else the global default.
 * Robust to being called before render builds the meshes (cursor setup, etc.). */
static float scr_arc(const struct mirage *m, int j) {
    float a = m->screen[j].arc_deg;
    if (a <= 0.0f)
        a = m->cfg.screen_arc[j] > 0.0f ? m->cfg.screen_arc[j] : m->cfg.screen_arc_deg;
    return a;
}

/* On-screen height (metres) of screen j for a given arc: arc length * source
 * aspect for a curved strip, or 2*d*tan(arc/2)*aspect for a flat panel. */
static float scr_height(const struct mirage *m, int j, float arc_deg) {
    const mirage_config *c = &m->cfg;
    const screen_t *s = &m->screen[j];
    float ang_w  = arc_deg * (float)M_PI/180.0f;
    float aspect = (s->width > 0 && s->height > 0)
                   ? (float)s->height / (float)s->width : 9.0f/16.0f;
    return (c->geometry == GEOM_FLAT)
           ? 2.0f * c->screen_distance_m * tanf(ang_w * 0.5f) * aspect
           : c->screen_distance_m * ang_w * aspect;
}

/* On-screen height (metres) of screen j. Uses the screen's own arc, so a wide
 * wall and a narrow 16:9 each measure correctly. */
static float screen_height(const struct mirage *m, int j) {
    return scr_height(m, j, scr_arc(m, j));
}

/* ---- column membership (legacy uniform grid OR explicit per-screen columns) ---- */

int layout_screen_col(const struct mirage *m, int i) {
    const mirage_config *c = &m->cfg;
    if (c->explicit_layout) { int v = c->screen_col[i]; return v < 0 ? 0 : v; }
    int cols = c->screen_cols > 0 ? c->screen_cols : 3;
    return i % cols;
}

int layout_num_cols(const struct mirage *m) {
    const mirage_config *c = &m->cfg;
    int n = m->n_screen > 0 ? m->n_screen : c->screen_count;
    if (!c->explicit_layout) return c->screen_cols > 0 ? c->screen_cols : 3;
    int mx = 0;
    for (int i = 0; i < n; i++) { int cc = layout_screen_col(m, i); if (cc > mx) mx = cc; }
    return mx + 1;
}

/* ---- legacy uniform-grid vertical stacking (row 0 = eye level, rows go up) ---- */
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

/* Place screen `i`: the yaw of its column centre (rad), the vertical lift of its
 * centre (m), and its arc width (deg). Two modes:
 *
 *   legacy grid   - uniform `screen_cols` columns; row 0 at eye level, rows up.
 *   explicit_layout - columns named per-screen by screen_col[]; each column laid
 *                     out left-to-right by its widest member, and its members
 *                     stacked vertically (array order = top->bottom) and centred
 *                     at eye level, so columns of different heights line up.
 *
 * +yaw rotates a screen to the viewer's LEFT, so the lowest column index lands
 * leftmost (matches layout_model_matrix / the cursor canvas). */
void layout_place(const struct mirage *m, int i, float *yaw_out, float *lift_out,
                  float *arc_out) {
    const mirage_config *c = &m->cfg;
    float gap = c->arc_spacing_deg * (float)M_PI/180.0f;
    float arc_i = scr_arc(m, i);
    *arc_out = arc_i;

    if (!c->explicit_layout) {
        int cols = c->screen_cols > 0 ? c->screen_cols : 3;
        int col = i % cols, row = i / cols;
        float ang_w = arc_i * (float)M_PI/180.0f;
        *yaw_out  = ((cols - 1) * 0.5f - (float)col) * (ang_w + gap);
        *lift_out = row_lift(m, col, row);
        return;
    }

    int n     = m->n_screen > 0 ? m->n_screen : c->screen_count;
    int ncols = layout_num_cols(m);
    int myc   = layout_screen_col(m, i);

    /* per-column angular width = widest member's arc (rad) */
    float colw[MIRAGE_MAX_SCREENS];
    for (int k = 0; k < ncols; k++) colw[k] = 0.0f;
    for (int j = 0; j < n; j++) {
        int cc = layout_screen_col(m, j);
        float a = scr_arc(m, j) * (float)M_PI/180.0f;
        if (a > colw[cc]) colw[cc] = a;
    }

    /* total span incl gaps; walk left->right (col 0 leftmost = highest +yaw) */
    float span = gap * (float)(ncols - 1);
    for (int k = 0; k < ncols; k++) span += colw[k];
    float edge = span * 0.5f;          /* left edge of the whole wall */
    float yaw_c = 0.0f;
    for (int k = 0; k < ncols; k++) {
        float center = edge - colw[k] * 0.5f;
        if (k == myc) yaw_c = center;
        edge -= (colw[k] + gap);
    }
    *yaw_out = yaw_c;

    /* vertical: stack my column's members (array order = top first), centre the
     * whole stack on eye level so columns of differing heights line up. */
    float vgap = c->screen_distance_m * gap;   /* metres */
    float Hc = 0.0f; int cnt = 0;
    for (int j = 0; j < n; j++)
        if (layout_screen_col(m, j) == myc) { Hc += scr_height(m, j, scr_arc(m, j)); cnt++; }
    if (cnt > 1) Hc += vgap * (float)(cnt - 1);
    float top = Hc * 0.5f, acc = 0.0f, mylift = 0.0f;
    for (int j = 0; j < n; j++)
        if (layout_screen_col(m, j) == myc) {
            float h = scr_height(m, j, scr_arc(m, j));
            float center = top - acc - h * 0.5f;
            if (j == i) mylift = center;
            acc += h + vgap;
        }
    *lift_out = mylift;
}

/* Place screen `i` on the arc: a yaw (its column) + a straight-up lift (its row),
 * both about the viewer at the origin. */
mat4 layout_model_matrix(const struct mirage *m, int i) {
    float yaw, lift, arc;
    layout_place(m, i, &yaw, &lift, &arc);
    mat4 R = m4_from_quat(q_from_euler_ypr(yaw, 0, 0));
    mat4 T = m4_translate(v3(0.0f, lift, 0.0f));
    return m4_mul(T, R);
}

/* The camera orientation that puts display `i` dead-centre: its column yaw, and a
 * pitch up to its row's lifted centre. Must match layout_model_matrix. */
void layout_focus_angles(const struct mirage *m, int i, float *yaw, float *pitch) {
    float y, lift, arc;
    layout_place(m, i, &y, &lift, &arc);
    *yaw   = y;
    *pitch = atan2f(lift, m->cfg.screen_distance_m);
}
