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

/* A screen is "free" when a named layout pinned it to an explicit yaw: it ignores
 * the column grid and sits exactly where placed (layouts.c / cfg.screen_yaw_deg). */
static bool screen_is_free(const struct mirage *m, int i) {
    return isfinite(m->cfg.screen_yaw_deg[i]);
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
    for (int i = 0; i < n; i++) {
        if (screen_is_free(m, i)) continue;
        int cc = layout_screen_col(m, i); if (cc > mx) mx = cc;
    }
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
static void layout_place_base(const struct mirage *m, int i, float *yaw_out,
                              float *lift_out, float *arc_out) {
    const mirage_config *c = &m->cfg;
    float gap = c->arc_spacing_deg * (float)M_PI/180.0f;
    float arc_i = scr_arc(m, i);
    *arc_out = arc_i;

    /* free-placed: pinned to an explicit yaw/lift, independent of any column. */
    if (screen_is_free(m, i)) {
        *yaw_out  = c->screen_yaw_deg[i] * (float)M_PI/180.0f;
        *lift_out = isfinite(c->screen_lift_m[i]) ? c->screen_lift_m[i] : 0.0f;
        return;
    }

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
        if (screen_is_free(m, j)) continue;
        int cc = layout_screen_col(m, j);
        float a = scr_arc(m, j) * (float)M_PI/180.0f;
        if (a > colw[cc]) colw[cc] = a;
    }

    /* total span incl gaps; walk left->right (col 0 leftmost = highest +yaw) */
    float span = gap * (float)(ncols - 1);
    for (int k = 0; k < ncols; k++) span += colw[k];
    float edge = span * 0.5f;          /* left edge of the whole wall */
    float yaw_c = 0.0f, anchor = 0.0f;
    for (int k = 0; k < ncols; k++) {
        float center = edge - colw[k] * 0.5f;
        if (k == myc)             yaw_c  = center;
        if (k == c->center_col)   anchor = center;   /* this column -> dead ahead */
        edge -= (colw[k] + gap);
    }
    /* anchor stays 0 when center_col is out of range -> the whole span is
     * centred (default); otherwise slide the wall so center_col sits at yaw 0. */
    *yaw_out = yaw_c - anchor;

    /* vertical: stack my column's members (array order = top first), centre the
     * whole stack on eye level so columns of differing heights line up. */
    float vgap = c->screen_distance_m * gap;   /* metres */
    float Hc = 0.0f; int cnt = 0;
    for (int j = 0; j < n; j++)
        if (!screen_is_free(m, j) && layout_screen_col(m, j) == myc) { Hc += scr_height(m, j, scr_arc(m, j)); cnt++; }
    if (cnt > 1) Hc += vgap * (float)(cnt - 1);
    float top = Hc * 0.5f, acc = 0.0f, mylift = 0.0f;
    for (int j = 0; j < n; j++)
        if (!screen_is_free(m, j) && layout_screen_col(m, j) == myc) {
            float h = scr_height(m, j, scr_arc(m, j));
            float center = top - acc - h * 0.5f;
            if (j == i) mylift = center;
            acc += h + vgap;
        }
    /* a column-placed screen may carry an explicit lift as an additive nudge:
     * it shifts the whole column up/down off its auto-centred position (e.g. to
     * anchor a stack's bottom panel at eye level instead of its centre). */
    if (isfinite(c->screen_lift_m[i])) mylift += c->screen_lift_m[i];
    *lift_out = mylift;
}

/* Public placement = base placement + the world's horizontal rotation (drag-to-spin
 * empty space, grab.c). One offset on every screen, so render and cursor picking
 * rotate together; the cursor's own direction stays in fixed world space. */
void layout_place(const struct mirage *m, int i, float *yaw_out, float *lift_out,
                  float *arc_out) {
    layout_place_base(m, i, yaw_out, lift_out, arc_out);
    *yaw_out += m->world_yaw;
}

/* Angular + vertical extent of the column-placed wall (free satellites excluded):
 * the yaw of its centre, its total angular width (rad), and the height of its top
 * edge (m) - the union of every column-placed screen's left/right yaw edges and
 * top. Used to hang the clock banner above the wall on the same curve. */
void layout_wall_extent(const struct mirage *m, float *yaw_c, float *arc_total,
                        float *top) {
    int n = m->n_screen > 0 ? m->n_screen : m->cfg.screen_count;
    if (n > MIRAGE_MAX_SCREENS) n = MIRAGE_MAX_SCREENS;
    float left = -1e30f, right = 1e30f, hi = -1e30f;
    bool any = false;
    for (int i = 0; i < n; i++) {
        if (screen_is_free(m, i)) continue;          /* satellites aren't the wall */
        float yaw, lift, arc;
        layout_place(m, i, &yaw, &lift, &arc);
        float halfw = arc * (float)M_PI/180.0f * 0.5f;
        float h2    = screen_height(m, i) * 0.5f;
        if (yaw + halfw > left)  left  = yaw + halfw;   /* +yaw = viewer's left */
        if (yaw - halfw < right) right = yaw - halfw;
        if (lift + h2   > hi)    hi    = lift + h2;
        any = true;
    }
    if (!any) { *yaw_c = 0.0f; *arc_total = 0.0f; *top = 0.0f; return; }
    *yaw_c     = 0.5f * (left + right);
    *arc_total = left - right;
    *top       = hi;
}

/* Place screen `i` on the arc: a yaw (its column) + a straight-up lift (its row),
 * both about the viewer at the origin. */
mat4 layout_model_matrix(const struct mirage *m, int i) {
    float yaw, lift, arc;
    layout_place(m, i, &yaw, &lift, &arc);
    mat4 R = m4_from_quat(q_from_euler_ypr(yaw, 0, 0));
    mat4 T = m4_translate(v3(0.0f, lift, 0.0f));
    /* The wall is fixed; vertical look-around is a dolly of the EYE along the cylinder
     * axis (m->world_lift), applied to eye_world in render.cpp - so nothing happens to
     * the per-screen placement here. layout_pick mirrors it via the cursor height. */
    return m4_mul(T, R);
}

/* The camera orientation that puts display `i` dead-centre: its column yaw, and a
 * pitch up to its row's lifted centre. Must match layout_model_matrix. */
void layout_focus_angles(const struct mirage *m, int i, float *yaw, float *pitch) {
    float y, lift, arc;
    layout_place(m, i, &y, &lift, &arc);
    *yaw   = y;
    /* eye sits world_lift above the axis, so the pitch to a screen at height `lift` is
     * measured from the lifted eye - keeps shake-to-gaze centred after a vertical dolly. */
    *pitch = atan2f(lift - m->world_lift, m->cfg.screen_distance_m);
}

/* Pointer pick - the input twin of layout_model_matrix. Screens sit on one cylinder
 * of radius d about the eye, so a wall-space look direction (yaw,pitch) maps to a
 * single point on that cylinder: azimuth = yaw, height = d*tan(pitch). Each screen is
 * then an axis-aligned rect there - azimuth in [yaw_i +/- arc/2], height in
 * [lift_i +/- H/2] - straight from layout_place, the same numbers render draws from.
 *
 * Pick = the screen that contains the point MOST CENTRALLY (largest edge margin), so
 * an overlap or a seam resolves to the natural screen instead of whichever happened
 * to be indexed first - that first-wins tie is exactly what stranded the side panel
 * in the old flat-strip nearest-rect mapping. On a miss we snap to the nearest screen
 * edge (distance measured in metres on the cylinder so azimuth and height compare on
 * one scale) and write the clamped dir back, so the cursor sticks to the wall and can
 * always slide into a neighbour. (Flat panels reuse the same azimuth/height rect; the
 * curve->plane error is sub-degree at these arcs.) */
int layout_pick(const struct mirage *m, float cyaw, float cpitch,
                float *u_out, float *v_out, bool *inside_out) {
    const mirage_config *c = &m->cfg;
    int n = m->n_screen > 0 ? m->n_screen : c->screen_count;
    if (n > MIRAGE_MAX_SCREENS) n = MIRAGE_MAX_SCREENS;
    if (inside_out) *inside_out = false;
    if (n <= 0) return -1;
    float d   = c->screen_distance_m;
    /* A look ray from the (vertically dollied) eye at pitch cpitch hits the cylinder of
     * radius d at world height world_lift + d*tan(cpitch); screens are placed at world
     * height `lift`, so add the eye lift here to keep clicks under the cursor. */
    float hgt = m->world_lift + d * tanf(cpitch);

    /* 1) direct hit: deepest-inside screen wins (no first-index bias on overlaps). */
    int best = -1; float best_margin = -1e30f, bu = 0, bv = 0;
    for (int i = 0; i < n; i++) {
        float yaw, lift, arc; layout_place(m, i, &yaw, &lift, &arc);
        float aw = arc * (float)M_PI/180.0f, H = screen_height(m, i);
        float u = (yaw + aw*0.5f - cyaw) / aw;   /* 0 = viewer-left edge, 1 = right */
        float v = (lift + H*0.5f - hgt)  / H;    /* 0 = top edge,         1 = bottom */
        if (u < 0.0f || u > 1.0f || v < 0.0f || v > 1.0f) continue;
        float margin = fminf(fminf(u, 1.0f - u), fminf(v, 1.0f - v));
        if (margin > best_margin) { best_margin = margin; best = i; bu = u; bv = v; }
    }
    if (best >= 0) { *u_out = bu; *v_out = bv; if (inside_out) *inside_out = true; return best; }

    /* 2) gap: the cursor stays free out here (the 3D arrow draws at its real dir).
     * We only need an injection TARGET for the desktop pointer, so report the
     * nearest screen's edge pixel - WITHOUT moving the cursor onto it. */
    float bd = 1e30f, byaw = 0, blift = 0, baw = 0, bH = 0, bcaz = 0, bch = 0;
    for (int i = 0; i < n; i++) {
        float yaw, lift, arc; layout_place(m, i, &yaw, &lift, &arc);
        float aw = arc * (float)M_PI/180.0f, H = screen_height(m, i);
        float caz = fminf(fmaxf(cyaw, yaw  - aw*0.5f), yaw  + aw*0.5f);
        float chg = fminf(fmaxf(hgt,  lift - H*0.5f),  lift + H*0.5f);
        float dx = (cyaw - caz) * d, dy = hgt - chg;        /* metres on the cylinder */
        float dd = dx*dx + dy*dy;
        if (dd < bd) { bd = dd; best = i;
            byaw = yaw; blift = lift; baw = aw; bH = H; bcaz = caz; bch = chg; }
    }
    *u_out = (byaw + baw*0.5f - bcaz) / baw;
    *v_out = (blift + bH*0.5f - bch)  / bH;
    return best;
}

/* sens_panel_compute now lives in render.cpp: it returns the snapshot of the
 * last Clay HUD layout pass (src/render.cpp, hud_render). grab.c hit-tests it
 * exactly as before - the geometry is just sourced from Clay now. */
