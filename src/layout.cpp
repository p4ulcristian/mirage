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
    mat4 model = m4_mul(T, R);
    /* world_pitch swings the whole wall up/down about the eye (rotation about the eye's
     * right axis, applied OUTSIDE the per-screen placement so every screen rotates as one).
     * layout_pick mirrors it via the cursor pitch so clicks stay aligned. */
    if (m->world_pitch != 0.0f)
        model = m4_mul(m4_from_quat(q_from_euler_ypr(0.0f, m->world_pitch, 0.0f)), model);
    return model;
}

/* The camera orientation that puts display `i` dead-centre: its column yaw, and a
 * pitch up to its row's lifted centre. Must match layout_model_matrix. */
void layout_focus_angles(const struct mirage *m, int i, float *yaw, float *pitch) {
    float y, lift, arc;
    layout_place(m, i, &y, &lift, &arc);
    *yaw   = y;
    *pitch = atan2f(lift, m->cfg.screen_distance_m);
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
    /* Subtract the wall's vertical swing so the cursor picks the screen that visually sits
     * under it (exact at centre yaw; the small off-centre error is sub-screen at sane swings). */
    float hgt = d * tanf(cpitch - m->world_pitch);

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

/* Sensitivity slider geometry. Hangs the slider row under the centre screen, BELOW
 * the FPS/help plaque stack (we recompute that stack's metres here so the row
 * lands just under it). The centre screen is the one nearest dead-ahead - smallest
 * |yaw|, lowest lift on a tie - exactly as render picks it for the plaques. The
 * handle's x is the current gain lerped across the track; render draws there and
 * grab maps the clicked cursor back to a gain with the same numbers. */
bool sens_panel_compute(const struct mirage *m, sens_panel *out) {
    int n = m->n_screen > 0 ? m->n_screen : m->cfg.screen_count;
    if (n > MIRAGE_MAX_SCREENS) n = MIRAGE_MAX_SCREENS;
    if (n <= 0) return false;

    /* centre screen: smallest |yaw|, then lowest lift (matches render.cpp) */
    int ci = -1; float best_yaw = 1e30f, best_lift = 1e30f, yaw_c = 0, lift_c = 0;
    for (int k = 0; k < n; k++) {
        float yw, lf, ar; layout_place(m, k, &yw, &lf, &ar);
        float ay = fabsf(yw);
        if (ay < best_yaw - 1e-4f || (ay < best_yaw + 1e-4f && lf < best_lift)) {
            best_yaw = ay; best_lift = lf; ci = k; yaw_c = yw; lift_c = lf;
        }
    }
    if (ci < 0) return false;

    float d  = m->cfg.screen_distance_m;
    float hh = screen_height(m, ci) * 0.5f;       /* centre screen half-height (m) */

    /* mirror the plaque stack in render.cpp to find where the help block bottoms
     * out, then drop the slider row a little below it. */
    const float fullH  = 0.11f;
    float yc_fps  = -hh - 0.05f - fullH * 0.5f;   /* FPS plaque tops the stack */
    float fps_bot = yc_fps - fullH * 0.5f;
    const float blockH = 0.26f;
    float yc_help  = fps_bot - 0.03f - blockH * 0.5f;
    float help_bot = yc_help - blockH * 0.5f;

    out->ci      = ci;
    out->d       = d;
    out->yaw_c   = yaw_c;
    out->lift_c  = lift_c;
    out->handle_h = 0.10f;
    out->handle_w = 0.03f;
    out->track_h  = 0.016f;
    out->row_y    = help_bot - 0.04f - out->handle_h * 0.5f;
    out->track_x0 = -0.23f;
    out->track_x1 =  0.23f;

    float g = m->cfg.yaw_gain;                     /* linked: yaw drives the handle */
    if (g < SENS_GAIN_MIN) g = SENS_GAIN_MIN;
    if (g > SENS_GAIN_MAX) g = SENS_GAIN_MAX;
    out->gain = g;
    float frac = (g - SENS_GAIN_MIN) / (SENS_GAIN_MAX - SENS_GAIN_MIN);
    out->handle_x = out->track_x0 + frac * (out->track_x1 - out->track_x0);

    /* DEFAULT button: a box to the right of the track */
    float def_cx = out->track_x1 + 0.16f, def_hw = 0.13f, def_hh = 0.06f;
    out->def_x0 = def_cx - def_hw; out->def_x1 = def_cx + def_hw;
    out->def_y0 = out->row_y - def_hh; out->def_y1 = out->row_y + def_hh;

    /* layout switcher: one clickable box per named layout, spread across the track
     * width in a row a little below the slider handle. Same compute-once contract:
     * render draws these rects and grab hit-tests them. */
    int nl = m->layouts.n;
    out->n_layout      = nl;
    out->active_layout = m->layouts.active;
    if (nl > 0) {
        const float bh = 0.07f, bgap = 0.02f;
        out->lay_y1 = out->row_y - out->handle_h * 0.5f - 0.06f;   /* top edge   */
        out->lay_y0 = out->lay_y1 - bh;                            /* bottom edge */
        float span = out->track_x1 - out->track_x0;
        float bw = (span - bgap * (float)(nl - 1)) / (float)nl;
        if (bw < 0.04f) bw = 0.04f;
        out->lay_w = bw;
        for (int k = 0; k < nl; k++)
            out->lay_cx[k] = out->track_x0 + bw * 0.5f + (bw + bgap) * (float)k;
    }

    /* environment switcher: a second box row below the layout row (or directly under
     * the slider if there are no layouts), same width/spacing contract. */
    int ne = MIRAGE_ENV_COUNT;
    if (ne > MIRAGE_MAX_ENVS) ne = MIRAGE_MAX_ENVS;
    out->n_env      = ne;
    out->active_env = m->active_env;
    if (ne > 0) {
        const float bh = 0.07f, bgap = 0.02f;
        float top = (nl > 0) ? out->lay_y0
                             : (out->row_y - out->handle_h * 0.5f - 0.06f);
        out->env_y1 = top - 0.03f;                  /* gap below the layout row */
        out->env_y0 = out->env_y1 - bh;
        float span = out->track_x1 - out->track_x0;
        float bw = (span - bgap * (float)(ne - 1)) / (float)ne;
        if (bw < 0.04f) bw = 0.04f;
        out->env_w = bw;
        for (int k = 0; k < ne; k++)
            out->env_cx[k] = out->track_x0 + bw * 0.5f + (bw + bgap) * (float)k;
    }

    /* environment brightness slider: a thin slider one row below the env tiles, the
     * handle position set from m->env_brightness. Same track span as the sens slider. */
    out->bri_track_h  = 0.012f;
    out->bri_handle_w = 0.026f;
    out->bri_handle_h = 0.05f;
    out->bri_x0 = out->track_x0;
    out->bri_x1 = out->track_x1;
    float bri_top = (ne > 0) ? out->env_y0
                  : (nl > 0) ? out->lay_y0
                             : (out->row_y - out->handle_h * 0.5f - 0.06f);
    out->bri_row_y = bri_top - 0.04f - out->bri_handle_h * 0.5f;
    float bf = (m->env_brightness - BRI_MIN) / (BRI_MAX - BRI_MIN);
    bf = bf < 0.0f ? 0.0f : (bf > 1.0f ? 1.0f : bf);
    out->bri_handle_x = out->bri_x0 + bf * (out->bri_x1 - out->bri_x0);

    /* window/screen transparency slider: same track, one row below the brightness
     * slider. Handle position from m->screen_opacity. grab.cpp drives it. */
    out->tr_track_h  = 0.012f;
    out->tr_handle_w = 0.026f;
    out->tr_handle_h = 0.05f;
    out->tr_x0 = out->track_x0;
    out->tr_x1 = out->track_x1;
    out->tr_row_y = out->bri_row_y - out->bri_handle_h * 0.5f - 0.04f - out->tr_handle_h * 0.5f;
    float tf = (m->screen_opacity - OPAC_MIN) / (OPAC_MAX - OPAC_MIN);
    tf = tf < 0.0f ? 0.0f : (tf > 1.0f ? 1.0f : tf);
    out->tr_handle_x = out->tr_x0 + tf * (out->tr_x1 - out->tr_x0);

    /* flat/curved toggle: a single button spanning the track, one row below the
     * transparency slider. Label/highlight reflect the current geometry; a click
     * flips it (grab.cpp). */
    const float geo_h = 0.06f;
    out->geo_x0 = out->track_x0;
    out->geo_x1 = out->track_x1;
    out->geo_y1 = out->tr_row_y - out->tr_handle_h * 0.5f - 0.04f;
    out->geo_y0 = out->geo_y1 - geo_h;
    out->geo_flat = (m->cfg.geometry == GEOM_FLAT);

    /* background-mode button (black / hdri / passthrough): one row below the
     * flat/curved toggle. A click cycles m->bg_mode (grab.cpp). */
    const float pt_h = 0.06f;
    out->pt_x0 = out->track_x0;
    out->pt_x1 = out->track_x1;
    out->pt_y1 = out->geo_y0 - 0.03f;
    out->pt_y0 = out->pt_y1 - pt_h;
    out->pt_mode = m->bg_mode;

    /* head-tilt (roll) toggle: one row below the background-mode button. A click flips
     * cfg.roll_damp between 1 (screens tilt with your head) and 0 (horizon-locked). */
    const float tl_h = 0.06f;
    out->tl_x0 = out->track_x0;
    out->tl_x1 = out->track_x1;
    out->tl_y1 = out->pt_y0 - 0.03f;
    out->tl_y0 = out->tl_y1 - tl_h;
    out->tl_on = (m->cfg.roll_damp > 0.5f) ? 1 : 0;
    return true;
}
