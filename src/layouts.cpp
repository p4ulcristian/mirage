/* layouts.cpp - named virtual-screen layouts, loaded from a TOML file.
 *
 * The project hardcodes a baseline geometry in config.cpp; this adds named
 * arrangements in `layouts.conf` (TOML) that the user toggles at runtime
 * (Alt+1/2/3, see grab.cpp). Each `[[layout]]` table starts from
 * mirage_config_defaults() and overrides only what differs, so a layout need
 * only state its changes.
 *
 *     active = "work"
 *
 *     [[layout]]
 *     name     = "work"
 *     distance = 2.0
 *     geometry = "cylinder"          # cylinder | flat (parsed via magic_enum)
 *     screens  = [
 *       { n = 1, col = 1, arc = 40 },          # VIRT1 -> column 1 (1-based n)
 *       { n = 3, yaw = 32.5, lift = 0, arc = 23 },  # VIRT3 pinned (free placement)
 *     ]
 *
 * A screen entry is keyed by VIRT number `n` (1-based). It carries the screen's
 * masonry cell - `w,h` (resolution) and `x,y` (2D desktop position) - and the 3D
 * arc placement (yaw/arc/lift) is PROJECTED from that cell at load (see
 * project_cells), so the desktop tiling and the on-glasses wall derive from ONE set
 * of numbers and can't drift. scripts/setup_displays.py reads the same w,h,x,y to
 * create the Hyprland outputs. `col` (column grid) or an explicit `yaw` (+ `lift`)
 * still override the projection for hand-placed / fanned layouts.
 */
#include "mirage.h"
#include "magic_enum.hpp"

#include <toml++/toml.hpp>

#include <cstdio>
#include <cstring>
#include <cmath>
#include <string>
#include <string_view>
#include <algorithm>
#include <cctype>
#include <print>

/* Parse a friendly geometry name ("cylinder"/"flat") through magic_enum: match
 * the lowercased, GEOM_-stripped enum name, so adding a geometry needs no edit
 * here. Returns `fallback` if nothing matches. */
static int parse_geometry(std::string_view s, int fallback) {
    for (auto v : magic_enum::enum_values<mirage_geometry>()) {
        std::string n(magic_enum::enum_name(v));        /* e.g. "GEOM_CYLINDER" */
        auto us = n.find('_');
        if (us != std::string::npos) n = n.substr(us + 1);
        std::transform(n.begin(), n.end(), n.begin(),
                       [](unsigned char c) { return (char)std::tolower(c); });
        if (n == s) return (int)v;
    }
    return fallback;
}

/* One screen's 2D masonry cell: resolution (w,h) + desktop position (x,y), in px.
 * Collected while parsing; projected onto the cylinder once the whole layout is in. */
struct screen_cell { double w, h, x, y; bool set; };

/* Apply one `screens = [ { n = .. } ]` entry (1-based VIRT number `n`). Captures the
 * masonry cell into cells[idx] and any explicit placement override (col / yaw / lift
 * / arc). */
static void apply_screen(mirage_config *cfg, const toml::table &sc, screen_cell *cells) {
    int n = (int)sc["n"].value_or<int64_t>(0);
    if (n < 1 || n > MIRAGE_MAX_SCREENS) return;
    int idx = n - 1;
    if (cfg->screen_count < n) cfg->screen_count = n;
    if (auto v = sc["col"].value<int64_t>())  { cfg->screen_col[idx] = (int)*v; cfg->explicit_layout = true; }
    if (auto v = sc["arc"].value<double>())     cfg->screen_arc[idx]     = (float)*v;
    if (auto v = sc["yaw"].value<double>())     cfg->screen_yaw_deg[idx] = (float)*v;
    if (auto v = sc["lift"].value<double>())    cfg->screen_lift_m[idx]  = (float)*v;
    if (auto b = sc["follow"].value<bool>(); b && *b) cfg->follow_screen = idx;  /* head-locked */
    auto w = sc["w"].value<double>(), h = sc["h"].value<double>();
    auto x = sc["x"].value<double>(), y = sc["y"].value<double>();
    if (w && h && x && y) cells[idx] = { *w, *h, *x, *y, true };
}

/* Project the masonry cells onto the cylinder, filling each screen's yaw/arc/lift
 * (the same free-placement fields an explicit `yaw=` would set). Mirrors the math in
 * scripts/setup_displays.py so the desktop tiling and the on-glasses wall derive from
 * ONE set of cells. wall_arc = total angular width of the masonry (deg); eye_y = the
 * masonry y (px) sitting at eye level; gap = bezel inset fraction. A screen already
 * pinned with an explicit yaw is left untouched. No-op when wall_arc <= 0 (a layout
 * that places by column/yaw instead of by cells). */
static void project_cells(mirage_config *cfg, const screen_cell *cells,
                          double wall_arc, double eye_y, double gap) {
    if (wall_arc <= 0.0) return;
    double x0 = 1e30, y0 = 1e30, xr = -1e30; bool any = false;
    for (int i = 0; i < MIRAGE_MAX_SCREENS; i++) {
        if (!cells[i].set || i == cfg->follow_screen) continue;   /* follow screen isn't on the wall */
        any = true;
        x0 = std::min(x0, cells[i].x);
        y0 = std::min(y0, cells[i].y);
        xr = std::max(xr, cells[i].x + cells[i].w);
    }
    if (!any) return;
    double Wt = xr - x0;
    if (Wt <= 0.0) return;
    double k = wall_arc * (M_PI/180.0) * cfg->screen_distance_m / Wt;   /* metres per px */
    for (int i = 0; i < MIRAGE_MAX_SCREENS; i++) {
        if (!cells[i].set || i == cfg->follow_screen || std::isfinite(cfg->screen_yaw_deg[i])) continue;
        double cx = (cells[i].x - x0) + cells[i].w * 0.5;
        cfg->screen_yaw_deg[i] = (float)(wall_arc * (Wt * 0.5 - cx) / Wt);   /* +yaw = left */
        cfg->screen_arc[i]     = (float)(wall_arc * cells[i].w / Wt * (1.0 - gap));
        cfg->screen_lift_m[i]  = (float)((eye_y - (cells[i].y - y0) - cells[i].h * 0.5) * k);
    }
}

/* Apply the global (non-screen) keys of a [[layout]] table. */
static void apply_globals(mirage_config *cfg, const toml::table &t) {
    if (auto v = t["distance"].value<double>())    cfg->screen_distance_m = (float)*v;
    if (auto v = t["spacing"].value<double>())     cfg->arc_spacing_deg   = (float)*v;
    if (auto v = t["center_col"].value<int64_t>()) cfg->center_col        = (int)*v;
    if (auto v = t["arc"].value<double>())         cfg->screen_arc_deg    = (float)*v;
    if (auto v = t["fov"].value<double>())         cfg->fov_deg           = (float)*v;
    if (auto v = t["sharpen"].value<double>())     cfg->sharpen           = (float)*v;
    if (auto v = t["radius"].value<double>())      cfg->screen_radius     = (float)*v;
    if (auto v = t["msaa"].value<int64_t>())       cfg->msaa_samples      = (int)*v;
    if (auto b = t["mipmap"].value<bool>())        cfg->mipmap            = *b;
    if (auto v = t["yaw_gain"].value<double>())    cfg->yaw_gain          = (float)*v;
    if (auto v = t["pitch_gain"].value<double>())  cfg->pitch_gain        = (float)*v;
    if (auto v = t["geometry"].value<std::string>())
        cfg->geometry = parse_geometry(*v, cfg->geometry);
    if (auto b = t["hdri"].value<bool>())          cfg->hdri_on = *b;
    else if (auto v = t["hdri"].value<std::string>())
        cfg->hdri_on = (*v == "on" || *v == "true" || *v == "1");
}

int layouts_load(mirage_layouts *L, const char *path) {
    memset(L, 0, sizeof *L);

    /* a missing file is fine (built-in defaults are used); only a real TOML
     * syntax error is worth a complaint. */
    if (FILE *probe = fopen(path, "r")) fclose(probe);
    else return 0;

    toml::table tbl;
    try {
        tbl = toml::parse_file(path);
    } catch (const toml::parse_error &e) {
        std::print(stderr, "layouts: parse error in {}: {}\n", path,
                   std::string(e.description()));
        return 0;
    }

    std::string want_active = tbl["active"].value_or(std::string{});

    if (auto arr = tbl["layout"].as_array()) {
        for (auto &node : *arr) {
            const toml::table *t = node.as_table();
            if (!t) continue;
            if (L->n >= MIRAGE_MAX_LAYOUTS) {
                std::print(stderr, "layouts: too many layouts (max {})\n", MIRAGE_MAX_LAYOUTS);
                break;
            }
            int cur = L->n++;
            mirage_config *cfg = &L->l[cur].cfg;
            mirage_config_defaults(cfg);
            cfg->screen_count = 0;                  /* count up from screen entries */
            std::string name = (*t)["name"].value_or(std::string{});
            snprintf(L->l[cur].name, sizeof L->l[cur].name, "%s", name.c_str());
            apply_globals(cfg, *t);
            screen_cell cells[MIRAGE_MAX_SCREENS] = {};
            if (auto screens = (*t)["screens"].as_array())
                for (auto &s : *screens)
                    if (const toml::table *st = s.as_table())
                        apply_screen(cfg, *st, cells);
            /* derive the 3D arc from the masonry cells (no-op if the layout has no
             * cells / no wall_arc - then it placed by column or explicit yaw above). */
            project_cells(cfg, cells, (*t)["wall_arc"].value_or(0.0),
                          (*t)["eye_y"].value_or(0.0), (*t)["gap"].value_or(0.0));
        }
    }

    /* resolve the active layout by name; default to the first */
    L->active = 0;
    if (!want_active.empty())
        for (int i = 0; i < L->n; i++)
            if (!strcmp(L->l[i].name, want_active.c_str())) { L->active = i; break; }
    return L->n;
}

void layouts_switch(struct mirage *m, int idx) {
    mirage_layouts *L = &m->layouts;
    if (L->n == 0 || idx < 0 || idx >= L->n) return;   /* no such layout: ignore */
    int geom = m->cfg.geometry;                        /* preserve the flat/curved toggle */
    L->active = idx;
    m->cfg = L->l[idx].cfg;
    m->cfg.geometry = geom;
    if (m->have_profile) profile_apply(&m->cfg, &m->calib_cfg);   /* keep calibration */
    m->layout_dirty = true;
    std::print(stderr, "layouts: switched to {} ({})\n", idx + 1, L->l[idx].name);
}
