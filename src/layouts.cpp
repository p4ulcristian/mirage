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
 * A screen entry is keyed by VIRT number `n` (1-based). `col` places it on the
 * column grid; `yaw` (+ `lift`) pins it to an exact pose (free placement).
 */
#include "mirage.h"
#include "magic_enum.hpp"

#include <toml++/toml.hpp>

#include <cstdio>
#include <cstring>
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

/* Apply one `screens = [ { n = .. } ]` entry (1-based VIRT number `n`). */
static void apply_screen(mirage_config *cfg, const toml::table &sc) {
    int n = (int)sc["n"].value_or<int64_t>(0);
    if (n < 1 || n > MIRAGE_MAX_SCREENS) return;
    int idx = n - 1;
    if (cfg->screen_count < n) cfg->screen_count = n;
    if (auto v = sc["col"].value<int64_t>())  { cfg->screen_col[idx] = (int)*v; cfg->explicit_layout = true; }
    if (auto v = sc["arc"].value<double>())     cfg->screen_arc[idx]     = (float)*v;
    if (auto v = sc["yaw"].value<double>())     cfg->screen_yaw_deg[idx] = (float)*v;
    if (auto v = sc["lift"].value<double>())    cfg->screen_lift_m[idx]  = (float)*v;
}

/* Apply the global (non-screen) keys of a [[layout]] table. */
static void apply_globals(mirage_config *cfg, const toml::table &t) {
    if (auto v = t["distance"].value<double>())    cfg->screen_distance_m = (float)*v;
    if (auto v = t["slab_depth"].value<double>())  cfg->slab_depth_m      = (float)*v;
    if (auto v = t["spacing"].value<double>())     cfg->arc_spacing_deg   = (float)*v;
    if (auto v = t["center_col"].value<int64_t>()) cfg->center_col        = (int)*v;
    if (auto v = t["arc"].value<double>())         cfg->screen_arc_deg    = (float)*v;
    if (auto v = t["fov"].value<double>())         cfg->fov_deg           = (float)*v;
    if (auto v = t["sharpen"].value<double>())     cfg->sharpen           = (float)*v;
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
            mirage_config_defaults(&L->l[cur].cfg);
            L->l[cur].cfg.screen_count = 0;         /* count up from screen entries */
            std::string name = (*t)["name"].value_or(std::string{});
            snprintf(L->l[cur].name, sizeof L->l[cur].name, "%s", name.c_str());
            apply_globals(&L->l[cur].cfg, *t);
            if (auto screens = (*t)["screens"].as_array())
                for (auto &s : *screens)
                    if (const toml::table *st = s.as_table())
                        apply_screen(&L->l[cur].cfg, *st);
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
    bool gaze = m->cfg.gaze_cursor;                    /* preserve runtime toggle */
    L->active = idx;
    m->cfg = L->l[idx].cfg;
    m->cfg.gaze_cursor = gaze;
    if (m->have_profile) profile_apply(&m->cfg, &m->calib);   /* keep calibration */
    m->layout_dirty = true;
    std::print(stderr, "layouts: switched to {} ({})\n", idx + 1, L->l[idx].name);
}
