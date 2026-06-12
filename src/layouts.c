/* layouts.c - named virtual-screen layouts loaded from a plain-text config file.
 *
 * The project otherwise hardcodes geometry in config.c; this adds a small, no-deps
 * `key = value` reader so several arrangements can live in `layouts.conf` and be
 * toggled at runtime (Alt+1/2/3, see grab.c). Each `[layout NAME]` section starts
 * from mirage_config_defaults() and overrides a handful of keys, so a section only
 * needs to say what differs from the baseline wall.
 *
 * Grammar (one statement per line; '#' or ';' starts a comment):
 *
 *     active = work                 # which layout to apply at startup (by name)
 *
 *     [layout work]
 *     distance   = 2.0              # metres eye->screen
 *     geometry   = cylinder         # cylinder | flat
 *     slab_depth = 0.05             # screen thickness (m); 0 = flat panels
 *     spacing    = 1.0              # gap between screens/columns (deg)
 *     arc        = 40               # default angular width (deg)
 *     screen 1 col=1 arc=40         # VIRT1 -> column 1, 40deg wide  (1-based)
 *     screen 3 yaw=32.5 lift=0 arc=23   # VIRT3 pinned to an explicit pose
 *
 * A `screen` line is keyed by VIRT number (1-based). `col=` places it on the
 * column grid; `yaw=`(+lift) pins it to an exact pose instead (free placement).
 */
#include "mirage.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <ctype.h>

/* trim leading/trailing ASCII whitespace in place, return the trimmed start. */
static char *trim(char *s) {
    while (*s && isspace((unsigned char)*s)) s++;
    char *e = s + strlen(s);
    while (e > s && isspace((unsigned char)e[-1])) e--;
    *e = '\0';
    return s;
}

/* apply one `screen N key=val ...` line to cfg (idx is 0-based VIRT number-1). */
static void parse_screen_line(mirage_config *cfg, char *rest) {
    char *save = NULL;
    char *num  = strtok_r(rest, " \t", &save);
    if (!num) return;
    int n = atoi(num);                       /* 1-based VIRT number */
    if (n < 1 || n > MIRAGE_MAX_SCREENS) return;
    int idx = n - 1;
    if (cfg->screen_count < n) cfg->screen_count = n;

    for (char *tok = strtok_r(NULL, " \t", &save); tok;
              tok = strtok_r(NULL, " \t", &save)) {
        char *eq = strchr(tok, '=');
        if (!eq) continue;
        *eq = '\0';
        const char *key = tok, *val = eq + 1;
        if      (!strcmp(key, "col"))  { cfg->screen_col[idx] = atoi(val); cfg->explicit_layout = true; }
        else if (!strcmp(key, "arc"))  { cfg->screen_arc[idx] = (float)atof(val); }
        else if (!strcmp(key, "yaw"))  { cfg->screen_yaw_deg[idx] = (float)atof(val); }
        else if (!strcmp(key, "lift")) { cfg->screen_lift_m[idx]  = (float)atof(val); }
        else fprintf(stderr, "layouts: unknown screen key '%s'\n", key);
    }
}

/* apply one `key = value` line (already split) to the current section's cfg. */
static void parse_kv(mirage_config *cfg, const char *key, const char *val) {
    if      (!strcmp(key, "distance"))   cfg->screen_distance_m = (float)atof(val);
    else if (!strcmp(key, "slab_depth")) cfg->slab_depth_m      = (float)atof(val);
    else if (!strcmp(key, "spacing"))    cfg->arc_spacing_deg   = (float)atof(val);
    else if (!strcmp(key, "center_col")) cfg->center_col        = atoi(val);
    else if (!strcmp(key, "arc"))        cfg->screen_arc_deg    = (float)atof(val);
    else if (!strcmp(key, "fov"))        cfg->fov_deg           = (float)atof(val);
    else if (!strcmp(key, "sharpen"))    cfg->sharpen           = (float)atof(val);
    else if (!strcmp(key, "yaw_gain"))   cfg->yaw_gain          = (float)atof(val);
    else if (!strcmp(key, "pitch_gain")) cfg->pitch_gain        = (float)atof(val);
    else if (!strcmp(key, "geometry"))
        cfg->geometry = !strcmp(val, "flat") ? GEOM_FLAT : GEOM_CYLINDER;
    else if (!strcmp(key, "hdri"))
        cfg->hdri_on = !strcmp(val, "on") || !strcmp(val, "true") || !strcmp(val, "1");
    else
        fprintf(stderr, "layouts: unknown key '%s'\n", key);
}

int layouts_load(mirage_layouts *L, const char *path) {
    memset(L, 0, sizeof *L);
    FILE *f = fopen(path, "r");
    if (!f) return 0;

    char want_active[32] = "";   /* `active = NAME`, resolved to an index at the end */
    int  cur = -1;               /* index of the section being filled, -1 = none yet */
    char line[512];
    while (fgets(line, sizeof line, f)) {
        /* strip comments, then trim */
        char *hash = line + strcspn(line, "#;");
        *hash = '\0';
        char *s = trim(line);
        if (!*s) continue;

        if (*s == '[') {                                 /* [layout NAME] */
            char *close = strchr(s, ']');
            if (close) *close = '\0';
            char *body = trim(s + 1);
            if (!strncmp(body, "layout", 6)) body = trim(body + 6);
            if (L->n >= MIRAGE_MAX_LAYOUTS) {
                fprintf(stderr, "layouts: too many layouts (max %d)\n", MIRAGE_MAX_LAYOUTS);
                cur = -1;
                continue;
            }
            cur = L->n++;
            mirage_config_defaults(&L->l[cur].cfg);
            L->l[cur].cfg.screen_count = 0;              /* count up from screen lines */
            snprintf(L->l[cur].name, sizeof L->l[cur].name, "%s", body);
            continue;
        }

        if (!strncmp(s, "screen", 6) && isspace((unsigned char)s[6])) {
            if (cur >= 0) parse_screen_line(&L->l[cur].cfg, s + 7);
            continue;
        }

        /* key = value */
        char *eq = strchr(s, '=');
        if (!eq) { fprintf(stderr, "layouts: ignoring line '%s'\n", s); continue; }
        *eq = '\0';
        char *key = trim(s), *val = trim(eq + 1);
        if (cur < 0) {                                   /* global, before any section */
            if (!strcmp(key, "active")) snprintf(want_active, sizeof want_active, "%s", val);
            else fprintf(stderr, "layouts: stray key '%s' before any [layout]\n", key);
            continue;
        }
        parse_kv(&L->l[cur].cfg, key, val);
    }
    fclose(f);

    /* resolve the active layout by name; default to the first */
    L->active = 0;
    if (*want_active)
        for (int i = 0; i < L->n; i++)
            if (!strcmp(L->l[i].name, want_active)) { L->active = i; break; }
    return L->n;
}

void layouts_switch(struct mirage *m, int idx) {
    mirage_layouts *L = &m->layouts;
    if (L->n == 0 || idx < 0 || idx >= L->n) return;   /* no such layout: ignore */
    bool gaze = m->cfg.gaze_cursor;                    /* preserve runtime toggle */
    L->active = idx;
    m->cfg = L->l[idx].cfg;
    m->cfg.gaze_cursor = gaze;
    m->layout_dirty = true;
    fprintf(stderr, "layouts: switched to %d (%s)\n", idx + 1, L->l[idx].name);
}
