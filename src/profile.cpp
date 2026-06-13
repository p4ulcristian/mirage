/* profile.cpp - persisted device/tracking/optics calibration.
 *
 * mirage's geometry (where the screens sit) lives in layouts.conf, but the
 * DEVICE calibration - head-tracking feel, optics FOV, the neck model, which
 * output is the glasses - is global and per-rig, not per-layout. It lives here, in
 * a writable profile.toml loaded over the compiled defaults:
 *
 *     defaults (config.cpp)  ->  profile.toml  ->  active layout (geometry)
 *
 * Because a layout switch snapshots the WHOLE mirage_config, profile_apply re-stamps
 * the calibration fields after every switch (main.cpp / layouts_switch), so dialing
 * in a layout never clobbers your tracking/optics tuning. The mirage-cal pre-flight
 * tool writes this file; mirage only reads it (plus this apply-after-switch glue).
 *
 * The calibration field set is listed ONCE in CALIB_FIELDS below; load/save/apply
 * all iterate it, so adding a knob is a single line.
 */
#include "mirage.h"

#include <toml++/toml.hpp>

#include <cstring>
#include <cstdlib>
#include <fstream>
#include <filesystem>
#include <print>

/* The calibration fields, X-macro style: one row per knob, reused by load, save
 * and apply. KIND is f(loat) / i(nt) / b(ool); STR handles glasses_match. Add a
 * knob here and it persists everywhere automatically. */
#define CALIB_FIELDS(F) \
    F(i, pose_port)            \
    F(b, pose_oneeuro)         \
    F(f, pose_mincutoff)       \
    F(f, pose_beta)            \
    F(f, pose_drift_tau)       \
    F(f, yaw_gain)             \
    F(f, pitch_gain)           \
    F(f, roll_damp)            \
    F(f, read_deadband_deg)    \
    F(f, neck_fwd_m)           \
    F(f, neck_up_m)            \
    F(f, fov_deg)              \
    F(f, sharpen)

/* $MIRAGE_PROFILE, else $XDG_CONFIG_HOME/mirage/profile.toml, else
 * ~/.config/mirage/profile.toml. */
std::string profile_default_path() {
    if (const char *e = getenv("MIRAGE_PROFILE"); e && *e) return e;
    std::string base;
    if (const char *xdg = getenv("XDG_CONFIG_HOME"); xdg && *xdg) base = xdg;
    else { const char *home = getenv("HOME"); base = (home && *home ? home : ".");
           base += "/.config"; }
    return base + "/mirage/profile.toml";
}

/* Overlay the profile's calibration keys onto c. Only keys PRESENT in the file are
 * touched, so a sparse profile nudges one knob and leaves the rest at default.
 * Returns the number of keys applied; 0 if the file is missing or unparseable (no
 * profile yet is the normal first-run state, not an error). */
int profile_load(const char *path, mirage_config *c) {
    toml::table tbl;
    try { tbl = toml::parse_file(path); }
    catch (const toml::parse_error &) { return 0; }   /* missing / malformed -> none */

    int applied = 0;
    /* accept an int literal for a float key too (hand-edited `fov_deg = 46`). */
    auto get_f = [&](const char *k, float &dst) {
        if (auto v = tbl[k].value<double>()) { dst = (float)*v; applied++; }
        else if (auto vi = tbl[k].value<int64_t>()) { dst = (float)*vi; applied++; } };
    auto get_i = [&](const char *k, int &dst) {
        if (auto v = tbl[k].value<int64_t>()) { dst = (int)*v; applied++; } };
    auto get_b = [&](const char *k, bool &dst) {
        if (auto v = tbl[k].value<bool>()) { dst = *v; applied++; } };

#define F(kind, name) get_##kind(#name, c->name);
    CALIB_FIELDS(F)
#undef F
    if (auto v = tbl["glasses_match"].value<std::string_view>()) {
        std::snprintf(c->glasses_match, sizeof c->glasses_match, "%.*s",
                      (int)v->size(), v->data());
        applied++;
    }
    return applied;
}

/* Write the calibration subset to path (creating the directory). Returns false on
 * any I/O error. Hand-written TOML so the file stays small and commented. */
bool profile_save(const char *path, const mirage_config *c) {
    std::filesystem::path p(path);
    std::error_code ec;
    if (p.has_parent_path()) std::filesystem::create_directories(p.parent_path(), ec);

    std::ofstream o(path, std::ios::trunc);
    if (!o) { std::print(stderr, "profile: cannot write {}\n", path); return false; }
    o << std::boolalpha;     /* pose_oneeuro etc. as true/false, not 1/0, so they re-parse as bool */
    o << "# mirage calibration profile - device tracking + optics, per rig.\n"
      << "# Written by mirage-cal; loaded over the compiled defaults, kept across\n"
      << "# layout switches. Geometry/screen arrangement lives in layouts.conf.\n\n";
#define F(kind, name) o << #name " = " << c->name << "\n";
    CALIB_FIELDS(F)
#undef F
    o << "glasses_match = \"" << c->glasses_match << "\"\n";
    return (bool)o;
}

/* Copy the calibration fields from src onto dst, leaving everything else (geometry,
 * the runtime gaze toggle, ...) untouched. Called after cfg is (re)assigned from a
 * layout, so the calibration survives a layout switch. */
void profile_apply(mirage_config *dst, const mirage_config *src) {
#define F(kind, name) dst->name = src->name;
    CALIB_FIELDS(F)
#undef F
    std::memcpy(dst->glasses_match, src->glasses_match, sizeof dst->glasses_match);
}
