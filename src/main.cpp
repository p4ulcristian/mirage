#include "mirage.h"
#include "pose.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <signal.h>
#include <unistd.h>
#include <time.h>
#include <poll.h>
#include <print>

#include "ext-image-capture-source-v1-client-protocol.h"
#include "ext-image-copy-capture-v1-client-protocol.h"
#include "linux-dmabuf-unstable-v1-client-protocol.h"
#include "xdg-shell-client-protocol.h"
#include "pointer-constraints-unstable-v1-client-protocol.h"
#include "relative-pointer-unstable-v1-client-protocol.h"
#include "wlr-virtual-pointer-unstable-v1-client-protocol.h"
#include <wayland-egl.h>

static struct mirage M;
static volatile sig_atomic_t g_stop = 0;
static volatile sig_atomic_t g_recenter = 0;
static volatile sig_atomic_t g_smooth_toggle = 0;
static void on_sig(int s) { (void)s; g_stop = 1; }
static void on_recenter(int s) { (void)s; g_recenter = 1; }     /* SIGUSR1 */
static void on_smooth(int s) { (void)s; g_smooth_toggle = 1; }  /* SIGHUP  */
static volatile sig_atomic_t g_calib = 0;
static void on_calib(int s) { (void)s; g_calib = 1; }           /* SIGUSR2: re-run wizard */

/* mirage renders as a fullscreen xdg-shell window on the glasses (DP-1), which
 * Hyprland page-flips straight to the panel (direct scanout). The glasses output
 * is auto-detected by description (cfg.glasses_match). Backing size before the
 * compositor's fullscreen configure arrives: */
#define WIN_W 1920
#define WIN_H 1080
static struct xdg_wm_base  *g_wm_base  = NULL;
static struct xdg_toplevel *g_toplevel = NULL;
static int32_t g_win_cfg_w = 0, g_win_cfg_h = 0;

static void wm_ping(void *d, struct xdg_wm_base *b, uint32_t serial) {
    (void)d; xdg_wm_base_pong(b, serial);
}
static const struct xdg_wm_base_listener WM_BASE_LISTENER = { .ping = wm_ping };

static void xsurf_configure(void *d, struct xdg_surface *xs, uint32_t serial) {
    (void)d; xdg_surface_ack_configure(xs, serial); M.configured = true;
}
static const struct xdg_surface_listener XSURF_LISTENER = { .configure = xsurf_configure };

static void xtop_configure(void *d, struct xdg_toplevel *t, int32_t w, int32_t h,
                           struct wl_array *states) {
    (void)d;(void)t;(void)states;
    if (w > 0 && h > 0) { g_win_cfg_w = w; g_win_cfg_h = h; }
}
static void xtop_close(void *d, struct xdg_toplevel *t) {
    (void)d;(void)t; M.running = false; g_stop = 1;
}
static const struct xdg_toplevel_listener XTOP_LISTENER = {
    .configure = xtop_configure, .close = xtop_close,
};

/* ---- wl_output discovery ---- */
static void out_geometry(void *d, struct wl_output *o, int32_t x, int32_t y,
        int32_t pw, int32_t ph, int32_t sp, const char *make, const char *model,
        int32_t tr) {
    (void)o;(void)x;(void)y;(void)pw;(void)ph;(void)sp;(void)tr;
    int idx = (int)(intptr_t)d;
    M.pending[idx].desc = std::string(make ? make : "") + " " + (model ? model : "");
}
static void out_mode(void *d, struct wl_output *o, uint32_t flags,
        int32_t w, int32_t h, int32_t refresh) {
    (void)o;(void)refresh;
    int idx = (int)(intptr_t)d;
    if (flags & WL_OUTPUT_MODE_CURRENT) { M.pending[idx].w = w; M.pending[idx].h = h; }
}
static void out_done(void *d, struct wl_output *o) {
    (void)o; int idx = (int)(intptr_t)d; M.pending[idx].done = true;
}
static void out_scale(void *d, struct wl_output *o, int32_t s) { (void)d;(void)o;(void)s; }
static void out_name(void *d, struct wl_output *o, const char *name) {
    (void)o; int idx = (int)(intptr_t)d;
    if (name) M.pending[idx].name = name;
}
static void out_description(void *d, struct wl_output *o, const char *desc) {
    (void)o; int idx = (int)(intptr_t)d;
    if (desc) M.pending[idx].desc = desc;
}
static const struct wl_output_listener OUTPUT_LISTENER = {
    .geometry = out_geometry, .mode = out_mode, .done = out_done,
    .scale = out_scale, .name = out_name, .description = out_description,
};

/* ---- registry ---- */
static void reg_global(void *d, struct wl_registry *r, uint32_t name,
                       const char *iface, uint32_t ver) {
    (void)d;
    if (!strcmp(iface, wl_compositor_interface.name)) {
        M.compositor = (struct wl_compositor*)wl_registry_bind(r, name, &wl_compositor_interface, 4);
    } else if (!strcmp(iface, ext_output_image_capture_source_manager_v1_interface.name)) {
        M.capture_src_mgr = (struct ext_output_image_capture_source_manager_v1*)wl_registry_bind(r, name,
            &ext_output_image_capture_source_manager_v1_interface, 1);
    } else if (!strcmp(iface, ext_image_copy_capture_manager_v1_interface.name)) {
        M.copy_capture_mgr = (struct ext_image_copy_capture_manager_v1*)wl_registry_bind(r, name,
            &ext_image_copy_capture_manager_v1_interface, 1);
    } else if (!strcmp(iface, zwp_linux_dmabuf_v1_interface.name)) {
        M.dmabuf = (struct zwp_linux_dmabuf_v1*)wl_registry_bind(r, name, &zwp_linux_dmabuf_v1_interface,
                                    ver < 3 ? ver : 3);
    } else if (!strcmp(iface, wl_shm_interface.name)) {
        M.shm = (struct wl_shm*)wl_registry_bind(r, name, &wl_shm_interface, 1);
    } else if (!strcmp(iface, wl_seat_interface.name)) {
        M.seat = (struct wl_seat*)wl_registry_bind(r, name, &wl_seat_interface, ver < 5 ? ver : 5);
    } else if (!strcmp(iface, zwp_pointer_constraints_v1_interface.name)) {
        M.pointer_constraints = (struct zwp_pointer_constraints_v1*)wl_registry_bind(r, name,
                                  &zwp_pointer_constraints_v1_interface, 1);
    } else if (!strcmp(iface, zwp_relative_pointer_manager_v1_interface.name)) {
        M.rel_pointer_mgr = (struct zwp_relative_pointer_manager_v1*)wl_registry_bind(r, name,
                                  &zwp_relative_pointer_manager_v1_interface, 1);
    } else if (!strcmp(iface, zwlr_virtual_pointer_manager_v1_interface.name)) {
        /* v2 required for create_virtual_pointer_with_output */
        M.vpointer_mgr = (struct zwlr_virtual_pointer_manager_v1*)wl_registry_bind(r, name,
                                  &zwlr_virtual_pointer_manager_v1_interface,
                                  ver < 2 ? ver : 2);
    } else if (!strcmp(iface, xdg_wm_base_interface.name)) {
        g_wm_base = (struct xdg_wm_base*)wl_registry_bind(r, name, &xdg_wm_base_interface, ver < 4 ? ver : 4);
        xdg_wm_base_add_listener(g_wm_base, &WM_BASE_LISTENER, NULL);
    } else if (!strcmp(iface, wl_output_interface.name)) {
        if (M.n_pending < 16) {
            int idx = M.n_pending++;
            struct wl_output *o = (struct wl_output*)wl_registry_bind(r, name, &wl_output_interface, 4);
            M.pending[idx].wl = o;
            wl_output_add_listener(o, &OUTPUT_LISTENER, (void*)(intptr_t)idx);
        }
    }
}
static void reg_remove(void *d, struct wl_registry *r, uint32_t name) {
    (void)d;(void)r;(void)name;
}
static const struct wl_registry_listener REGISTRY_LISTENER = {
    .global = reg_global, .global_remove = reg_remove,
};

static int classify_outputs(void) {
    /* glasses output (Wayland): the one whose description/name matches the glasses
     * (cfg.glasses_match, e.g. "SmartGlasses"). We render a fullscreen window onto
     * it; if it's not present we fall back to a normal window so the scene is still
     * usable on the laptop. */
    int glasses = -1;
    for (int i = 0; i < M.n_pending; i++) {
        const std::string &n = M.pending[i].name, &de = M.pending[i].desc;
        if (de.find(M.cfg.glasses_match) != std::string::npos ||
            n.find(M.cfg.glasses_match)  != std::string::npos ||
            de.find("RayNeo")            != std::string::npos) {
            glasses = i; break;
        }
    }
    if (glasses >= 0) {
        M.glasses_out  = M.pending[glasses].wl;
        M.glasses_name = M.pending[glasses].name;
        M.glasses_desc = M.pending[glasses].desc;
        M.glasses_w = M.pending[glasses].w;
        M.glasses_h = M.pending[glasses].h;
    } else {
        M.glasses_name = "windowed";
        M.glasses_w = WIN_W; M.glasses_h = WIN_H;
    }

    /* virtual screens: outputs named VIRT*, in name order */
    for (int pass = 1; pass <= MIRAGE_MAX_SCREENS; pass++) {
        char want[16]; snprintf(want, sizeof want, "VIRT%d", pass);
        for (int i = 0; i < M.n_pending; i++) {
            if (M.pending[i].name == want && M.n_screen < MIRAGE_MAX_SCREENS) {
                screen_t *s = &M.screen[M.n_screen];
                memset(s, 0, sizeof *s);
                s->wl = M.pending[i].wl;
                snprintf(s->name, sizeof s->name, "%s", want);
                s->width  = M.pending[i].w;
                s->height = M.pending[i].h;
                s->index  = M.n_screen;
                s->image  = EGL_NO_IMAGE_KHR;
                s->dmabuf_fd = -1;
                M.n_screen++;
                break;
            }
        }
    }
    std::print(stderr, "mirage: glasses={} ({}x{}) [{}]; {} virtual screens\n",
            M.glasses_name, M.glasses_w, M.glasses_h,
            M.glasses_desc, M.n_screen);
    return M.n_screen > 0 ? 0 : -1;
}

int main(void) {
    mirage_config_defaults(&M.cfg);
    M.zoom = 1.0f;
    M.env_brightness = BRI_DEF;   /* HUD brightness slider starts centred (1.0 = as tuned) */
    M.screen_opacity = OPAC_DEF;  /* windows fully opaque until the transparency slider moves */
    M.bg_mode = BG_HDRI;          /* default background = the environment dome */
    M.track_mode = TRACK_NECK;    /* default 3DoF+ (neck model) - grounds head turns */

    /* device/tracking/optics calibration: overlay profile.toml on the defaults and
     * stash the result, so it can be re-stamped after every layout switch (layouts
     * snapshot the whole config). No file yet = first run, defaults stand. */
    { std::string pp = profile_default_path();
      int np = profile_load(pp.c_str(), &M.cfg);
      M.calib_cfg = M.cfg;
      M.have_profile = true;
      if (np > 0)
          std::print(stderr, "mirage: loaded {} calibration key(s) from {}\n", np, pp);
      /* first run (no profile yet) -> full guided wizard; otherwise the quick
       * "look forward, hold still" centre that runs every launch. */
      calib_init(&M, np > 0);
    }

    /* persisted HUD selections (geometry toggle, brightness, environment, active
     * layout): read once here, applied below after the layout is resolved. Kept
     * apart from the calibration profile so they don't ride the profile_apply
     * re-stamp. No file yet = first run, the compiled/layout defaults stand. */
    mirage_ui_state ui{};
    { std::string up = ui_state_default_path();
      if (ui_state_load(up.c_str(), &ui) > 0)
          std::print(stderr, "mirage: loaded HUD state from {}\n", up); }

    /* named layouts: if layouts.conf is present, apply the active one over the
     * hardcoded defaults. Toggle between them at runtime with Alt+1/2/3 (grab.c).
     * Path overridable via $MIRAGE_LAYOUTS; otherwise the cwd's layouts.conf. */
    { const char *lp = getenv("MIRAGE_LAYOUTS");
      if (!lp || !*lp) lp = "layouts.conf";
      if (layouts_load(&M.layouts, lp) > 0) {
          /* restore the last-active layout by name (overrides layouts.conf's own
           * `active`), so the wall comes back to the one the user left running. */
          if (ui.has_layout)
              for (int i = 0; i < M.layouts.n; i++)
                  if (!strcmp(M.layouts.l[i].name, ui.layout)) { M.layouts.active = i; break; }
          M.cfg = M.layouts.l[M.layouts.active].cfg;
          profile_apply(&M.cfg, &M.calib_cfg);   /* keep calibration over the layout */
          std::print(stderr, "mirage: loaded {} layout(s) from {}, active '{}'\n",
                  M.layouts.n, lp, M.layouts.l[M.layouts.active].name);
      } }

    /* apply the remaining persisted HUD selections over the resolved layout: the
     * flat/curved toggle, brightness, and environment all override the layout's own
     * values (they're user choices, not part of the layout). env_switch just stamps
     * cfg.hdri_* + flags a reload; the first rendered frame does the GL work. */
    if (ui.has_geometry)   M.cfg.geometry  = ui.geometry;
    if (ui.has_brightness) M.env_brightness = ui.brightness;
    if (ui.has_env)        env_switch(&M, ui.env);
    if (ui.has_bg_mode)    M.bg_mode       = ui.bg_mode;     /* render starts the cam on frame 1 if PASSTHROUGH */
    if (ui.has_opacity)    M.screen_opacity = ui.opacity;
    if (ui.has_track_mode) M.track_mode    = ui.track_mode;

    signal(SIGINT, on_sig);
    signal(SIGTERM, on_sig);
    signal(SIGUSR1, on_recenter);   /* recenter head pose on demand (scriptable: pkill -USR1 mirage) */
    signal(SIGUSR2, on_calib);      /* re-run the calibration wizard (pkill -USR2 mirage) */
    signal(SIGHUP,  on_smooth);     /* A/B the pose smoothing filter (perf diag) */

    M.display = wl_display_connect(NULL);
    if (!M.display) { std::print(stderr, "mirage: cannot connect to wayland\n"); return 1; }
    M.registry = wl_display_get_registry(M.display);
    wl_registry_add_listener(M.registry, &REGISTRY_LISTENER, NULL);
    wl_display_roundtrip(M.display);   /* globals */
    wl_display_roundtrip(M.display);   /* output name/desc/mode events */

    if (!M.compositor || !g_wm_base || !M.capture_src_mgr || !M.copy_capture_mgr || !M.dmabuf) {
        std::print(stderr, "mirage: missing required wayland globals "
                "(compositor={} xdg_wm_base={} capture_src={} copy_capture={} dmabuf={})\n",
                (const void*)M.compositor, (const void*)g_wm_base,
                (const void*)M.capture_src_mgr, (const void*)M.copy_capture_mgr, (const void*)M.dmabuf);
        return 1;
    }
    if (classify_outputs() != 0) return 1;

    /* Fullscreen xdg-shell window on the glasses output. Hyprland page-flips this
     * straight to the panel (direct scanout) once it's opaque + fullscreen. Asking
     * for fullscreen up front is the deterministic path - far more reliable than a
     * `fullscreen` windowrule or a post-launch dispatch, which race the window's
     * first map and can drop scanout into an unpresented surface free-running at
     * hundreds of fps. M.glasses_out targets the glasses (NULL = compositor picks). */
    M.surface = wl_compositor_create_surface(M.compositor);
    struct xdg_surface *xs = xdg_wm_base_get_xdg_surface(g_wm_base, M.surface);
    xdg_surface_add_listener(xs, &XSURF_LISTENER, NULL);
    g_toplevel = xdg_surface_get_toplevel(xs);
    xdg_toplevel_add_listener(g_toplevel, &XTOP_LISTENER, NULL);
    xdg_toplevel_set_title(g_toplevel, "mirage");
    xdg_toplevel_set_app_id(g_toplevel, "mirage");
    xdg_toplevel_set_fullscreen(g_toplevel, M.glasses_out);
    wl_surface_commit(M.surface);

    while (!M.configured && wl_display_dispatch(M.display) >= 0) { /* wait config */ }
    if (g_win_cfg_w > 0) { M.glasses_w = g_win_cfg_w; M.glasses_h = g_win_cfg_h; }
    if (M.glasses_w <= 0 || M.glasses_h <= 0) {
        std::print(stderr, "mirage: bad glasses size {}x{}\n", M.glasses_w, M.glasses_h);
        return 1;
    }

    if (auto r = render_init(&M);  !r) { std::print(stderr, "mirage: render_init: {}\n",  r.error()); return 1; }
    if (auto r = capture_init(&M); !r) { std::print(stderr, "mirage: capture_init: {}\n", r.error()); return 1; }
    if (M.seat) M.pointer = wl_seat_get_pointer(M.seat);
    grab_init(&M);

    /* Head pose over OpenTrack UDP (the RayNeo bridge streams to 127.0.0.1:4242). */
    pose_config pc = { .backend = POSE_OPENTRACK_UDP, .udp_port = M.cfg.pose_port,
                       .smoothing = M.cfg.pose_smoothing,
                       .use_oneeuro = M.cfg.pose_oneeuro,
                       .oe_mincutoff = M.cfg.pose_mincutoff,
                       .oe_beta = M.cfg.pose_beta, .oe_dcutoff = 1.0f,
                       .drift_comp_tau = M.cfg.pose_drift_tau,
                       .facecam_enable = M.cfg.facecam_enable,
                       .facecam_socket = "/tmp/mirage-facecam.sock",
                       .facecam_smooth = M.cfg.facecam_smooth,
                       .facecam_fusion = M.cfg.facecam_fusion };
    if (pose_start(&pc) != 0)
        std::print(stderr, "mirage: pose backend failed to start (rendering without tracking)\n");

    std::print(stderr, "mirage: running. Ctrl-C to quit.\n");
    M.running = true;
    struct timespec fps_t0; clock_gettime(CLOCK_MONOTONIC, &fps_t0);
    struct timespec frame_prev = fps_t0;   /* for per-frame interval timing */
    long fps_frames = 0;
    double worst_ms = 0.0;                  /* slowest frame in the window   */
    struct timespec cap_t = fps_t0;
    /* Capture content at 60Hz to match the scanout, so the desktop + cursor track
     * smoothly (30Hz felt laggy). ext-image-copy-capture only re-copies DAMAGED
     * regions, so a mostly-static desktop is cheap even at 60 - the old full-output
     * wlr-screencopy blit that forced 30Hz is gone. */
    const double CAP_PERIOD = 1.0 / 60.0;
    while (M.running && !g_stop) {
        /* drain pending events first (xdg ping/pong, resizes) so the
         * compositor never flags us as unresponsive */
        wl_display_dispatch_pending(M.display);

        /* follow compositor-driven resizes (e.g. the fullscreen configure) */
        if (g_win_cfg_w > 0 &&
            (g_win_cfg_w != M.glasses_w || g_win_cfg_h != M.glasses_h)) {
            M.glasses_w = g_win_cfg_w; M.glasses_h = g_win_cfg_h;
            wl_egl_window_resize(M.egl_window, M.glasses_w, M.glasses_h, 0, 0);
        }
        /* Decoupled capture: fire a screencopy for any idle screen, then pump
         * the Wayland socket NON-blocking and consume whatever has landed. We
         * never block waiting for content here — the glasses vsync (eglSwapBuffers
         * below) is the only pacer, so the head-pose camera redraws every panel
         * refresh (120 Hz) while desktop content refreshes opportunistically.
         * Blocking on capture here used to stack a second 120 Hz wait in series
         * with vsync, halving the rate to 60. */
        {   /* fire a fresh capture batch only when the ~60 Hz period elapses */
            struct timespec ct; clock_gettime(CLOCK_MONOTONIC, &ct);
            double since = (ct.tv_sec - cap_t.tv_sec) + (ct.tv_nsec - cap_t.tv_nsec) * 1e-9;
            if (since >= CAP_PERIOD) { capture_begin_frame(&M); cap_t = ct; }
        }
        while (wl_display_prepare_read(M.display) != 0)
            wl_display_dispatch_pending(M.display);
        wl_display_flush(M.display);
        struct pollfd pfd = { .fd = wl_display_get_fd(M.display), .events = POLLIN };
        if (poll(&pfd, 1, 0) > 0 && (pfd.revents & POLLIN))
            wl_display_read_events(M.display);
        else
            wl_display_cancel_read(M.display);
        wl_display_dispatch_pending(M.display);
        capture_poll(&M);   /* bind any ready frames into textures; never blocks */

        /* A protocol error tears down the connection but leaves eglSwapBuffers
         * spinning unthrottled into an unmapped surface (invisible, 1000s of
         * fps). Catch it loudly instead of pretending to render. */
        int derr = wl_display_get_error(M.display);
        if (derr) {
            std::print(stderr, "mirage: wayland connection error ({}) - surface is "
                    "no longer presented; exiting.\n", derr);
            M.running = false;
            break;
        }

        if (g_smooth_toggle) {
            bool on = pose_toggle_smoothing(); g_smooth_toggle = 0;
            std::print(stderr, "mirage: pose smoothing {}\n",
                    on ? "ON (filtered)" : "OFF (raw passthrough)");
        }
        grab_pump(&M);   /* drain trackpad motion/buttons while captured */

        /* Centring is owned by the calibration overlay (calib.c): it waits for you
         * to look forward and hold still, then recenters - the guided version of
         * the old "recenter on first sample". SIGUSR2 re-opens the full wizard. */
        if (g_calib) { calib_start_wizard(&M); g_calib = 0; }
        if (g_recenter) { pose_recenter(); g_recenter = 0;
                          std::print(stderr, "mirage: recentered\n"); }
        quat head = pose_has_signal()
                  ? pose_predicted(M.cfg.pose_predict_ms * 0.001f) : q_identity();
        render_frame(&M, head);
        wl_display_flush(M.display);

        /* once-a-second perf readout. fps = throughput (gated by capture +
         * glasses vsync); 'worst' is the slowest single frame in the window —
         * an average of 120 can still hide a recurring 16 ms hitch. We also
         * print head-pose freshness: 'pose Hz' is the inbound sample rate
         * (if the source is 60 Hz you render 120 fps but only half carry a new
         * head reading), 'age' is staleness of the newest sample, and the
         * SMOOTHING flag shows whether the filter (a latency source) is on. */
        fps_frames++;
        {
            struct timespec now; clock_gettime(CLOCK_MONOTONIC, &now);
            double fdt = (now.tv_sec - frame_prev.tv_sec) + (now.tv_nsec - frame_prev.tv_nsec) * 1e-9;
            frame_prev = now;
            if (fdt > worst_ms) worst_ms = fdt;
            static long hitch_count = 0; static double hitch_ms_sum = 0;
            if (fdt > 0.022) { hitch_count++; hitch_ms_sum += fdt * 1000.0; }   /* >22ms = missed a 60Hz vblank */
            double dt = (now.tv_sec - fps_t0.tv_sec) + (now.tv_nsec - fps_t0.tv_nsec) * 1e-9;
            if (dt >= 1.0) {
                M.fps = (float)(fps_frames / dt);   /* publish for the in-scene HUD */
                double phz = pose_take_sample_count() / dt;
                uint32_t age = pose_age_ms();
                std::print(stderr, "mirage: {:.1f} fps | worst {:.1f} ms | hitches {} | pose {:.0f} Hz, "
                        "age {} ms{}\n", fps_frames / dt, worst_ms * 1000.0, hitch_count, phz,
                        age, pose_smoothing_enabled() ? "" : " | SMOOTHING OFF");
                hitch_count = 0; hitch_ms_sum = 0; (void)hitch_ms_sum;
                fps_t0 = now; fps_frames = 0; worst_ms = 0.0;
            }
        }
    }

    std::print(stderr, "\nmirage: shutting down\n");
    pose_stop();
    capture_finish(&M);
    grab_destroy(&M);
    render_finish(&M);
    wl_display_disconnect(M.display);
    return 0;
}
