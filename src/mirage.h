/* mirage.h - shared state for the head-tracked AR compositor.
 *
 * mirage runs as a normal Wayland client on Hyprland:
 *   - it captures the N virtual outputs (VIRT1..) via wlr-screencopy into GL
 *     textures (zero-copy through dmabuf/EGLImage),
 *   - it owns one fullscreen xdg-shell surface on the glasses output, which
 *     Hyprland page-flips straight to the panel (direct scanout),
 *   - each frame it draws the captured screens as quads floating in a 3D arc,
 *     viewed through a camera rotated by the live head pose,
 *   - Super+G confines input to the virtual workspace (see grab.c).
 */
#ifndef MIRAGE_H
#define MIRAGE_H

#include <stdbool.h>
#include <stdint.h>

#include <expected>
#include <string>

#include <wayland-client.h>
#include <EGL/egl.h>
#include <EGL/eglext.h>
#include <GLES2/gl2.h>
#include <GLES2/gl2ext.h>

#include "math3d.h"

#define MIRAGE_MAX_SCREENS 10

/* screen surface: curved cylinder strips, or flat quads in the same column-yaw
 * / straight-up-row layout (no bend, but same orientation). */
enum mirage_geometry { GEOM_CYLINDER = 0, GEOM_FLAT = 1 };

/* what sits behind the windows; cycled by the HUD background-mode button. */
enum mirage_bg_mode { BG_BLACK = 0, BG_HDRI = 1, BG_PASSTHROUGH = 2, BG_MODE_COUNT = 3 };

struct mirage; /* fwd */

/* One captured virtual display, ready to texture onto a quad. */
typedef struct {
    struct wl_output *wl;
    char     name[32];
    int32_t  width, height;     /* pixel size of the source output      */
    int      index;             /* 0-based slot among virtual screens    */

    /* capture target (allocated lazily once the format is known) */
    struct gbm_bo        *bo;
    int                   dmabuf_fd;
    uint32_t              drm_format;
    uint64_t              modifier;
    uint32_t              stride;
    struct wl_buffer     *buffer;
    EGLImageKHR           image;
    GLuint                tex;        /* dmabuf EGLImage, level 0 only (no mip chain) */
    GLuint                mip_tex;    /* mirror of tex with a full mip chain (cfg.mipmap) */
    bool                  tex_dirty;  /* a new frame landed; rebuild the mip chain next draw */

    /* per-frame capture state (ext-image-copy-capture-v1) */
    struct ext_image_copy_capture_session_v1 *session;  /* persistent per output */
    struct ext_image_copy_capture_frame_v1   *frame;    /* one capture in flight */
    int      frame_state;       /* 0 idle, 1 pending, 2 ready, 3 failed  */
    bool     session_ready;     /* buffer constraints (size+format) received */
    bool     have_tex;          /* texture has valid contents            */
    bool     y_invert;          /* compositor reported flipped Y         */

    float    arc_deg;           /* angular width this screen spans (per-screen)  */

    /* curved-screen mesh (built once, drawn each frame in 3D mode) */
    GLuint   mesh_vbo;
    int      mesh_verts;
} screen_t;

typedef struct {
    /* virtual-display geometry */
    int   screen_count;         /* how many VIRT outputs to expect       */
    float screen_distance_m;    /* metres from eye to screen centre      */
    float screen_width_m;       /* physical width of each virtual screen */
    float arc_spacing_deg;      /* extra gap between screens (0 = touch)  */
    float screen_arc_deg;       /* default angular width a screen spans   */
    float screen_arc[MIRAGE_MAX_SCREENS]; /* per-screen arc override (0 = use default) */
    int   screen_cols;          /* legacy uniform grid: screens per row (rows stack vertically) */
    bool  explicit_layout;      /* true: place by screen_col[] + per-column vertical centring,
                                 * so columns can hold different screen counts (uneven grid) */
    int   screen_col[MIRAGE_MAX_SCREENS]; /* yaw-column index per screen (left->right), explicit_layout */
    int   center_col;           /* explicit_layout: column to anchor dead-ahead (yaw 0);
                                 * <0 or out of range = centre the whole span (default) */
    /* free placement: a screen with a finite yaw is pinned to this pose and skips
     * the column/grid logic entirely (NAN = auto, derive from the column). lift is
     * the metres above eye level. Set by named layouts (layouts.c). */
    float screen_yaw_deg[MIRAGE_MAX_SCREENS];  /* NAN = auto (column) */
    float screen_lift_m [MIRAGE_MAX_SCREENS];  /* NAN = auto (row)    */

    /* glasses optics */
    float fov_deg;              /* vertical field of view of the glasses */

    /* pose */
    int   pose_port;            /* OpenTrack UDP port                    */
    float pose_smoothing;       /* legacy fixed nlerp factor, 0..1       */
    bool  pose_oneeuro;         /* use the One-Euro adaptive filter      */
    float pose_mincutoff;       /* One-Euro cutoff at rest (Hz)          */
    float pose_beta;            /* One-Euro speed coupling               */
    float pose_drift_tau;       /* heading-drift cancel time const (s); 0=off */
    float pose_predict_ms;      /* forward-prediction horizon (ms); cancels motion-to-photon lag, 0=off */
    float yaw_gain;             /* head-yaw amplification (1 = 1:1)      */
    float pitch_gain;           /* head-pitch amplification (1 = 1:1)    */
    float roll_damp;            /* keep this fraction of head roll (0=horizon lock) */
    float read_deadband_deg;    /* freeze camera tremor below this angle (0 = off) */
    float neck_fwd_m;           /* parallax: eye distance ahead of the neck pivot (synthesised
                                 * from rotation; the near windows shift against the far dome; 0 = off) */
    float neck_up_m;            /* parallax: eye height above the neck pivot */
    float sharpen;              /* contrast-adaptive sharpen strength (0 = off)    */
    int   msaa_samples;         /* MSAA sample count for the window surface (0/1 = off); init-time only */
    bool  mipmap;               /* trilinear minification: mirror each screen into a mipmapped texture */
    int   geometry;             /* GEOM_CYLINDER / GEOM_FLAT                        */

    /* HDRI environment dome: an equirectangular image drawn as an infinite,
     * world-fixed backdrop you look around. On the additive optics it only adds
     * light (dark = transparent), so a dark/starry HDRI reads best. */
    bool  hdri_on;              /* draw the environment dome                       */
    char  hdri_path[256];       /* path to a flat Radiance .hdr (see hdri/exr2hdr.py) */
    float hdri_exposure;        /* linear gain (boosts faint stars)                */
    float hdri_intensity;       /* final additive strength; lower = more see-through */
    float hdri_black;           /* black point: linear floor below this -> 0 (true black) */
    float hdri_saturation;      /* chroma boost around luma (1 = unchanged)        */

    /* identification */
    char  glasses_match[64];    /* substring of glasses output desc/name */

    float bg[3];                /* background clear colour               */
} mirage_config;

/* Named layouts loaded from layouts.conf: a set of full mirage_config snapshots
 * the user toggles between at runtime (Alt+1/2/3). See layouts.c. */
#define MIRAGE_MAX_LAYOUTS 8
typedef struct {
    char          name[32];
    mirage_config cfg;
} mirage_layout;
typedef struct {
    mirage_layout l[MIRAGE_MAX_LAYOUTS];
    int           n;        /* layouts parsed (0 = none; hardcoded defaults used) */
    int           active;   /* index currently applied to m->cfg                  */
} mirage_layouts;

/* Selectable HDRI environments (the dome behind the wall): the starfield plus calm
 * nature scenes, switched at runtime from the HUD. Each carries its own dome params
 * because the optics are ADDITIVE (black = transparent): the starfield's hard black
 * point / high exposure crush a nature scene, so every scene is tuned independently
 * and curated dark/dusk so it stays mostly see-through. on=false = dome hidden. */
#define MIRAGE_MAX_ENVS 8
typedef struct {
    char  name[16];          /* HUD caption ("Space", "Forest", ...)   */
    char  hdri_path[256];    /* flat Radiance .hdr (hdri/exr2hdr.py)    */
    float exposure, intensity, black, saturation;  /* dome shader params */
    bool  on;                /* false = "Off" tile: hide the dome       */
} mirage_env;
extern const mirage_env MIRAGE_ENVS[];
extern const int         MIRAGE_ENV_COUNT;

/* Apply environment idx: copies its dome params into m->cfg.hdri_* and flags a
 * texture reload (render does the GL work next frame). Mirrors layouts_switch. */
void env_switch(struct mirage *m, int idx);

/* In-glasses guided calibration overlay (calib.cpp): a head-locked panel that
 * tells you what to do. Two cadences from ONE machine: the full wizard (first run
 * - center, then tracking check, then FOV, then save) and the quick centre (every
 * launch - just "look forward, hold still" -> recenter). */
typedef enum {
    CALIB_OFF = 0,
    CALIB_CENTER,   /* hold still looking forward -> recenter            */
    CALIB_TRACK,    /* look around, screens stay put -> click to go on   */
    CALIB_FOV,      /* scroll to resize the screens -> click to save     */
    CALIB_DONE,     /* brief "Ready" flash, then off                     */
} calib_step;

typedef struct {
    calib_step step;
    bool   wizard;     /* true = full first-run wizard; false = quick centre only */
    float  still_t;    /* seconds the head has held still (CENTER step)           */
    float  done_t;     /* DONE-flash countdown (s)                               */
    float  wait_t;     /* seconds spent waiting for a pose signal (CENTER step)  */
    quat   prev;       /* previous head sample, for the stillness speed          */
    bool   have_prev;
} calib_state;

struct mirage {
    /* wayland globals */
    struct wl_display    *display;
    struct wl_registry   *registry;
    struct wl_compositor *compositor;
    /* ext-image-copy-capture-v1: the modern, damage-aware capture path */
    struct ext_output_image_capture_source_manager_v1 *capture_src_mgr;
    struct ext_image_copy_capture_manager_v1           *copy_capture_mgr;
    struct zwp_linux_dmabuf_v1        *dmabuf;
    struct wl_shm        *shm;

    /* input grab (grab.c): seat + the three managers that let us lock the real
     * pointer, read its raw motion, and inject a cursor onto the arc. */
    struct wl_seat       *seat;
    struct wl_pointer    *pointer;
    struct zwp_pointer_constraints_v1        *pointer_constraints;
    struct zwp_relative_pointer_manager_v1   *rel_pointer_mgr;
    struct zwlr_virtual_pointer_manager_v1   *vpointer_mgr;
    void   *grab;   /* opaque grab_state*, owned by grab.c */

    /* glasses output + render surface */
    struct wl_output *glasses_out;
    std::string glasses_name;
    std::string glasses_desc;
    int32_t  glasses_w, glasses_h;
    struct wl_surface             *surface;
    struct wl_egl_window          *egl_window;
    bool     configured;

    /* egl/gl */
    EGLDisplay edpy;
    EGLContext ectx;
    EGLSurface esurf;
    EGLConfig  ecfg;

    /* gbm for capture buffer allocation */
    int  drm_fd;
    struct gbm_device *gbm;

    /* virtual screens */
    screen_t screen[MIRAGE_MAX_SCREENS];
    int      n_screen;

    /* transient output discovery: outputs we've bound but not classified */
    struct { struct wl_output *wl; std::string name; std::string desc;
             int32_t w, h; bool done; } pending[16];
    int n_pending;

    mirage_config cfg;
    /* device/tracking/optics calibration (profile.cpp): the calibrated values,
     * stashed so they can be re-stamped onto cfg after a layout switch (layouts
     * snapshot the whole config and would otherwise wipe them). */
    mirage_config calib_cfg;
    bool  have_profile;
    calib_state calib;   /* in-glasses guided calibration overlay (calib.cpp) */
    float zoom;       /* view zoom (Super+scroll); 1.0 = default, clamped       */
    float fps;        /* last measured throughput, published by the main loop (HUD) */
    /* perf profiling (dormant; set profile=true to enable): split a frame into pure
     * GPU draw cost (glFinish before swap) vs present wait (eglSwapBuffers). gpu high
     * = render/texture-sampling bound; swap high = compositor/present bound. */
    bool   profile;
    double prof_gpu_ms, prof_swap_ms;
    /* head gaze (grab.c): the final camera yaw/pitch (rad) render_frame looked
     * along this frame, so shake-to-gaze ("find my cursor") can warp the cursor to
     * where the eye points. gaze_have gates it off until the first tracked frame. */
    float gaze_yaw, gaze_pitch;
    bool  gaze_have;
    /* 3D pointer (grab.c publishes the cursor's wall-space look direction each
     * update; render.c draws an arrow billboard on the cylinder at this dir, so the
     * pointer floats in the scene - visible over screens AND in the gaps). */
    float cursor_yaw, cursor_pitch;
    bool  cursor_have;
    bool  cursor_in_gap;   /* cursor is between screens: draw the arrow (no desktop
                            * cursor reaches the gaps); over a screen it stays hidden
                            * so it doesn't duplicate the painted desktop pointer. */
    float world_yaw;       /* horizontal rotation of the whole wall about the eye,
                            * driven by 3-finger horizontal swipe (grab.c). Added to every
                            * screen's placement, so render + cursor picking spin as
                            * one; the cursor's own direction stays in fixed space. */
    float world_lift;      /* vertical TRANSLATION of the eye along the cylinder axis (metres,
                            * +up), driven by 4-finger vertical swipe. We're inside a cylinder, so
                            * dollying the eye up/down feels right where swinging the wall did not.
                            * Added to eye_world.y in render; layout_pick offsets the cursor height
                            * by it so clicks stay aligned. */

    /* named layouts (layouts.c): the registry plus a dirty flag the render loop
     * watches so a runtime layout switch rebuilds the screen meshes. */
    mirage_layouts layouts;
    bool  layout_dirty;

    /* selectable HDRI environment (env_switch): the index shown now and a dirty flag
     * the render loop watches to reload the dome texture after a switch. */
    int   active_env;
    bool  env_dirty;
    float env_brightness;   /* HUD slider: multiplies the dome intensity (1.0 = as tuned) */

    /* HUD idle auto-collapse: monotonic ms (matches grab.c now_ms) of the last touch/
     * hover on the panel. After HUD_IDLE_MS with no interaction the panel collapses to
     * just the FPS strip; hovering it wakes it. 0 = uninitialised (starts awake). */
    uint32_t hud_active_ms;

    /* background mode (HUD cycle button): what sits behind the windows.
     *   BG_BLACK       - nothing (true black -> see-through on the additive optics)
     *   BG_HDRI        - the environment dome (default)
     *   BG_PASSTHROUGH - the world-facing camera, head-locked fullscreen
     * render.cpp owns the camera's lifecycle off this (lazy start / release). */
    int         bg_mode;        /* mirage_bg_mode */
    struct cam *cam;

    /* window/screen opacity (HUD transparency slider): screens alpha-blend over the
     * background at this alpha, so you can fade the wall to see the real world (or the
     * passthrough) through it. 1.0 = fully opaque (as before). */
    float       screen_opacity;
    bool running;
};

#define MIRAGE_ZOOM_MIN 0.5f
#define MIRAGE_ZOOM_MAX 4.0f

/* Init result: success, or a human-readable reason the step failed. main()
 * prints the reason and exits, so the message travels with the failure instead
 * of each call site printing its own. */
using mirage_status = std::expected<void, std::string>;

/* config.c */
void mirage_config_defaults(mirage_config *c);

/* profile.c - persisted device/tracking/optics calibration (profile.toml), loaded
 * over the compiled defaults and re-stamped after layout switches (calibration is
 * per-rig and global, not per-layout). Written by the mirage-cal pre-flight tool. */
std::string profile_default_path();                       /* $MIRAGE_PROFILE / XDG path */
int  profile_load(const char *path, mirage_config *c);    /* overlay present keys; count */
bool profile_save(const char *path, const mirage_config *c);
void profile_apply(mirage_config *dst, const mirage_config *src);  /* copy calib fields */

/* ui_state.toml - persisted HUD selections (geometry toggle, env brightness, active
 * environment, active layout) so the wall comes back as you left it. Separate from
 * the calibration profile: these are user-facing runtime choices, not per-rig tuning,
 * and they must NOT ride the profile_apply re-stamp (which would pin them per layout).
 * ui_state_save snapshots the live mirage; ui_state_load fills *s for main to apply. */
typedef struct {
    bool  has_geometry;  int   geometry;       /* GEOM_CYLINDER / GEOM_FLAT */
    bool  has_env;       int   env;            /* active_env index          */
    bool  has_brightness; float brightness;    /* env_brightness            */
    bool  has_layout;    char  layout[64];     /* active layout NAME        */
    bool  has_bg_mode;   int   bg_mode;        /* mirage_bg_mode            */
    bool  has_opacity;   float opacity;        /* screen_opacity            */
} mirage_ui_state;
std::string ui_state_default_path();                      /* $MIRAGE_UI_STATE / XDG path */
int  ui_state_load(const char *path, mirage_ui_state *s); /* present keys; count          */
bool ui_state_save(const char *path, const struct mirage *m);

/* calib.c - in-glasses guided calibration overlay. calib_init picks the cadence
 * (full wizard on first run, else quick centre); calib_update runs the state
 * machine each frame from the head pose; click/scroll drive the wizard steps;
 * render.c draws the head-locked panel from calib_text/calib_progress. */
void calib_init(struct mirage *m, bool had_profile);
void calib_start_wizard(struct mirage *m);   /* re-summon the full wizard (SIGUSR2) */
void calib_update(struct mirage *m, quat head);
bool calib_active(const struct mirage *m);
void calib_click(struct mirage *m);          /* trackpad click: advance a wizard step */
void calib_scroll(struct mirage *m, double v); /* FOV step: resize the screens        */
const char *calib_text(const struct mirage *m);   /* current instruction (head-locked) */
float calib_progress(const struct mirage *m);     /* 0..1 in CENTER, <0 = no bar        */

/* render.c - EGL/GLES scene rendering */
mirage_status render_init(struct mirage *m);
void render_frame(struct mirage *m, quat head);   /* 3D head-tracked arc      */
void render_finish(struct mirage *m);
/* (re)build every screen's mesh from the current cfg; call after a layout
 * switch changes arcs/geometry/distance. Safe to call only with a live GL ctx. */
void render_rebuild_meshes(struct mirage *m);

/* layouts.c - named layouts loaded from layouts.conf, toggled at runtime */
int  layouts_load(mirage_layouts *L, const char *path); /* parse file -> registry; returns count */
void layouts_switch(struct mirage *m, int idx);         /* apply layout idx, flag a mesh rebuild */

/* capture.c - wlr-screencopy into GL textures */
mirage_status capture_init(struct mirage *m);  /* opens drm/gbm           */
void capture_begin_frame(struct mirage *m); /* kick a capture per screen */
bool capture_poll(struct mirage *m);     /* true when all frames settled */
void capture_finish(struct mirage *m);

/* layout.c - where each screen sits in 3D */
mat4 layout_model_matrix(const struct mirage *m, int screen_index);
/* Camera yaw/pitch (rad) that centres display `i` in view - used to pan the
 * wall to a focused screen. Mirrors the yaw + lift placement in the model. */
void layout_focus_angles(const struct mirage *m, int i, float *yaw, float *pitch);
/* Single source of truth for placement: screen i's column-centre yaw (rad), the
 * vertical lift of its centre (m), and its arc width (deg). Shared by render +
 * the cursor grid so both agree on an uneven, multi-column wall. */
void layout_place(const struct mirage *m, int i, float *yaw, float *lift, float *arc_deg);
/* Pointer pick (input twin of layout_model_matrix): which screen does a wall-space
 * look direction (*yaw,*pitch, rad) land on, and where within it (u,v in 0..1; u=0
 * left edge, v=0 top edge, in source-pixel orientation)? Returns the screen index.
 * On a direct hit returns that screen with u,v inside it and *inside = true. In a
 * gap it returns the NEAREST screen's edge pixel as the desktop-pointer injection
 * target with *inside = false - WITHOUT moving the cursor, so the free cursor (and
 * the 3D arrow drawn at its real dir) can roam the gaps. *inside (nullable) lets
 * render draw the arrow only in the gaps, where the desktop cursor can't reach.
 * Returns -1 only when there are no screens. Picks against the SAME placement
 * render draws, so cursor and picture can never disagree about where a screen is. */
int  layout_pick(const struct mirage *m, float yaw, float pitch,
                 float *u, float *v, bool *inside);
int  layout_num_cols(const struct mirage *m);          /* distinct yaw columns       */
int  layout_screen_col(const struct mirage *m, int i); /* column index of screen i   */

/* Angular + vertical extent of the column-placed wall (free satellites excluded):
 * the yaw of its centre (rad), its total angular width (rad), and the height of
 * its top edge (m). Lets the clock banner hang above the wall on the same curve.
 * Sets *arc_total <= 0 when there are no column-placed screens. */
void layout_wall_extent(const struct mirage *m, float *yaw_c, float *arc_total, float *top);

/* Sensitivity slider (in-view widget): one draggable handle sets yaw_gain and
 * pitch_gain together (look sensitivity), a DEFAULT button resets them. render.c
 * draws it under the centre screen alongside the FPS plaque; grab.c hit-tests
 * a cursor against it and drives the value. The geometry is computed ONCE here so
 * the picture and the click can't disagree (the same trick layout_pick plays for
 * the screens). Everything is in the centre screen's LOCAL frame (metres, on the
 * z = -d plane), so render multiplies by layout_model_matrix(ci) and grab maps a
 * cursor (yaw,pitch) into it via x = d*tan(yaw_c - cyaw), y = d*tan(cpitch) - lift_c. */
typedef struct {
    int   ci;                 /* centre screen the panel hangs under        */
    float d;                  /* eye->wall distance (m)                     */
    float yaw_c, lift_c;      /* centre screen placement (rad, m): cursor->local */
    float gain;               /* current linked yaw/pitch gain              */
    float row_y;              /* local y of the slider row centre (m)       */
    float track_x0, track_x1; /* track ends (m); handle slides between them */
    float track_h;            /* track thickness (m)                        */
    float handle_x;           /* handle centre x for the current gain (m)   */
    float handle_w, handle_h;
    float def_x0, def_x1, def_y0, def_y1;  /* DEFAULT button rect (m)       */
    /* layout switcher: a clickable box per loaded named layout, in one row below
     * the slider. Buttons share a vertical extent; each has its own centre x. */
    int   n_layout;                        /* buttons to draw (= layouts.n) */
    int   active_layout;                   /* index applied now (highlight)  */
    float lay_y0, lay_y1;                  /* button row vertical extent (m) */
    float lay_w;                           /* each button's width (m)        */
    float lay_cx[MIRAGE_MAX_LAYOUTS];      /* each button's centre x (m)     */
    /* environment switcher: a second row below the layout row, same box style */
    int   n_env;                           /* buttons to draw (= MIRAGE_ENV_COUNT) */
    int   active_env;                      /* index applied now (highlight)       */
    float env_y0, env_y1;                  /* env row vertical extent (m)         */
    float env_w;                           /* each button's width (m)             */
    float env_cx[MIRAGE_MAX_ENVS];         /* each button's centre x (m)          */
    /* environment brightness slider: one more row below the env tiles */
    float bri_row_y;                       /* slider y centre (m)                 */
    float bri_x0, bri_x1;                  /* track ends (m)                      */
    float bri_track_h;                     /* track thickness (m)                 */
    float bri_handle_x;                    /* handle centre for current brightness */
    float bri_handle_w, bri_handle_h;      /* handle size (m)                     */
    /* window/screen transparency slider: one row below the brightness slider */
    float tr_row_y;                        /* slider y centre (m)                 */
    float tr_x0, tr_x1;                    /* track ends (m)                      */
    float tr_track_h;                      /* track thickness (m)                 */
    float tr_handle_x;                     /* handle centre for current opacity   */
    float tr_handle_w, tr_handle_h;        /* handle size (m)                     */
    /* flat/curved geometry toggle: one button row below the transparency slider */
    float geo_x0, geo_x1;                  /* button horizontal extent (m)        */
    float geo_y0, geo_y1;                  /* button vertical extent (m)          */
    int   geo_flat;                        /* 1 = currently flat (else curved)    */
    /* background-mode button (black / hdri / passthrough): below the flat/curved toggle */
    float pt_x0, pt_x1;                    /* button horizontal extent (m)        */
    float pt_y0, pt_y1;                    /* button vertical extent (m)          */
    int   pt_mode;                         /* current mirage_bg_mode              */
    /* head-tilt (roll) toggle: below the background-mode button */
    float tl_x0, tl_x1;                    /* button horizontal extent (m)        */
    float tl_y0, tl_y1;                    /* button vertical extent (m)          */
    int   tl_on;                           /* 1 = head roll tracked, 0 = horizon-locked */
    /* idle auto-collapse: the whole laid-out panel's bounds (m) so grab can tell a
     * hover/touch is on the HUD (wakes it); collapsed = only the FPS strip is shown. */
    float panel_x0, panel_x1, panel_y0, panel_y1;
    int   collapsed;                       /* 1 = idle-collapsed (FPS only, no widgets) */
} sens_panel;

#define BRI_MIN 0.20f
#define BRI_MAX 3.00f
#define BRI_DEF 1.00f

#define OPAC_MIN 0.15f   /* don't let windows vanish entirely */
#define OPAC_MAX 1.00f
#define OPAC_DEF 1.00f

#define SENS_GAIN_MIN 1.0f
#define SENS_GAIN_MAX 16.0f
#define SENS_GAIN_DEF 1.0f    /* matches config.cpp yaw_gain/pitch_gain default (1:1, natural) */

/* Fill *out with the slider geometry for the current frame; false if there's no
 * screen to hang it under. Shared by render (draw) and grab (hit-test). */
bool sens_panel_compute(const struct mirage *m, sens_panel *out);

/* grab.c - Super+G input capture: lock the real pointer, read raw motion, and
 * drive a cursor across the virtual screens as one continuous strip. */
bool grab_init(struct mirage *m);
void grab_toggle(struct mirage *m);   /* enter/leave capture mode (Super+G)    */
void grab_pump(struct mirage *m);     /* drain trackpad events (every frame)   */
bool grab_active(struct mirage *m);
int  grab_cursor_screen(struct mirage *m);  /* focused screen idx, -1 if none   */
void grab_destroy(struct mirage *m);

#endif /* MIRAGE_H */
