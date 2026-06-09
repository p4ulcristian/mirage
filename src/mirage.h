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

#include <wayland-client.h>
#include <EGL/egl.h>
#include <EGL/eglext.h>
#include <GLES2/gl2.h>
#include <GLES2/gl2ext.h>

#include "math3d.h"

#define MIRAGE_MAX_SCREENS 8

/* screen surface: curved cylinder strips, or flat quads in the same column-yaw
 * / straight-up-row layout (no bend, but same orientation). */
enum mirage_geometry { GEOM_CYLINDER = 0, GEOM_FLAT = 1 };

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
    GLuint                tex;

    /* per-frame capture state (ext-image-copy-capture-v1) */
    struct ext_image_copy_capture_session_v1 *session;  /* persistent per output */
    struct ext_image_copy_capture_frame_v1   *frame;    /* one capture in flight */
    int      frame_state;       /* 0 idle, 1 pending, 2 ready, 3 failed  */
    bool     session_ready;     /* buffer constraints (size+format) received */
    bool     have_tex;          /* texture has valid contents            */
    bool     y_invert;          /* compositor reported flipped Y         */

    /* curved-screen mesh (built once, drawn each frame in 3D mode) */
    GLuint   mesh_vbo;
    int      mesh_verts;

    /* slab body: 5 solid faces (top/bottom/left/right/back) extruded behind the
     * front face to give each screen real thickness (built once, 3D mode only). */
    GLuint   slab_vbo;
    int      slab_verts;
} screen_t;

typedef struct {
    /* virtual-display geometry */
    int   screen_count;         /* how many VIRT outputs to expect       */
    float screen_distance_m;    /* metres from eye to screen centre      */
    float screen_width_m;       /* physical width of each virtual screen */
    float arc_spacing_deg;      /* extra gap between screens (0 = touch)  */
    float screen_arc_deg;       /* angular width each curved screen spans */
    int   screen_cols;          /* screens per row (rows stack vertically) */
    float slab_depth_m;         /* screen thickness; 0 = flat panels (no slab) */
    bool  gaze_cursor;          /* Cmd-held: cursor follows head gaze (grab.c)  */

    /* glasses optics */
    float fov_deg;              /* vertical field of view of the glasses */

    /* pose */
    int   pose_port;            /* OpenTrack UDP port                    */
    float pose_smoothing;       /* legacy fixed nlerp factor, 0..1       */
    bool  pose_oneeuro;         /* use the One-Euro adaptive filter      */
    float pose_mincutoff;       /* One-Euro cutoff at rest (Hz)          */
    float pose_beta;            /* One-Euro speed coupling               */
    float yaw_gain;             /* head-yaw amplification (1 = 1:1)      */
    float pitch_gain;           /* head-pitch amplification (1 = 1:1)    */
    float roll_damp;            /* keep this fraction of head roll (0=horizon lock) */
    float read_deadband_deg;    /* freeze camera tremor below this angle (0 = off) */
    float sharpen;              /* contrast-adaptive sharpen strength (0 = off)    */
    int   geometry;             /* GEOM_CYLINDER / GEOM_FLAT                        */

    /* HDRI environment dome: an equirectangular image drawn as an infinite,
     * world-fixed backdrop you look around. On the additive optics it only adds
     * light (dark = transparent), so a dark/starry HDRI reads best. */
    bool  hdri_on;              /* draw the environment dome                       */
    char  hdri_path[256];       /* path to a flat Radiance .hdr (see hdri/exr2hdr.py) */
    float hdri_exposure;        /* linear gain before tonemap (boosts faint stars) */
    float hdri_intensity;       /* final additive strength; lower = more see-through */

    /* identification */
    char  glasses_match[64];    /* substring of glasses output desc/name */

    float bg[3];                /* background clear colour               */
} mirage_config;

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
    char     glasses_name[32];
    char     glasses_desc[128];
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
    struct { struct wl_output *wl; char name[32]; char desc[128];
             int32_t w, h; bool done; } pending[16];
    int n_pending;

    mirage_config cfg;
    float zoom;       /* view zoom (Super+scroll); 1.0 = default, clamped       */
    int   view_focus; /* Cmd+H-scroll focus: display the view pans to (ring idx) */
    float pan_yaw;    /* current eased pan angles (rad) toward view_focus screen */
    float pan_pitch;
    float fps;        /* last measured throughput, published by the main loop (HUD) */
    /* perf profiling (dormant; set profile=true to enable): split a frame into pure
     * GPU draw cost (glFinish before swap) vs present wait (eglSwapBuffers). gpu high
     * = render/texture-sampling bound; swap high = compositor/present bound. */
    bool   profile;
    double prof_gpu_ms, prof_swap_ms;
    /* gaze cursor (grab.c): the final camera yaw/pitch (rad) render_frame looked
     * along this frame, so the cursor can warp to where the eye points while Cmd
     * is held. gaze_have gates it off until the first head-tracked frame. */
    float gaze_yaw, gaze_pitch;
    bool  gaze_have;
    bool running;
};

#define MIRAGE_ZOOM_MIN 0.5f
#define MIRAGE_ZOOM_MAX 4.0f

/* config.c */
void mirage_config_defaults(mirage_config *c);

/* render.c - EGL/GLES scene rendering */
bool render_init(struct mirage *m);
void render_frame(struct mirage *m, quat head);   /* 3D head-tracked arc      */
void render_finish(struct mirage *m);

/* capture.c - wlr-screencopy into GL textures */
bool capture_init(struct mirage *m);     /* opens drm/gbm                */
void capture_begin_frame(struct mirage *m); /* kick a capture per screen */
bool capture_poll(struct mirage *m);     /* true when all frames settled */
void capture_finish(struct mirage *m);

/* layout.c - where each screen sits in 3D */
mat4 layout_model_matrix(const struct mirage *m, int screen_index);
/* Camera yaw/pitch (rad) that centres display `i` in view - used to pan the
 * wall to a focused screen. Mirrors the yaw + lift placement in the model. */
void layout_focus_angles(const struct mirage *m, int i, float *yaw, float *pitch);

/* grab.c - Super+G input capture: lock the real pointer, read raw motion, and
 * drive a cursor across the virtual screens as one continuous strip. */
bool grab_init(struct mirage *m);
void grab_toggle(struct mirage *m);   /* enter/leave capture mode (Super+G)    */
void grab_pump(struct mirage *m);     /* drain trackpad events (every frame)   */
bool grab_active(struct mirage *m);
int  grab_cursor_screen(struct mirage *m);  /* focused screen idx, -1 if none   */
void grab_destroy(struct mirage *m);

#endif /* MIRAGE_H */
