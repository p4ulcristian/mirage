/* mirage.h - shared state for the head-tracked AR compositor.
 *
 * mirage runs as a normal Wayland client on Hyprland:
 *   - it captures the N virtual outputs (VIRT1..) via wlr-screencopy into GL
 *     textures (zero-copy through dmabuf/EGLImage),
 *   - it owns one fullscreen layer-shell surface on the glasses output,
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

    /* per-frame capture state */
    struct zwlr_screencopy_frame_v1 *frame;
    int      frame_state;       /* 0 idle, 1 pending, 2 ready, 3 failed  */
    bool     have_tex;          /* texture has valid contents            */
    bool     y_invert;          /* compositor reported flipped Y         */

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
    float screen_arc_deg;       /* angular width each curved screen spans */
    int   screen_cols;          /* screens per row (rows stack vertically) */

    /* glasses optics */
    float fov_deg;              /* vertical field of view of the glasses */

    /* pose */
    int   pose_port;            /* OpenTrack UDP port                    */
    float pose_smoothing;       /* nlerp factor, 0..1                    */

    /* identification */
    char  glasses_match[64];    /* substring of glasses output desc/name */

    float bg[3];                /* background clear colour               */
} mirage_config;

struct mirage {
    /* wayland globals */
    struct wl_display    *display;
    struct wl_registry   *registry;
    struct wl_compositor *compositor;
    struct zwlr_layer_shell_v1        *layer_shell;
    struct zwlr_screencopy_manager_v1 *screencopy;
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
    struct zwlr_layer_surface_v1  *layer_surface;
    struct wl_egl_window          *egl_window;
    bool     configured;

    /* optional laptop preview: a second toplevel window mirroring the flat view
     * of the virtual screens, so they stay visible/usable without the glasses. */
    struct wl_surface   *pv_surface;
    struct xdg_surface  *pv_xsurf;
    struct xdg_toplevel *pv_xtop;
    struct wl_egl_window *pv_egl_window;
    EGLSurface           pv_esurf;
    int32_t  pv_w, pv_h;            /* current backing size                 */
    int32_t  pv_cfg_w, pv_cfg_h;   /* size requested by the last configure */
    bool     pv_enabled;

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
    bool running;
};

#define MIRAGE_ZOOM_MIN 0.5f
#define MIRAGE_ZOOM_MAX 4.0f

/* config.c */
void mirage_config_defaults(mirage_config *c);

/* render.c - EGL/GLES scene rendering */
bool render_init(struct mirage *m);
void render_frame(struct mirage *m, quat head);   /* 3D head-tracked arc      */
void render_frame_flat(struct mirage *m);         /* flat capture-only, no pose */
bool render_preview_init(struct mirage *m);       /* laptop preview EGL surface */
void render_preview(struct mirage *m);            /* draw the preview window    */
void render_finish(struct mirage *m);

/* capture.c - wlr-screencopy into GL textures */
bool capture_init(struct mirage *m);     /* opens drm/gbm                */
void capture_begin_frame(struct mirage *m); /* kick a capture per screen */
bool capture_poll(struct mirage *m);     /* true when all frames settled */
void capture_finish(struct mirage *m);

/* layout.c - where each screen sits in 3D */
mat4 layout_model_matrix(const struct mirage *m, int screen_index);

/* grab.c - Super+G input capture: lock the real pointer, read raw motion, and
 * drive a cursor across the virtual screens as one continuous strip. */
bool grab_init(struct mirage *m);
void grab_toggle(struct mirage *m);   /* enter/leave capture mode (Super+G)    */
void grab_pump(struct mirage *m);     /* drain trackpad events (every frame)   */
bool grab_active(struct mirage *m);
void grab_destroy(struct mirage *m);

#endif /* MIRAGE_H */
