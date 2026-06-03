#include "mirage.h"
#include <stdio.h>
#include <stdlib.h>

#include <wayland-egl.h>

static const char *VERT_SRC =
    "attribute vec3 aPos;\n"
    "attribute vec2 aUV;\n"
    "uniform mat4 uMVP;\n"
    "uniform float uYFlip;\n"
    "varying highp vec2 vUV;\n"
    "void main() {\n"
    "  gl_Position = uMVP * vec4(aPos, 1.0);\n"
    "  vUV = vec2(aUV.x, mix(aUV.y, 1.0 - aUV.y, uYFlip));\n"
    "}\n";

/* Contrast-adaptive sharpen (RCAS-style): an unsharp mask whose result is
 * clamped to the 4-neighbour min/max, so it crisps glyph stems WITHOUT the
 * overshoot halos a plain unsharp produces. Recovers apparent resolution lost
 * to minifying the desktop onto the arc - the cheap win the research flagged
 * over (bandwidth-hungry, near-useless here) supersampling. uTexel is one
 * source texel in UV; highp so the sub-pixel taps land precisely. */
static const char *FRAG_SRC =
    "precision mediump float;\n"
    "varying highp vec2 vUV;\n"
    "uniform sampler2D uTex;\n"
    "uniform float uHasTex;\n"
    "uniform vec3 uColor;\n"
    "uniform highp vec2 uTexel;\n"
    "uniform float uSharpen;\n"
    "void main() {\n"
    "  if (uHasTex < 0.5) { gl_FragColor = vec4(uColor, 1.0); return; }\n"
    "  vec3 e = texture2D(uTex, vUV).rgb;\n"
    "  if (uSharpen > 0.0) {\n"
    "    vec3 b = texture2D(uTex, vUV + vec2(0.0, -uTexel.y)).rgb;\n"
    "    vec3 d = texture2D(uTex, vUV + vec2(-uTexel.x, 0.0)).rgb;\n"
    "    vec3 f = texture2D(uTex, vUV + vec2( uTexel.x, 0.0)).rgb;\n"
    "    vec3 h = texture2D(uTex, vUV + vec2(0.0,  uTexel.y)).rgb;\n"
    "    vec3 s = e + (e - (b + d + f + h) * 0.25) * uSharpen;\n"
    "    e = clamp(s, min(min(b,d), min(f,h)), max(max(b,d), max(f,h)));\n"
    "  }\n"
    "  gl_FragColor = vec4(e, 1.0);\n"
    "}\n";

/* Loupe warp: a fullscreen pass that re-samples the rendered frame through a
 * rounded-RECTANGLE magnifier. A signed-distance field to a rounded box gives
 * the local magnification: uM everywhere INSIDE the rect (sdf<=0) so the whole
 * panel-shaped plateau is uniformly magnified with NO bowing, then a smoothstep
 * eases it back to 1x across a soft border band of thickness (uRout-uRin) - the
 * only place straight lines bend - and 1x (untouched) outside. Sampling is
 * suv = centre + offset / Mlocal, so uniform Mlocal = uniform zoom = straight
 * text. Distances are in half-height units, isotropic (p.y in [-1,1]). uRin is
 * the rect's half-height; its half-width and corner radius scale off it. A
 * sharpen scaled by Mlocal-1 crisps the upscaled centre and spares the 1x edge.
 * uM<=1 collapses to a pure passthrough. */
static const char *WARP_VERT =
    "attribute vec3 aPos;\n"
    "attribute vec2 aUV;\n"
    "varying highp vec2 vUV;\n"
    "void main() {\n"
    "  gl_Position = vec4(aPos.xy * 2.0, 0.0, 1.0);\n"
    "  vUV = vec2(aUV.x, 1.0 - aUV.y);\n"   /* QUAD uv is Y-down; flip to image-up */
    "}\n";
static const char *WARP_FRAG =
    "precision mediump float;\n"
    "varying highp vec2 vUV;\n"
    "uniform sampler2D uTex;\n"
    "uniform highp vec2 uTexel;\n"
    "uniform float uAspect;\n"          /* W/H, makes the metric isotropic */
    "uniform float uM;\n"              /* magnification (eased lens_power) */
    "uniform float uRin;\n"            /* rect half-height (plateau)       */
    "uniform float uRout;\n"           /* outer edge of the border band    */
    "uniform float uSharpen;\n"
    /* signed distance to a rounded box (half-extents b, corner radius r) */
    "float sdRoundBox(vec2 p, vec2 b, float r) {\n"
    "  vec2 q = abs(p) - b + vec2(r);\n"
    "  return length(max(q, 0.0)) + min(max(q.x, q.y), 0.0) - r;\n"
    "}\n"
    "void main() {\n"
    "  vec2 c = vUV - vec2(0.5);\n"
    "  vec2 p = vec2(c.x * uAspect, c.y) * 2.0;\n"   /* isotropic, p.y in [-1,1] */
    "  float by = uRin;\n"                            /* rect half-height        */
    "  float bx = uRin * 1.7;\n"                      /* landscape half-width    */
    "  float rc = uRin * 0.45;\n"                     /* corner radius           */
    "  float band = max(uRout - uRin, 1e-3);\n"
    "  float sdf = sdRoundBox(p, vec2(bx, by), rc);\n"
    "  float t = clamp(sdf / band, 0.0, 1.0);\n"
    "  float ease = t * t * (3.0 - 2.0 * t);\n"       /* C1 at both edges */
    "  float Ml = mix(uM, 1.0, ease);\n"              /* uM inside -> 1 outside */
    "  vec2 suv = vec2(0.5) + c / Ml;\n"
    "  vec3 e = texture2D(uTex, suv).rgb;\n"
    "  float sh = uSharpen * clamp(Ml - 1.0, 0.0, 1.0);\n"
    "  if (sh > 0.0) {\n"
    "    vec3 b = texture2D(uTex, suv + vec2(0.0, -uTexel.y)).rgb;\n"
    "    vec3 d = texture2D(uTex, suv + vec2(-uTexel.x, 0.0)).rgb;\n"
    "    vec3 g = texture2D(uTex, suv + vec2( uTexel.x, 0.0)).rgb;\n"
    "    vec3 hp = texture2D(uTex, suv + vec2(0.0,  uTexel.y)).rgb;\n"
    "    vec3 s = e + (e - (b + d + g + hp) * 0.25) * sh;\n"
    "    e = clamp(s, min(min(b,d), min(g,hp)), max(max(b,d), max(g,hp)));\n"
    "  }\n"
    "  gl_FragColor = vec4(e, 1.0);\n"
    "}\n";

static struct {
    GLuint prog;
    GLint  aPos, aUV;
    GLint  uMVP, uYFlip, uHasTex, uColor, uTex, uTexel, uSharpen;
    GLuint vbo;

    /* loupe warp pass + its offscreen target */
    GLuint warp;
    GLint  wTex, wTexel, wAspect, wM, wRin, wRout, wSharpen;
    GLuint fbo, fbo_tex, fbo_depth;
    int    fbo_w, fbo_h;
} R;

/* unit quad in XY plane: pos.xyz, uv.xy (interleaved), triangle strip */
static const GLfloat QUAD[] = {
    /*  x      y     z     u    v  */
    -0.5f,  0.5f, 0.0f,  0.0f, 0.0f,   /* top-left     */
     0.5f,  0.5f, 0.0f,  1.0f, 0.0f,   /* top-right    */
    -0.5f, -0.5f, 0.0f,  0.0f, 1.0f,   /* bottom-left  */
     0.5f, -0.5f, 0.0f,  1.0f, 1.0f,   /* bottom-right */
};

static GLuint compile(GLenum type, const char *src) {
    GLuint s = glCreateShader(type);
    glShaderSource(s, 1, &src, NULL);
    glCompileShader(s);
    GLint ok = 0;
    glGetShaderiv(s, GL_COMPILE_STATUS, &ok);
    if (!ok) {
        char log[1024];
        glGetShaderInfoLog(s, sizeof log, NULL, log);
        fprintf(stderr, "render: shader compile failed: %s\n", log);
        glDeleteShader(s);
        return 0;
    }
    return s;
}

static EGLConfig choose_config(EGLDisplay dpy) {
    const EGLint attrs[] = {
        EGL_SURFACE_TYPE,    EGL_WINDOW_BIT,
        EGL_RENDERABLE_TYPE, EGL_OPENGL_ES2_BIT,
        EGL_RED_SIZE, 8, EGL_GREEN_SIZE, 8, EGL_BLUE_SIZE, 8, EGL_ALPHA_SIZE, 8,
        EGL_NONE
    };
    EGLConfig cfg;
    EGLint n = 0;
    if (!eglChooseConfig(dpy, attrs, &cfg, 1, &n) || n < 1) return NULL;
    return cfg;
}

/* Build a curved screen: a vertical triangle strip swept along an arc of a
 * cylinder (radius = screen distance) centred on the viewer and centred on -Z.
 * The flat source image is mapped across the arc, so it bends around you and
 * every point faces the centre. */
static void build_curved_mesh(struct mirage *m, screen_t *s) {
    const mirage_config *c = &m->cfg;
    float d   = c->screen_distance_m;
    float ang = c->screen_arc_deg * (float)M_PI/180.0f;
    float aspect = (s->width > 0 && s->height > 0)
                   ? (float)s->height / (float)s->width : 9.0f/16.0f;
    float L = d * ang;          /* arc length = on-screen width */
    float h = L * aspect;       /* height preserves source aspect */

    const int cols = 64;
    int verts = (cols + 1) * 2;
    GLfloat *buf = malloc((size_t)verts * 5 * sizeof(GLfloat));
    int k = 0;
    for (int ci = 0; ci <= cols; ci++) {
        float u   = (float)ci / (float)cols;
        float phi = -ang * 0.5f + u * ang;
        float x   =  d * sinf(phi);
        float z   = -d * cosf(phi);
        buf[k++]=x; buf[k++]= h*0.5f; buf[k++]=z; buf[k++]=u; buf[k++]=0.0f; /* top */
        buf[k++]=x; buf[k++]=-h*0.5f; buf[k++]=z; buf[k++]=u; buf[k++]=1.0f; /* bottom */
    }
    glGenBuffers(1, &s->mesh_vbo);
    glBindBuffer(GL_ARRAY_BUFFER, s->mesh_vbo);
    glBufferData(GL_ARRAY_BUFFER, (GLsizeiptr)(verts*5*sizeof(GLfloat)), buf, GL_STATIC_DRAW);
    s->mesh_verts = verts;
    free(buf);
}

/* Same screen as build_curved_mesh but FLAT: a single quad tangent to the
 * radius-d sphere at -Z, subtending the same angular width (so the layout's
 * yaw spacing and gaps are unchanged). The layout still rotates it onto its
 * column and lifts it straight up - same orientation as the cylinder, just not
 * bent, so text stays straight. 4 verts (flat = already perspective-correct). */
static void build_flat_mesh(struct mirage *m, screen_t *s) {
    const mirage_config *c = &m->cfg;
    float d      = c->screen_distance_m;
    float ang_w  = c->screen_arc_deg * (float)M_PI/180.0f;
    float aspect = (s->width > 0 && s->height > 0)
                   ? (float)s->height / (float)s->width : 9.0f/16.0f;
    float hw = d * tanf(ang_w * 0.5f);       /* half width  (subtends ang_w) */
    float hh = hw * aspect;                  /* half height keeps 16:9       */

    const GLfloat q[] = {                    /* TL, TR, BL, BR (strip)       */
        -hw,  hh, -d,  0.0f, 0.0f,
         hw,  hh, -d,  1.0f, 0.0f,
        -hw, -hh, -d,  0.0f, 1.0f,
         hw, -hh, -d,  1.0f, 1.0f,
    };
    glGenBuffers(1, &s->mesh_vbo);
    glBindBuffer(GL_ARRAY_BUFFER, s->mesh_vbo);
    glBufferData(GL_ARRAY_BUFFER, sizeof q, q, GL_STATIC_DRAW);
    s->mesh_verts = 4;
}

bool render_init(struct mirage *m) {
    m->edpy = eglGetDisplay((EGLNativeDisplayType)m->display);
    if (m->edpy == EGL_NO_DISPLAY) { fprintf(stderr, "render: no EGL display\n"); return false; }
    EGLint major, minor;
    if (!eglInitialize(m->edpy, &major, &minor)) {
        fprintf(stderr, "render: eglInitialize failed\n"); return false;
    }
    if (!eglBindAPI(EGL_OPENGL_ES_API)) {
        fprintf(stderr, "render: eglBindAPI failed\n"); return false;
    }
    m->ecfg = choose_config(m->edpy);
    if (!m->ecfg) { fprintf(stderr, "render: no matching EGL config\n"); return false; }

    const EGLint ctx_attrs[] = { EGL_CONTEXT_CLIENT_VERSION, 2, EGL_NONE };
    m->ectx = eglCreateContext(m->edpy, m->ecfg, EGL_NO_CONTEXT, ctx_attrs);
    if (m->ectx == EGL_NO_CONTEXT) { fprintf(stderr, "render: no context\n"); return false; }

    m->egl_window = wl_egl_window_create(m->surface, m->glasses_w, m->glasses_h);
    if (!m->egl_window) { fprintf(stderr, "render: egl_window failed\n"); return false; }
    m->esurf = eglCreateWindowSurface(m->edpy, m->ecfg,
                                      (EGLNativeWindowType)m->egl_window, NULL);
    if (m->esurf == EGL_NO_SURFACE) { fprintf(stderr, "render: window surface failed\n"); return false; }

    if (!eglMakeCurrent(m->edpy, m->esurf, m->esurf, m->ectx)) {
        fprintf(stderr, "render: makeCurrent failed\n"); return false;
    }
    eglSwapInterval(m->edpy, 1);

    GLuint vs = compile(GL_VERTEX_SHADER, VERT_SRC);
    GLuint fs = compile(GL_FRAGMENT_SHADER, FRAG_SRC);
    if (!vs || !fs) return false;
    R.prog = glCreateProgram();
    glAttachShader(R.prog, vs);
    glAttachShader(R.prog, fs);
    glBindAttribLocation(R.prog, 0, "aPos");
    glBindAttribLocation(R.prog, 1, "aUV");
    glLinkProgram(R.prog);
    GLint ok = 0; glGetProgramiv(R.prog, GL_LINK_STATUS, &ok);
    if (!ok) {
        char log[1024]; glGetProgramInfoLog(R.prog, sizeof log, NULL, log);
        fprintf(stderr, "render: link failed: %s\n", log); return false;
    }
    glDeleteShader(vs); glDeleteShader(fs);

    R.aPos    = 0; R.aUV = 1;
    R.uMVP    = glGetUniformLocation(R.prog, "uMVP");
    R.uYFlip  = glGetUniformLocation(R.prog, "uYFlip");
    R.uHasTex = glGetUniformLocation(R.prog, "uHasTex");
    R.uColor  = glGetUniformLocation(R.prog, "uColor");
    R.uTex    = glGetUniformLocation(R.prog, "uTex");
    R.uTexel  = glGetUniformLocation(R.prog, "uTexel");
    R.uSharpen = glGetUniformLocation(R.prog, "uSharpen");

    /* loupe warp program (shares aPos/aUV attribute slots 0/1) */
    GLuint wv = compile(GL_VERTEX_SHADER, WARP_VERT);
    GLuint wf = compile(GL_FRAGMENT_SHADER, WARP_FRAG);
    if (!wv || !wf) return false;
    R.warp = glCreateProgram();
    glAttachShader(R.warp, wv);
    glAttachShader(R.warp, wf);
    glBindAttribLocation(R.warp, 0, "aPos");
    glBindAttribLocation(R.warp, 1, "aUV");
    glLinkProgram(R.warp);
    glGetProgramiv(R.warp, GL_LINK_STATUS, &ok);
    if (!ok) {
        char log[1024]; glGetProgramInfoLog(R.warp, sizeof log, NULL, log);
        fprintf(stderr, "render: warp link failed: %s\n", log); return false;
    }
    glDeleteShader(wv); glDeleteShader(wf);
    R.wTex     = glGetUniformLocation(R.warp, "uTex");
    R.wTexel   = glGetUniformLocation(R.warp, "uTexel");
    R.wAspect  = glGetUniformLocation(R.warp, "uAspect");
    R.wM       = glGetUniformLocation(R.warp, "uM");
    R.wRin     = glGetUniformLocation(R.warp, "uRin");
    R.wRout    = glGetUniformLocation(R.warp, "uRout");
    R.wSharpen = glGetUniformLocation(R.warp, "uSharpen");

    glGenBuffers(1, &R.vbo);
    glBindBuffer(GL_ARRAY_BUFFER, R.vbo);
    glBufferData(GL_ARRAY_BUFFER, sizeof QUAD, QUAD, GL_STATIC_DRAW);

    glEnable(GL_DEPTH_TEST);

    /* build one mesh per screen (flat quad or curved strip) */
    int nbuild = m->n_screen;
    if (nbuild < 0 || nbuild > MIRAGE_MAX_SCREENS) nbuild = 0;
    for (int i = 0; i < nbuild; i++) {
        if (m->cfg.geometry == GEOM_FLAT) build_flat_mesh(m, &m->screen[i]);
        else                              build_curved_mesh(m, &m->screen[i]);
    }

    fprintf(stderr, "render: EGL %d.%d, GL_RENDERER=%s\n", major, minor,
            (const char*)glGetString(GL_RENDERER));
    return true;
}

/* placeholder tints for screens with no capture yet */
static const float PLACEHOLDER[][3] = {
    {0.20f, 0.10f, 0.30f}, {0.10f, 0.25f, 0.20f}, {0.28f, 0.18f, 0.08f},
    {0.10f, 0.18f, 0.30f}, {0.25f, 0.10f, 0.15f},
};

/* Lazily (re)build the offscreen colour+depth target the scene renders into when
 * the loupe is active. Sized to the glasses surface. Returns false if the FBO
 * can't be completed - the caller then falls back to drawing straight to screen. */
static bool ensure_fbo(int w, int h) {
    if (R.fbo && R.fbo_w == w && R.fbo_h == h) return true;
    if (!R.fbo)       glGenFramebuffers(1, &R.fbo);
    if (!R.fbo_tex)   glGenTextures(1, &R.fbo_tex);
    if (!R.fbo_depth) glGenRenderbuffers(1, &R.fbo_depth);

    glBindTexture(GL_TEXTURE_2D, R.fbo_tex);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, w, h, 0, GL_RGBA, GL_UNSIGNED_BYTE, NULL);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);

    glBindRenderbuffer(GL_RENDERBUFFER, R.fbo_depth);
    glRenderbufferStorage(GL_RENDERBUFFER, GL_DEPTH_COMPONENT16, w, h);

    glBindFramebuffer(GL_FRAMEBUFFER, R.fbo);
    glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, R.fbo_tex, 0);
    glFramebufferRenderbuffer(GL_FRAMEBUFFER, GL_DEPTH_ATTACHMENT, GL_RENDERBUFFER, R.fbo_depth);
    GLenum st = glCheckFramebufferStatus(GL_FRAMEBUFFER);
    glBindFramebuffer(GL_FRAMEBUFFER, 0);
    if (st != GL_FRAMEBUFFER_COMPLETE) {
        fprintf(stderr, "render: loupe FBO incomplete (0x%x)\n", st);
        return false;
    }
    R.fbo_w = w; R.fbo_h = h;
    return true;
}

void render_frame(struct mirage *m, quat head) {
    eglMakeCurrent(m->edpy, m->esurf, m->esurf, m->ectx);

    /* Ease the loupe toward its target (lens_max while Alt held, else 1.0).
     * Active only above ~1, so the FBO indirection is skipped entirely when the
     * loupe is off - zero cost in the common case. */
    m->lens_power += (m->lens_target - m->lens_power) * 0.2f;
    bool loupe = m->lens_power > 1.001f && ensure_fbo(m->glasses_w, m->glasses_h);

    glBindFramebuffer(GL_FRAMEBUFFER, loupe ? R.fbo : 0);
    glViewport(0, 0, m->glasses_w, m->glasses_h);
    glClearColor(m->cfg.bg[0], m->cfg.bg[1], m->cfg.bg[2], 1.0f);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    float z = m->zoom > 0.0f ? m->zoom : 1.0f;
    float aspect = (float)m->glasses_w / (float)m->glasses_h;
    /* zoom narrows the field of view (zoom in = see less, bigger) */
    mat4 proj = m4_perspective((m->cfg.fov_deg / z) * (float)M_PI/180.0f, aspect, 0.05f, 100.0f);

    /* Reshape the head orientation for comfort: amplify yaw so the side screens
     * need less neck turn, and damp roll so the wall stays near-level when you
     * look around (the glasses sit pitched on your nose, so a plain yaw injects
     * roll - see q_to_euler_ypr). Identity when yaw_gain=1 and roll_damp=1. */
    float yaw, pitch, roll;
    q_to_euler_ypr(head, &yaw, &pitch, &roll);
    head = q_from_euler_ypr(yaw * m->cfg.yaw_gain, pitch * m->cfg.pitch_gain,
                            roll * m->cfg.roll_damp);

    /* Reading-stability deadband. The comfort gains above amplify head tremor
     * 2.5-3x, so even a still head leaves text shimmering. We hold the last
     * presented orientation and only follow once the camera has moved past a
     * small angle, then follow the EXCESS over that angle - a soft deadband, so
     * crossing the threshold doesn't snap. Sub-threshold tremor is frozen out
     * entirely (text sits dead still), while a real head turn dwarfs the
     * threshold and passes through with only ~deadband worth of lag. */
    if (m->cfg.read_deadband_deg > 0.0f) {
        static quat presented; static bool seeded = false;
        if (!seeded) { presented = head; seeded = true; }
        else {
            float dot = presented.w*head.w + presented.x*head.x
                      + presented.y*head.y + presented.z*head.z;
            if (dot < 0.0f) dot = -dot;
            if (dot > 1.0f) dot = 1.0f;
            float ang = 2.0f * acosf(dot);                 /* rad moved this frame */
            float db  = m->cfg.read_deadband_deg * (float)M_PI/180.0f;
            float follow = ang > 1e-6f ? (ang - db) / ang : 0.0f;
            if (follow < 0.0f) follow = 0.0f;              /* inside deadband: freeze */
            presented = q_nlerp(presented, head, follow);
        }
        head = presented;
    }

    /* Cmd+horizontal-scroll pan: ease the camera toward the focused display so
     * the whole wall glides until that screen sits dead-centre. Applied after
     * the comfort gains + deadband (it's a deliberate offset, not head tremor),
     * and composed in world space so head-turns stay relative to where we look. */
    {
        int nf = m->n_screen > 0 ? m->n_screen : m->cfg.screen_count;
        int f  = m->view_focus;
        if (f < 0) f = 0;
        if (nf > 0 && f >= nf) f = nf - 1;
        float ty = 0.0f, tp = 0.0f;
        layout_focus_angles(m, f, &ty, &tp);
        const float k = 0.15f;   /* per-frame easing toward the focused screen */
        m->pan_yaw   += (ty - m->pan_yaw)   * k;
        m->pan_pitch += (tp - m->pan_pitch) * k;
        quat pan = q_from_euler_ypr(m->pan_yaw, m->pan_pitch, 0.0f);
        head = q_mul(pan, head);
    }

    mat4 view = m4_from_quat(q_conj(head));   /* world -> head space */
    mat4 vp   = m4_mul(proj, view);

    glUseProgram(R.prog);

    int n = m->n_screen > 0 ? m->n_screen : m->cfg.screen_count;
    if (n > MIRAGE_MAX_SCREENS) n = MIRAGE_MAX_SCREENS;
    for (int i = 0; i < n; i++) {
        screen_t *s = &m->screen[i];
        if (!s->mesh_vbo) continue;

        mat4 model = layout_model_matrix(m, i);
        mat4 mvp   = m4_mul(vp, model);
        glUniformMatrix4fv(R.uMVP, 1, GL_FALSE, mvp.m);

        glBindBuffer(GL_ARRAY_BUFFER, s->mesh_vbo);
        glEnableVertexAttribArray(R.aPos);
        glVertexAttribPointer(R.aPos, 3, GL_FLOAT, GL_FALSE, 5*sizeof(GLfloat), (void*)0);
        glEnableVertexAttribArray(R.aUV);
        glVertexAttribPointer(R.aUV, 2, GL_FLOAT, GL_FALSE, 5*sizeof(GLfloat), (void*)(3*sizeof(GLfloat)));

        if (s->have_tex) {
            glActiveTexture(GL_TEXTURE0);
            glBindTexture(GL_TEXTURE_2D, s->tex);
            glUniform1i(R.uTex, 0);
            glUniform1f(R.uHasTex, 1.0f);
            glUniform1f(R.uYFlip, s->y_invert ? 1.0f : 0.0f);
            glUniform2f(R.uTexel, s->width > 0 ? 1.0f/(float)s->width : 0.0f,
                                  s->height > 0 ? 1.0f/(float)s->height : 0.0f);
            glUniform1f(R.uSharpen, m->cfg.sharpen);
        } else {
            const float *p = PLACEHOLDER[i % 5];
            glUniform3f(R.uColor, p[0], p[1], p[2]);
            glUniform1f(R.uHasTex, 0.0f);
            glUniform1f(R.uYFlip, 0.0f);
        }
        glDrawArrays(GL_TRIANGLE_STRIP, 0, s->mesh_verts);
    }

    /* Loupe: resolve the offscreen scene to the screen through the radial warp.
     * Fullscreen quad, depth off (it covers every pixel and writes no depth). */
    if (loupe) {
        glBindFramebuffer(GL_FRAMEBUFFER, 0);
        glViewport(0, 0, m->glasses_w, m->glasses_h);
        glDisable(GL_DEPTH_TEST);
        glUseProgram(R.warp);
        glActiveTexture(GL_TEXTURE0);
        glBindTexture(GL_TEXTURE_2D, R.fbo_tex);
        glUniform1i(R.wTex, 0);
        glUniform2f(R.wTexel, 1.0f/(float)m->glasses_w, 1.0f/(float)m->glasses_h);
        glUniform1f(R.wAspect, (float)m->glasses_w / (float)m->glasses_h);
        glUniform1f(R.wM, m->lens_power);
        glUniform1f(R.wRin, m->cfg.lens_rin);
        glUniform1f(R.wRout, m->cfg.lens_rout);
        glUniform1f(R.wSharpen, m->cfg.sharpen);

        glBindBuffer(GL_ARRAY_BUFFER, R.vbo);
        glEnableVertexAttribArray(R.aPos);
        glVertexAttribPointer(R.aPos, 3, GL_FLOAT, GL_FALSE, 5*sizeof(GLfloat), (void*)0);
        glEnableVertexAttribArray(R.aUV);
        glVertexAttribPointer(R.aUV, 2, GL_FLOAT, GL_FALSE, 5*sizeof(GLfloat), (void*)(3*sizeof(GLfloat)));
        glDrawArrays(GL_TRIANGLE_STRIP, 0, 4);
        glEnable(GL_DEPTH_TEST);
    }

    eglSwapBuffers(m->edpy, m->esurf);
}

/* Flat capture-only view: lay the N captured screens out in a row, each fit to
 * its cell preserving the source aspect. No pose, no perspective. Rendered into
 * an arbitrary target surface so both the glasses (flat mode) and the laptop
 * preview window can share it. */
static void render_flat_to(struct mirage *m, EGLSurface surf, int W, int H) {
    eglMakeCurrent(m->edpy, surf, surf, m->ectx);
    glViewport(0, 0, W, H);
    glClearColor(m->cfg.bg[0], m->cfg.bg[1], m->cfg.bg[2], 1.0f);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    int n = m->n_screen > 0 ? m->n_screen : m->cfg.screen_count;
    mat4 proj = m4_ortho(0, (float)W, 0, (float)H, -1, 1);

    glUseProgram(R.prog);
    glBindBuffer(GL_ARRAY_BUFFER, R.vbo);
    glEnableVertexAttribArray(R.aPos);
    glVertexAttribPointer(R.aPos, 3, GL_FLOAT, GL_FALSE, 5*sizeof(GLfloat), (void*)0);
    glEnableVertexAttribArray(R.aUV);
    glVertexAttribPointer(R.aUV, 2, GL_FLOAT, GL_FALSE, 5*sizeof(GLfloat), (void*)(3*sizeof(GLfloat)));

    float cellW = (float)W / (float)n;
    for (int i = 0; i < n; i++) {
        screen_t *s = &m->screen[i];
        float srcA = (s->width > 0 && s->height > 0)
                     ? (float)s->height/(float)s->width : 9.0f/16.0f;
        float rw = cellW * 0.96f;
        float rh = rw * srcA;
        if (rh > H * 0.96f) { rh = H * 0.96f; rw = rh / srcA; }
        float z = m->zoom > 0.0f ? m->zoom : 1.0f;   /* Super+scroll zoom */
        rw *= z; rh *= z;
        float cx = cellW * (i + 0.5f);
        float cy = H * 0.5f;

        mat4 model = m4_mul(m4_translate(v3(cx, cy, 0)), m4_scale(v3(rw, rh, 1)));
        mat4 mvp   = m4_mul(proj, model);
        glUniformMatrix4fv(R.uMVP, 1, GL_FALSE, mvp.m);

        if (s->have_tex) {
            glActiveTexture(GL_TEXTURE0);
            glBindTexture(GL_TEXTURE_2D, s->tex);
            glUniform1i(R.uTex, 0);
            glUniform1f(R.uHasTex, 1.0f);
            glUniform1f(R.uYFlip, s->y_invert ? 1.0f : 0.0f);
            glUniform2f(R.uTexel, s->width > 0 ? 1.0f/(float)s->width : 0.0f,
                                  s->height > 0 ? 1.0f/(float)s->height : 0.0f);
            glUniform1f(R.uSharpen, m->cfg.sharpen);
        } else {
            const float *p = PLACEHOLDER[i % 5];
            glUniform3f(R.uColor, p[0], p[1], p[2]);
            glUniform1f(R.uHasTex, 0.0f);
            glUniform1f(R.uYFlip, 0.0f);
        }
        glDrawArrays(GL_TRIANGLE_STRIP, 0, 4);
    }
    eglSwapBuffers(m->edpy, surf);
}

void render_frame_flat(struct mirage *m) {
    render_flat_to(m, m->esurf, m->glasses_w, m->glasses_h);
}

/* ---- laptop preview window: a second surface showing the same flat view ---- */
bool render_preview_init(struct mirage *m) {
    if (!m->pv_surface) return false;
    m->pv_egl_window = wl_egl_window_create(m->pv_surface, m->pv_w, m->pv_h);
    if (!m->pv_egl_window) { fprintf(stderr, "preview: egl_window failed\n"); return false; }
    m->pv_esurf = eglCreateWindowSurface(m->edpy, m->ecfg,
                                         (EGLNativeWindowType)m->pv_egl_window, NULL);
    if (m->pv_esurf == EGL_NO_SURFACE) { fprintf(stderr, "preview: surface failed\n"); return false; }
    /* don't vsync the preview - it must never pace (or stall) the glasses arc */
    eglMakeCurrent(m->edpy, m->pv_esurf, m->pv_esurf, m->ectx);
    eglSwapInterval(m->edpy, 0);
    fprintf(stderr, "preview: window ready (%dx%d)\n", m->pv_w, m->pv_h);
    return true;
}

void render_preview(struct mirage *m) {
    if (m->pv_esurf == EGL_NO_SURFACE) return;
    if (m->pv_cfg_w > 0 && (m->pv_cfg_w != m->pv_w || m->pv_cfg_h != m->pv_h)) {
        m->pv_w = m->pv_cfg_w; m->pv_h = m->pv_cfg_h;
        wl_egl_window_resize(m->pv_egl_window, m->pv_w, m->pv_h, 0, 0);
    }
    render_flat_to(m, m->pv_esurf, m->pv_w, m->pv_h);
}

void render_finish(struct mirage *m) {
    if (m->edpy == EGL_NO_DISPLAY) return;
    eglMakeCurrent(m->edpy, EGL_NO_SURFACE, EGL_NO_SURFACE, EGL_NO_CONTEXT);
    if (m->pv_esurf && m->pv_esurf != EGL_NO_SURFACE) eglDestroySurface(m->edpy, m->pv_esurf);
    if (m->pv_egl_window) wl_egl_window_destroy(m->pv_egl_window);
    if (m->esurf != EGL_NO_SURFACE) eglDestroySurface(m->edpy, m->esurf);
    if (m->ectx  != EGL_NO_CONTEXT) eglDestroyContext(m->edpy, m->ectx);
    if (m->egl_window) wl_egl_window_destroy(m->egl_window);
    eglTerminate(m->edpy);
}
