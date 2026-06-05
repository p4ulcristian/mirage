#include "mirage.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

#include <wayland-egl.h>

/* MIRAGE_PROFILE timing helper: milliseconds between two monotonic samples. */
static double prof_ms(struct timespec a, struct timespec b) {
    return (b.tv_sec - a.tv_sec) * 1000.0 + (b.tv_nsec - a.tv_nsec) / 1e6;
}

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

/* Ground plane: a big horizontal quad of world positions. The fragment shades
 * it as dim grass-green that FADES to black (= transparent on the see-through
 * optics) past a radius, so it reads as a glowing pool under the wall with no
 * hard rectangle edge cutting across the real room. A faint 1 m grid gives
 * strong motion parallax as the head moves - the main "I'm standing on ground"
 * cue. Drawn additively, so the faded edge adds nothing and just shows through. */
static const char *FLOOR_VERT =
    "attribute vec3 aPos;\n"
    "uniform mat4 uMVP;\n"
    "varying highp vec2 vXZ;\n"
    "void main() {\n"
    "  gl_Position = uMVP * vec4(aPos, 1.0);\n"
    "  vXZ = aPos.xz;\n"
    "}\n";
static const char *FLOOR_FRAG =
    "precision highp float;\n"
    "varying highp vec2 vXZ;\n"
    "uniform vec3  uGrass;\n"   /* near grass tint                          */
    "uniform vec3  uHaze;\n"    /* far horizon ground tint (blends to sky)  */
    "uniform float uCell;\n"    /* grid spacing in metres                   */
    "float hash(vec2 p){ return fract(sin(dot(p, vec2(127.1, 311.7))) * 43758.5453); }\n"
    "float vnoise(vec2 p){ vec2 i=floor(p), f=fract(p); f=f*f*(3.0-2.0*f);\n"
    "  float a=hash(i), b=hash(i+vec2(1.0,0.0)), c=hash(i+vec2(0.0,1.0)), d=hash(i+vec2(1.0,1.0));\n"
    "  return mix(mix(a,b,f.x), mix(c,d,f.x), f.y); }\n"
    "float fbm(vec2 p){ float v=0.0, a=0.5; for (int i=0;i<3;i++){ v+=a*vnoise(p); p*=2.0; a*=0.5; } return v; }\n"
    "void main() {\n"
    "  highp float dist = length(vXZ);\n"
    "  float detail = 1.0 - smoothstep(5.0, 45.0, dist);\n"   /* hide noise far (anti-shimmer) */
    "  float patches = fbm(vXZ * 0.5);\n"                     /* broad grass tone variation  */
    "  float fine    = fbm(vXZ * 2.5);\n"                     /* finer blades                */
    "  vec3 grass = uGrass * (0.70 + 0.55 * patches * detail);\n"
    "  grass += vec3(0.0, 0.05, 0.0) * fine * detail;\n"
    "  highp vec2 fr = fract(vXZ / uCell);\n"
    "  highp vec2 dl = min(fr, 1.0 - fr);\n"
    "  float line = 1.0 - smoothstep(0.0, 0.03, min(dl.x, dl.y));\n"
    "  float gridVis = 1.0 - smoothstep(8.0, 22.0, dist);\n"  /* grid only nearby            */
    "  grass += uGrass * 0.45 * line * gridVis;\n"
    "  float far = smoothstep(20.0, 130.0, dist);\n"          /* blend to haze, never black  */
    "  vec3 col = mix(grass, uHaze, far);\n"
    "  float edge = 1.0 - smoothstep(150.0, 200.0, dist);\n"  /* ease out the quad's far rim */
    "  gl_FragColor = vec4(col * edge, 1.0);\n"
    "}\n";

/* Drop shadow: a soft-edged quad on the floor that DARKENS the floor glow
 * (blend dst*(1-alpha)), so it removes our own emitted light rather than trying
 * to paint black (impossible on the see-through optics). uv runs 0..1 over the
 * quad; alpha fades near the edges for a soft penumbra. */
static const char *SHADOW_VERT =
    "attribute vec3 aPos;\n"
    "attribute vec2 aUV;\n"
    "uniform mat4 uMVP;\n"
    "varying highp vec2 vUV;\n"
    "void main() { gl_Position = uMVP * vec4(aPos, 1.0); vUV = aUV; }\n";
static const char *SHADOW_FRAG =
    "precision mediump float;\n"
    "varying highp vec2 vUV;\n"
    "uniform float uStrength;\n"
    "void main() {\n"
    "  vec2 d = min(vUV, 1.0 - vUV);\n"                 /* distance to edge 0..0.5 */
    "  float e = smoothstep(0.0, 0.22, min(d.x, d.y));\n"
    "  gl_FragColor = vec4(0.0, 0.0, 0.0, uStrength * e);\n"
    "}\n";

/* Sky dome: a sphere of directions around the (only-ever-rotating) eye. Horizon
 * -> zenith gradient plus cheap fbm clouds, faded to 0 at/below the horizon so
 * it never tints the floor or the real lower field. Drawn additively. */
static const char *SKY_VERT =
    "attribute vec3 aPos;\n"
    "uniform mat4 uMVP;\n"
    "varying highp vec3 vDir;\n"
    "void main() { gl_Position = uMVP * vec4(aPos, 1.0); vDir = aPos; }\n";
static const char *SKY_FRAG =
    "precision highp float;\n"
    "varying highp vec3 vDir;\n"
    "uniform float uIntensity;\n"
    "float hash(vec2 p){ return fract(sin(dot(p, vec2(127.1, 311.7))) * 43758.5453); }\n"
    "float vnoise(vec2 p){ vec2 i=floor(p), f=fract(p); f=f*f*(3.0-2.0*f);\n"
    "  float a=hash(i), b=hash(i+vec2(1.0,0.0)), c=hash(i+vec2(0.0,1.0)), d=hash(i+vec2(1.0,1.0));\n"
    "  return mix(mix(a,b,f.x), mix(c,d,f.x), f.y); }\n"
    "float fbm(vec2 p){ float v=0.0, a=0.5; for (int i=0;i<4;i++){ v+=a*vnoise(p); p*=2.0; a*=0.5; } return v; }\n"
    "void main() {\n"
    "  vec3 dir = normalize(vDir);\n"
    "  float h = clamp(dir.y, 0.0, 1.0);\n"
    "  vec3 horizon = vec3(0.16, 0.22, 0.30);\n"
    "  vec3 zenith  = vec3(0.04, 0.09, 0.20);\n"
    "  vec3 sky = mix(horizon, zenith, sqrt(h));\n"
    "  vec2 cp = dir.xz / (dir.y + 0.25);\n"            /* project onto a high cloud plane */
    "  float puff = smoothstep(0.45, 0.85, fbm(cp * 1.5));\n"
    "  sky += vec3(0.18) * puff * (1.0 - 0.3 * h);\n"   /* whiter clouds near the horizon  */
    "  float band = smoothstep(-0.02, 0.10, dir.y);\n"  /* fade out at/below the horizon   */
    "  gl_FragColor = vec4(sky * band * uIntensity, 1.0);\n"
    "}\n";

/* Terrain: a noise-displaced heightfield mountain landscape you float above.
 * Vertices carry a precomputed normal; the fragment shades by altitude (valley
 * -> rock -> snow) and sun angle, then hazes into the horizon with distance.
 * On the see-through optics the snow/lit slopes glow while shaded valleys go
 * dark = transparent, which reads naturally as deep shade. */
static const char *TERRAIN_VERT =
    "attribute vec3 aPos;\n"
    "attribute vec3 aNormal;\n"
    "attribute vec2 aShade;\n"   /* baked: x = sun shadow (1 lit), y = AO (1 open) */
    "uniform mat4 uMVP;\n"
    "varying highp vec3 vWorld;\n"
    "varying highp vec3 vN;\n"
    "varying highp vec2 vShade;\n"
    "void main() {\n"
    "  gl_Position = uMVP * vec4(aPos, 1.0);\n"
    "  vWorld = aPos;\n"
    "  vN = aNormal;\n"
    "  vShade = aShade;\n"
    "}\n";
static const char *TERRAIN_FRAG =
    "precision highp float;\n"
    "varying highp vec3 vWorld;\n"
    "varying highp vec3 vN;\n"
    "varying highp vec2 vShade;\n"
    "uniform vec3  uSun;\n"     /* unit direction TO the sun        */
    "uniform vec3  uHaze;\n"    /* cool horizon haze colour (sky)   */
    "uniform float uSnow;\n"    /* snow-line altitude (y)           */
    "uniform float uRock;\n"    /* rock-line altitude (y)           */
    "float h21(vec2 p){ return fract(sin(dot(p, vec2(127.1, 311.7))) * 43758.5453); }\n"
    "float vn2(vec2 p){ vec2 i=floor(p), f=fract(p); f=f*f*(3.0-2.0*f);\n"
    "  float a=h21(i), b=h21(i+vec2(1.0,0.0)), c=h21(i+vec2(0.0,1.0)), d=h21(i+vec2(1.0,1.0));\n"
    "  return mix(mix(a,b,f.x), mix(c,d,f.x), f.y); }\n"
    "float fbm2(vec2 p){ float v=0.0,a=0.5; for(int i=0;i<2;i++){ v+=a*vn2(p); p*=2.0; a*=0.5; } return v; }\n"
    "void main() {\n"
    "  vec3 gN = normalize(vN);\n"
    "  float slope = 1.0 - gN.y;\n"                      /* 0 flat .. 1 vertical (geometric) */
    "  vec2 dp = vWorld.xz * 0.6;\n"                     /* detail normal from a noise gradient */
    "  float n0 = fbm2(dp), nx = fbm2(dp + vec2(0.45,0.0)), nz = fbm2(dp + vec2(0.0,0.45));\n"
    "  vec3 n = normalize(gN + vec3(n0 - nx, 0.0, n0 - nz) * 0.7);\n"
    "  float m = fbm2(vWorld.xz * 0.08);\n"              /* material band jitter */
    "  float y = vWorld.y + (m - 0.5) * 6.0;\n"
    "  vec3 grass = vec3(0.06, 0.13, 0.06);\n"
    "  vec3 rock  = vec3(0.22, 0.20, 0.18);\n"
    "  vec3 scree = vec3(0.30, 0.27, 0.23);\n"
    "  vec3 snow  = vec3(0.85, 0.90, 1.00);\n"
    "  float rockAmt = max(smoothstep(uRock - 10.0, uRock + 4.0, y), smoothstep(0.35, 0.62, slope));\n"
    "  vec3 col = mix(grass, rock, rockAmt);\n"
    "  float screeAmt = smoothstep(0.30, 0.5, slope) * (1.0 - smoothstep(0.6, 0.82, slope))\n"
    "                 * smoothstep(uRock - 6.0, uRock + 8.0, y) * smoothstep(0.5, 0.7, m);\n"
    "  col = mix(col, scree, screeAmt * 0.7);\n"
    "  float snowLine = uSnow + (m - 0.5) * 8.0;\n"
    "  float snowAmt = smoothstep(snowLine - 3.0, snowLine + 3.0, vWorld.y) * (1.0 - smoothstep(0.5, 0.8, slope));\n"
    "  col = mix(col, snow, snowAmt);\n"
    "  float shadow = vShade.x, ao = vShade.y;\n"        /* baked sun visibility + sky openness */
    "  float diff = max(dot(n, uSun), 0.0) * shadow;\n"
    "  col *= 0.22 * ao + 0.78 * diff;\n"
    "  vec3 vd = normalize(vWorld);\n"                   /* aerial perspective (eye at origin) */
    "  float sun = max(dot(vd, uSun), 0.0);\n"
    "  vec3 haze = mix(uHaze, vec3(0.55, 0.45, 0.34), sun * sun);\n"   /* warm toward the sun */
    "  float distFog = smoothstep(60.0, 320.0, length(vWorld.xz));\n"
    "  float lowFog  = mix(1.15, 0.55, smoothstep(-42.0, -6.0, vWorld.y));\n"  /* valleys hazier */
    "  float fog = clamp(distFog * lowFog, 0.0, 1.0);\n"
    "  col = mix(col, haze, fog);\n"
    "  gl_FragColor = vec4(col, 1.0);\n"
    "}\n";

static struct {
    GLuint prog;
    GLint  aPos, aUV;
    GLint  uMVP, uYFlip, uHasTex, uColor, uTex, uTexel, uSharpen;
    GLuint vbo;

    /* ground plane */
    GLuint floor_prog, floor_vbo;
    GLint  fMVP, fGrass, fHaze, fCell;

    /* floor drop-shadows */
    GLuint shadow_prog;
    GLint  shMVP, shStrength;

    /* sky dome */
    GLuint sky_prog, sky_vbo;
    int    sky_verts;
    GLint  skMVP, skIntensity;

    /* terrain heightfield */
    GLuint terrain_prog, terrain_vbo;
    int    terrain_verts;
    GLint  tMVP, tSun, tHaze, tSnow, tRock;

    /* loupe warp pass + its offscreen target */
    GLuint warp;
    GLint  wTex, wTexel, wAspect, wM, wRin, wRout, wSharpen;
    GLuint fbo, fbo_tex, fbo_depth;
    int    fbo_w, fbo_h;

    /* "GAZE: ON/OFF" status plaque below the centre screen (baked text textures,
     * drawn on the unit QUAD via R.prog). */
    GLuint label_on, label_off;
    int    label_w, label_h;
    /* live FPS plaque: re-baked only when the integer value changes */
    GLuint label_fps;
    int    fps_w, fps_h, fps_val;
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

/* We deliberately ask for an OPAQUE (0-alpha) config: a fullscreen XR24 buffer
 * is what the compositor needs to direct-scanout the glasses overlay straight to
 * the panel (an alpha AR24 buffer is rejected with "mismatched format" and falls
 * back to compositing, capping us below the panel's refresh). The glasses optics
 * are additive, so we never needed per-pixel alpha anyway - black reads as
 * transparent regardless. */
static EGLConfig choose_config(EGLDisplay dpy) {
    const EGLint attrs[] = {
        EGL_SURFACE_TYPE,    EGL_WINDOW_BIT,
        EGL_RENDERABLE_TYPE, EGL_OPENGL_ES2_BIT,
        EGL_RED_SIZE, 8, EGL_GREEN_SIZE, 8, EGL_BLUE_SIZE, 8, EGL_ALPHA_SIZE, 0,
        EGL_NONE
    };
    EGLConfig cfgs[32];
    EGLint n = 0;
    if (!eglChooseConfig(dpy, attrs, cfgs, 32, &n) || n < 1) return NULL;
    /* eglChooseConfig's ALPHA_SIZE is a minimum, so it can still hand back an
     * alpha config; pick one with EXACTLY 0 alpha bits (true XRGB8888). */
    for (EGLint i = 0; i < n; i++) {
        EGLint a = 8;
        eglGetConfigAttrib(dpy, cfgs[i], EGL_ALPHA_SIZE, &a);
        if (a == 0) return cfgs[i];
    }
    return cfgs[0];
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

/* ---- slab body: give each screen real thickness ----------------------------
 * The textured front face is unchanged; this adds the 5 other faces of a box
 * extruded behind it (back + 4 sides). They're drawn as flat solid colour and
 * lit by a single fixed key light (brightness = ambient + n.L), so the top edge
 * glows and the underside fades to ~black = transparent on the see-through
 * optics. On the curved arc each side face catches the light differently across
 * the wall, which is what sells the panels as solid objects. */
static const vec3  SLAB_LIGHT   = {0.356f, 0.814f, 0.458f}; /* key dir: up/front/right (unit) */
static const float SLAB_TINT[3] = {0.55f, 0.57f, 0.62f};    /* cool-grey bezel             */
static const float SLAB_AMBIENT = 0.12f;                    /* fill so unlit faces aren't 0 */

/* face local normals, order: top, bottom, left, right, back */
static const vec3 SLAB_N[5] = {
    {0,1,0}, {0,-1,0}, {-1,0,0}, {1,0,0}, {0,0,-1}
};

/* Rotate a local direction into world space through a model matrix's upper-left
 * 3x3 (our model matrices are rotation+translation, so this gives the world
 * normal for lighting). Column-major: element (row,col) = m[col*4+row]. */
static vec3 m4_dir(const mat4 *m, vec3 v) {
    vec3 r = { m->m[0]*v.x + m->m[4]*v.y + m->m[8]*v.z,
               m->m[1]*v.x + m->m[5]*v.y + m->m[9]*v.z,
               m->m[2]*v.x + m->m[6]*v.y + m->m[10]*v.z };
    return v3_norm(r);
}

/* Transform a point (w=1) through a model matrix - rotation plus translation. */
static vec3 m4_point(const mat4 *m, vec3 v) {
    return (vec3){ m->m[0]*v.x + m->m[4]*v.y + m->m[8]*v.z  + m->m[12],
                   m->m[1]*v.x + m->m[5]*v.y + m->m[9]*v.z  + m->m[13],
                   m->m[2]*v.x + m->m[6]*v.y + m->m[10]*v.z + m->m[14] };
}

/* Box around the front panel, extruded back by slab_depth_m. Front half-extents
 * match the textured mesh so the slab wraps it (curved strips bulge a touch
 * proud of the front rim - reads fine). 5 faces x 2 tris, interleaved x,y,z,u,v
 * (uv unused). No-op when slab depth is 0 (flat panels). */
static void build_slab_mesh(struct mirage *m, screen_t *s) {
    const mirage_config *c = &m->cfg;
    if (c->slab_depth_m <= 0.0f) { s->slab_vbo = 0; s->slab_verts = 0; return; }
    float d      = c->screen_distance_m;
    float ang_w  = c->screen_arc_deg * (float)M_PI/180.0f;
    float aspect = (s->width > 0 && s->height > 0)
                   ? (float)s->height / (float)s->width : 9.0f/16.0f;
    float hw, hh;
    if (c->geometry == GEOM_FLAT) { hw = d * tanf(ang_w * 0.5f); hh = hw * aspect; }
    else { float L = d * ang_w; hw = L * 0.5f; hh = L * aspect * 0.5f; }
    float zf = -d;                       /* front rim (screen plane) */
    float zb = -d - c->slab_depth_m;     /* back rim                 */

    GLfloat buf[5*6*5];
    int k = 0;
    #define V(X,Y,Z) do { buf[k++]=(X); buf[k++]=(Y); buf[k++]=(Z); \
                          buf[k++]=0.0f; buf[k++]=0.0f; } while (0)
    #define QUAD(ax,ay,az, bx,by,bz, cx,cy,cz, dx,dy,dz) do { \
        V(ax,ay,az); V(bx,by,bz); V(cx,cy,cz); \
        V(ax,ay,az); V(cx,cy,cz); V(dx,dy,dz); } while (0)
    QUAD(-hw, hh,zf,  hw, hh,zf,  hw, hh,zb, -hw, hh,zb);  /* top    (+Y) */
    QUAD(-hw,-hh,zf,  hw,-hh,zf,  hw,-hh,zb, -hw,-hh,zb);  /* bottom (-Y) */
    QUAD(-hw, hh,zf, -hw,-hh,zf, -hw,-hh,zb, -hw, hh,zb);  /* left   (-X) */
    QUAD( hw, hh,zf,  hw,-hh,zf,  hw,-hh,zb,  hw, hh,zb);  /* right  (+X) */
    QUAD(-hw, hh,zb,  hw, hh,zb,  hw,-hh,zb, -hw,-hh,zb);  /* back   (-Z) */
    #undef QUAD
    #undef V

    glGenBuffers(1, &s->slab_vbo);
    glBindBuffer(GL_ARRAY_BUFFER, s->slab_vbo);
    glBufferData(GL_ARRAY_BUFFER, sizeof buf, buf, GL_STATIC_DRAW);
    s->slab_verts = 5*6;
}

/* ground-plane look (see FLOOR_FRAG). The quad reaches far enough to meet the
 * horizon; the shader blends grass -> haze with distance so it never cuts to
 * black, and eases out before the quad's real far edge. */
#define FLOOR_HALF   200.0f     /* quad half-extent (m) - out to the horizon */
#define FLOOR_CELL    1.0f      /* grid spacing (m)                          */
static const float FLOOR_GRASS[3] = {0.10f, 0.22f, 0.07f};  /* near grass green */
static const float FLOOR_HAZE[3]  = {0.10f, 0.14f, 0.18f};  /* far horizon haze */

/* One big horizontal quad at y = -floor_height, in world space (model identity).
 * Triangle strip: TL, TR, BL, BR over x,z in [-H,H]. */
static void build_floor_mesh(struct mirage *m) {
    float y = -m->cfg.floor_height_m;
    float H = FLOOR_HALF;
    const GLfloat q[] = {
        -H, y, -H,
         H, y, -H,
        -H, y,  H,
         H, y,  H,
    };
    glGenBuffers(1, &R.floor_vbo);
    glBindBuffer(GL_ARRAY_BUFFER, R.floor_vbo);
    glBufferData(GL_ARRAY_BUFFER, sizeof q, q, GL_STATIC_DRAW);
}

#define SHADOW_STRENGTH 0.6f    /* peak floor-glow dimming under a slab     */
#define SHADOW_LIFT     0.01f   /* raise the shadow above the floor (z-fight) */
/* Shadows use a near-OVERHEAD light (tilted slightly toward the viewer), NOT
 * the slab key light. The key light comes from the front, which would throw the
 * shadow behind the panel - where the panel itself hides it and you just see
 * the page. Overhead+forward drops the blob onto the visible grass below. */
static const vec3 SHADOW_LIGHT = {0.151f, 0.956f, -0.252f};  /* unit, up & toward viewer */

/* Project screen i's front face onto the floor along the key light to get its
 * drop-shadow quad (world space; static, so built once). No-op without a floor. */
static void build_shadow_mesh(struct mirage *m, screen_t *s, int i) {
    const mirage_config *c = &m->cfg;
    if (!c->floor_on || !c->shadows_on) { s->shadow_vbo = 0; return; }
    float d      = c->screen_distance_m;
    float ang_w  = c->screen_arc_deg * (float)M_PI/180.0f;
    float aspect = (s->width > 0 && s->height > 0)
                   ? (float)s->height / (float)s->width : 9.0f/16.0f;
    float hw, hh;
    if (c->geometry == GEOM_FLAT) { hw = d * tanf(ang_w * 0.5f); hh = hw * aspect; }
    else { float L = d * ang_w; hw = L * 0.5f; hh = L * aspect * 0.5f; }

    mat4 model = layout_model_matrix(m, i);
    vec3 corner[4] = {                          /* TL, TR, BL, BR of the front face */
        {-hw,  hh, -d}, { hw,  hh, -d}, {-hw, -hh, -d}, { hw, -hh, -d}
    };
    vec3 ldir = v3_scale(SHADOW_LIGHT, -1.0f);  /* direction the light travels (down) */
    float yf = -c->floor_height_m;
    GLfloat buf[4*5];
    const float uv[4][2] = {{0,0},{1,0},{0,1},{1,1}};
    int k = 0;
    for (int j = 0; j < 4; j++) {
        vec3 P = m4_point(&model, corner[j]);
        float t = (yf - P.y) / ldir.y;          /* hit the floor plane y = yf */
        vec3 S = v3_add(P, v3_scale(ldir, t));
        buf[k++] = S.x; buf[k++] = yf + SHADOW_LIFT; buf[k++] = S.z;
        buf[k++] = uv[j][0]; buf[k++] = uv[j][1];
    }
    glGenBuffers(1, &s->shadow_vbo);
    glBindBuffer(GL_ARRAY_BUFFER, s->shadow_vbo);
    glBufferData(GL_ARRAY_BUFFER, sizeof buf, buf, GL_STATIC_DRAW);
}

/* Sky dome look/size. */
#define SKY_R         40.0f
#define SKY_NLAT      16
#define SKY_NLON      32
#define SKY_INTENSITY 1.0f

/* UV-sphere dome of directions around the eye, from just below the horizon to
 * the zenith. Position doubles as the view direction in the shader. */
static void build_sky_dome(void) {
    const float lat0 = -10.0f * (float)M_PI/180.0f;   /* a little below horizon */
    const float lat1 =  90.0f * (float)M_PI/180.0f;
    int verts = SKY_NLAT * SKY_NLON * 6;
    GLfloat *buf = malloc((size_t)verts * 3 * sizeof(GLfloat));
    int k = 0;
    for (int i = 0; i < SKY_NLAT; i++) {
        float a0 = lat0 + (lat1 - lat0) * (float)i     / SKY_NLAT;
        float a1 = lat0 + (lat1 - lat0) * (float)(i+1) / SKY_NLAT;
        for (int j = 0; j < SKY_NLON; j++) {
            float o0 = 2.0f*(float)M_PI * (float)j     / SKY_NLON;
            float o1 = 2.0f*(float)M_PI * (float)(j+1) / SKY_NLON;
            #define SKYV(LAT,LON) do { \
                buf[k++] = SKY_R * cosf(LAT) * cosf(LON); \
                buf[k++] = SKY_R * sinf(LAT); \
                buf[k++] = SKY_R * cosf(LAT) * sinf(LON); } while (0)
            SKYV(a0,o0); SKYV(a1,o0); SKYV(a1,o1);
            SKYV(a0,o0); SKYV(a1,o1); SKYV(a0,o1);
            #undef SKYV
        }
    }
    glGenBuffers(1, &R.sky_vbo);
    glBindBuffer(GL_ARRAY_BUFFER, R.sky_vbo);
    glBufferData(GL_ARRAY_BUFFER, (GLsizeiptr)(verts*3*sizeof(GLfloat)), buf, GL_STATIC_DRAW);
    R.sky_verts = verts;
    free(buf);
}

/* ---- terrain heightfield ---------------------------------------------------
 * A grid of TERR_N x TERR_N vertices spanning +-TERR_HALF metres, raised by
 * ridged fbm into mountains. You float at the origin (y=0); peaks top out
 * TERR_PEAK_BELOW metres under you, so you skim just above them. */
#define TERR_N           192
#define TERR_HALF        280.0f    /* half-extent (m), out toward the horizon */
#define TERR_FEAT        60.0f     /* base mountain feature size (m)          */
#define TERR_AMP         36.0f     /* valley-to-peak height (m)               */
#define TERR_PEAK_BELOW   6.0f     /* highest peak sits this far below the eye */
#define TERR_BASE        (-TERR_PEAK_BELOW - TERR_AMP)  /* valley floor (y)    */
static const vec3  TERR_SUN    = {0.356f, 0.814f, 0.458f};  /* same dir as the slab key light */
static const float TERR_HAZE[3] = {0.16f, 0.22f, 0.30f};    /* horizon haze (matches sky)     */
#define TERR_SNOW_Y      (-14.0f)  /* above here: snow      */
#define TERR_ROCK_Y      (-30.0f)  /* above here: bare rock */

static float terr_hash(int x, int z) {
    unsigned int n = (unsigned int)(x * 374761393) + (unsigned int)(z * 668265263);
    n = (n ^ (n >> 13)) * 1274126177u;
    n ^= n >> 16;
    return (float)(n & 0xffffffu) / (float)0x1000000;
}
static float terr_vnoise(float x, float z) {
    float fxr = floorf(x), fzr = floorf(z);
    int xi = (int)fxr, zi = (int)fzr;
    float fx = x - fxr, fz = z - fzr;
    float ux = fx*fx*(3.0f-2.0f*fx), uz = fz*fz*(3.0f-2.0f*fz);
    float a = terr_hash(xi, zi),     b = terr_hash(xi+1, zi);
    float c = terr_hash(xi, zi+1),   d = terr_hash(xi+1, zi+1);
    return a + (b-a)*ux + (c-a)*uz + (a-b-c+d)*ux*uz;
}
static float terr_smoothstepf(float a, float b, float x) {
    float t = (x - a) / (b - a);
    t = t < 0.0f ? 0.0f : (t > 1.0f ? 1.0f : t);
    return t * t * (3.0f - 2.0f * t);
}
static float terr_fbm_plain(float x, float z, int oct) {   /* rolling fbm */
    float v = 0.0f, a = 0.5f, f = 1.0f;
    for (int i = 0; i < oct; i++) {
        v += a * terr_vnoise(x*f, z*f);
        f *= 2.0f; a *= 0.5f;
    }
    return v;
}
static float terr_ridged(float x, float z, int oct) {      /* sharp ridge fbm */
    float v = 0.0f, a = 0.5f, f = 1.0f;
    for (int i = 0; i < oct; i++) {
        float n = terr_vnoise(x*f, z*f);
        n = 1.0f - fabsf(2.0f*n - 1.0f);
        v += a * n * n;
        f *= 2.0f; a *= 0.5f;
    }
    return v;
}
/* world (x,z) -> height. Domain-warped ridged fbm (meandering ranges) gated by a
 * broad highland mask, so some areas rise into ranges and others stay low. */
static float terr_height(float x, float z) {
    float u = x / TERR_FEAT, v = z / TERR_FEAT;
    float wx = terr_fbm_plain(u * 0.5f + 11.3f, v * 0.5f +  7.1f, 4);
    float wz = terr_fbm_plain(u * 0.5f +  3.7f, v * 0.5f + 19.2f, 4);
    float WARP = 0.9f;
    float pu = u + WARP * (wx - 0.5f) * 2.0f;
    float pv = v + WARP * (wz - 0.5f) * 2.0f;
    float mask = terr_smoothstepf(0.35f, 0.85f, terr_fbm_plain(u * 0.18f, v * 0.18f, 3));
    float h = terr_ridged(pu, pv, 6) * (0.30f + 0.70f * mask);
    return TERR_BASE + TERR_AMP * h;
}

/* bilinear sample of the prebuilt height grid (cheap; used by the bakes) */
static float terr_sampleH(const float *H, int N, float step, float x, float z) {
    float gx = (x + TERR_HALF) / step, gz = (z + TERR_HALF) / step;
    if (gx < 0.0f) gx = 0.0f;
    if (gx > (float)(N-1)) gx = (float)(N-1);
    if (gz < 0.0f) gz = 0.0f;
    if (gz > (float)(N-1)) gz = (float)(N-1);
    int x0 = (int)gx, z0 = (int)gz;
    int x1 = x0 < N-1 ? x0+1 : x0, z1 = z0 < N-1 ? z0+1 : z0;
    float fx = gx - x0, fz = gz - z0;
    float h0 = H[z0*N+x0] + (H[z0*N+x1] - H[z0*N+x0]) * fx;
    float h1 = H[z1*N+x0] + (H[z1*N+x1] - H[z1*N+x0]) * fx;
    return h0 + (h1 - h0) * fz;
}

/* baked SUN shadow: march toward the sun; if a ridge rises above the sun ray,
 * the point is shaded. Returns 1=lit .. 0=shadowed (soft). */
static float terr_bake_shadow(const float *H, int N, float step, float x, float z, float y) {
    float horiz = sqrtf(TERR_SUN.x*TERR_SUN.x + TERR_SUN.z*TERR_SUN.z);
    float sunSlope = TERR_SUN.y / horiz;
    float dx = TERR_SUN.x / horiz, dz = TERR_SUN.z / horiz;
    float maxSlope = -1e9f, d = step;
    for (int i = 0; i < 48 && d < 200.0f; i++) {
        float h = terr_sampleH(H, N, step, x + dx*d, z + dz*d);
        float s = (h - y) / d;
        if (s > maxSlope) maxSlope = s;
        d *= 1.12f;
    }
    return 1.0f - terr_smoothstepf(0.0f, 0.16f, maxSlope - sunSlope);
}

/* baked ambient occlusion: average horizon openness over 8 directions; valleys
 * and gullies (walled in) come back darker. Returns 1=open .. 0=occluded. */
static float terr_bake_ao(const float *H, int N, float step, float x, float z, float y) {
    const int DIRS = 8;
    float occ = 0.0f;
    for (int k = 0; k < DIRS; k++) {
        float a = 6.2831853f * (float)k / DIRS;
        float dx = cosf(a), dz = sinf(a), maxSlope = 0.0f, d = step;
        for (int i = 0; i < 8 && d < 45.0f; i++) {
            float h = terr_sampleH(H, N, step, x + dx*d, z + dz*d);
            float s = (h - y) / d;
            if (s > maxSlope) maxSlope = s;
            d *= 1.45f;
        }
        occ += sinf(atanf(maxSlope));      /* fraction of sky blocked this way */
    }
    float ao = 1.0f - occ / (float)DIRS;
    return ao < 0.0f ? 0.0f : ao;
}

/* push one vertex: pos(3) + normal(3) + shade(2 = shadow, ao) = 8 floats */
static void terr_pushv(GLfloat *buf, int *k, int i, int j,
                       const float *H, const float *SH, const float *AO, int N, float step) {
    int il = i>0 ? i-1 : 0, ir = i<N-1 ? i+1 : N-1;
    int jd = j>0 ? j-1 : 0, ju = j<N-1 ? j+1 : N-1;
    float hl = H[j*N+il], hr = H[j*N+ir], hd = H[jd*N+i], hu = H[ju*N+i];
    vec3 nrm = v3_norm(v3(hl - hr, 2.0f*step, hd - hu));
    buf[(*k)++] = -TERR_HALF + i*step;
    buf[(*k)++] = H[j*N+i];
    buf[(*k)++] = -TERR_HALF + j*step;
    buf[(*k)++] = nrm.x; buf[(*k)++] = nrm.y; buf[(*k)++] = nrm.z;
    buf[(*k)++] = SH[j*N+i]; buf[(*k)++] = AO[j*N+i];
}

static void build_terrain(void) {
    int N = TERR_N;
    float step = (2.0f*TERR_HALF) / (float)(N-1);
    float *H  = malloc((size_t)N*N*sizeof(float));
    float *SH = malloc((size_t)N*N*sizeof(float));
    float *AO = malloc((size_t)N*N*sizeof(float));
    for (int j = 0; j < N; j++)
        for (int i = 0; i < N; i++)
            H[j*N+i] = terr_height(-TERR_HALF + i*step, -TERR_HALF + j*step);
    /* bake the static lighting once (samples the cheap H grid, not the noise) */
    for (int j = 0; j < N; j++) {
        for (int i = 0; i < N; i++) {
            float x = -TERR_HALF + i*step, z = -TERR_HALF + j*step, y = H[j*N+i];
            SH[j*N+i] = terr_bake_shadow(H, N, step, x, z, y);
            AO[j*N+i] = terr_bake_ao(H, N, step, x, z, y);
        }
    }

    int verts = (N-1)*(N-1)*6;
    GLfloat *buf = malloc((size_t)verts * 8 * sizeof(GLfloat));
    int k = 0;
    for (int j = 0; j < N-1; j++) {
        for (int i = 0; i < N-1; i++) {
            terr_pushv(buf, &k, i,   j,   H, SH, AO, N, step);
            terr_pushv(buf, &k, i+1, j,   H, SH, AO, N, step);
            terr_pushv(buf, &k, i+1, j+1, H, SH, AO, N, step);
            terr_pushv(buf, &k, i,   j,   H, SH, AO, N, step);
            terr_pushv(buf, &k, i+1, j+1, H, SH, AO, N, step);
            terr_pushv(buf, &k, i,   j+1, H, SH, AO, N, step);
        }
    }
    free(H); free(SH); free(AO);
    glGenBuffers(1, &R.terrain_vbo);
    glBindBuffer(GL_ARRAY_BUFFER, R.terrain_vbo);
    glBufferData(GL_ARRAY_BUFFER, (GLsizeiptr)(verts*8*sizeof(GLfloat)), buf, GL_STATIC_DRAW);
    R.terrain_verts = verts;
    free(buf);
}

/* ---- status plaque text ---------------------------------------------------
 * A 5x7 bitmap font, just the glyphs the "GAZE: ON/OFF" label needs. Each glyph
 * is 7 rows; the low 5 bits of each byte are the pixels, MSB (bit 4) leftmost.
 * We rasterise a whole string into an RGBA texture once at init (two of them:
 * ON in green, OFF in grey), then draw it on a quad below the centre screen. */
#define GLYPH_W 5
#define GLYPH_H 7
static const unsigned char *glyph_rows(char ch) {
    static const unsigned char G[] = {0x0E,0x11,0x10,0x17,0x11,0x11,0x0E};
    static const unsigned char A[] = {0x0E,0x11,0x11,0x1F,0x11,0x11,0x11};
    static const unsigned char Z[] = {0x1F,0x01,0x02,0x04,0x08,0x10,0x1F};
    static const unsigned char E[] = {0x1F,0x10,0x10,0x1E,0x10,0x10,0x1F};
    static const unsigned char O[] = {0x0E,0x11,0x11,0x11,0x11,0x11,0x0E};
    static const unsigned char N[] = {0x11,0x19,0x15,0x13,0x11,0x11,0x11};
    static const unsigned char F[] = {0x1F,0x10,0x10,0x1E,0x10,0x10,0x10};
    static const unsigned char P[] = {0x1E,0x11,0x11,0x1E,0x10,0x10,0x10};
    static const unsigned char S[] = {0x0F,0x10,0x10,0x0E,0x01,0x01,0x1E};
    static const unsigned char C[] = {0x00,0x04,0x04,0x00,0x04,0x04,0x00}; /* colon */
    static const unsigned char SP[] = {0,0,0,0,0,0,0};                    /* space */
    static const unsigned char D0[] = {0x0E,0x11,0x13,0x15,0x19,0x11,0x0E};
    static const unsigned char D1[] = {0x04,0x0C,0x04,0x04,0x04,0x04,0x0E};
    static const unsigned char D2[] = {0x0E,0x11,0x01,0x02,0x04,0x08,0x1F};
    static const unsigned char D3[] = {0x1F,0x02,0x04,0x02,0x01,0x11,0x0E};
    static const unsigned char D4[] = {0x02,0x06,0x0A,0x12,0x1F,0x02,0x02};
    static const unsigned char D5[] = {0x1F,0x10,0x1E,0x01,0x01,0x11,0x0E};
    static const unsigned char D6[] = {0x06,0x08,0x10,0x1E,0x11,0x11,0x0E};
    static const unsigned char D7[] = {0x1F,0x01,0x02,0x04,0x08,0x08,0x08};
    static const unsigned char D8[] = {0x0E,0x11,0x11,0x0E,0x11,0x11,0x0E};
    static const unsigned char D9[] = {0x0E,0x11,0x11,0x0F,0x01,0x02,0x0C};
    switch (ch) {
    case 'G': return G; case 'A': return A; case 'Z': return Z;
    case 'E': return E; case 'O': return O; case 'N': return N;
    case 'F': return F; case 'P': return P; case 'S': return S;
    case ':': return C;
    case '0': return D0; case '1': return D1; case '2': return D2;
    case '3': return D3; case '4': return D4; case '5': return D5;
    case '6': return D6; case '7': return D7; case '8': return D8;
    case '9': return D9;
    default:  return SP;
    }
}

/* Rasterise `str` into a fresh RGBA texture. PX = pixels per font cell; text in
 * fg over a dark plaque background. Stores the (shared) pixel dims in R. */
static GLuint bake_label(const char *str, const float fg[3], int *ow, int *oh) {
    const int PX = 5, PAD = 6, GAP = 1;
    int n = (int)strlen(str);
    int tw = PAD*2 + n*GLYPH_W*PX + (n-1)*GAP*PX;
    int th = PAD*2 + GLYPH_H*PX;
    unsigned char *px = malloc((size_t)tw*th*4);
    const unsigned char bg[3] = {14, 18, 34};
    unsigned char fc[3] = { (unsigned char)(fg[0]*255), (unsigned char)(fg[1]*255),
                            (unsigned char)(fg[2]*255) };
    for (int i = 0; i < tw*th; i++) {
        px[i*4+0] = bg[0]; px[i*4+1] = bg[1]; px[i*4+2] = bg[2]; px[i*4+3] = 255;
    }
    for (int c = 0; c < n; c++) {
        const unsigned char *g = glyph_rows(str[c]);
        int ox = PAD + c*(GLYPH_W+GAP)*PX;
        for (int gy = 0; gy < GLYPH_H; gy++)
            for (int gx = 0; gx < GLYPH_W; gx++) {
                if (!(g[gy] & (1 << (GLYPH_W-1-gx)))) continue;
                for (int yy = 0; yy < PX; yy++)
                    for (int xx = 0; xx < PX; xx++) {
                        int x = ox + gx*PX + xx, y = PAD + gy*PX + yy;
                        unsigned char *p = &px[(y*tw + x)*4];
                        p[0] = fc[0]; p[1] = fc[1]; p[2] = fc[2]; p[3] = 255;
                    }
            }
    }
    GLuint tex; glGenTextures(1, &tex);
    glBindTexture(GL_TEXTURE_2D, tex);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, tw, th, 0, GL_RGBA, GL_UNSIGNED_BYTE, px);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
    free(px);
    if (ow) *ow = tw;
    if (oh) *oh = th;
    return tex;
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

    /* ground-plane program (position-only; shares attribute slot 0) */
    GLuint fv = compile(GL_VERTEX_SHADER, FLOOR_VERT);
    GLuint ff = compile(GL_FRAGMENT_SHADER, FLOOR_FRAG);
    if (!fv || !ff) return false;
    R.floor_prog = glCreateProgram();
    glAttachShader(R.floor_prog, fv);
    glAttachShader(R.floor_prog, ff);
    glBindAttribLocation(R.floor_prog, 0, "aPos");
    glLinkProgram(R.floor_prog);
    glGetProgramiv(R.floor_prog, GL_LINK_STATUS, &ok);
    if (!ok) {
        char log[1024]; glGetProgramInfoLog(R.floor_prog, sizeof log, NULL, log);
        fprintf(stderr, "render: floor link failed: %s\n", log); return false;
    }
    glDeleteShader(fv); glDeleteShader(ff);
    R.fMVP   = glGetUniformLocation(R.floor_prog, "uMVP");
    R.fGrass = glGetUniformLocation(R.floor_prog, "uGrass");
    R.fHaze  = glGetUniformLocation(R.floor_prog, "uHaze");
    R.fCell  = glGetUniformLocation(R.floor_prog, "uCell");

    /* shadow program (pos + uv; slots 0/1) */
    GLuint sv = compile(GL_VERTEX_SHADER, SHADOW_VERT);
    GLuint sf = compile(GL_FRAGMENT_SHADER, SHADOW_FRAG);
    if (!sv || !sf) return false;
    R.shadow_prog = glCreateProgram();
    glAttachShader(R.shadow_prog, sv);
    glAttachShader(R.shadow_prog, sf);
    glBindAttribLocation(R.shadow_prog, 0, "aPos");
    glBindAttribLocation(R.shadow_prog, 1, "aUV");
    glLinkProgram(R.shadow_prog);
    glGetProgramiv(R.shadow_prog, GL_LINK_STATUS, &ok);
    if (!ok) {
        char log[1024]; glGetProgramInfoLog(R.shadow_prog, sizeof log, NULL, log);
        fprintf(stderr, "render: shadow link failed: %s\n", log); return false;
    }
    glDeleteShader(sv); glDeleteShader(sf);
    R.shMVP      = glGetUniformLocation(R.shadow_prog, "uMVP");
    R.shStrength = glGetUniformLocation(R.shadow_prog, "uStrength");

    /* sky program (pos only; slot 0) */
    GLuint yv = compile(GL_VERTEX_SHADER, SKY_VERT);
    GLuint yf = compile(GL_FRAGMENT_SHADER, SKY_FRAG);
    if (!yv || !yf) return false;
    R.sky_prog = glCreateProgram();
    glAttachShader(R.sky_prog, yv);
    glAttachShader(R.sky_prog, yf);
    glBindAttribLocation(R.sky_prog, 0, "aPos");
    glLinkProgram(R.sky_prog);
    glGetProgramiv(R.sky_prog, GL_LINK_STATUS, &ok);
    if (!ok) {
        char log[1024]; glGetProgramInfoLog(R.sky_prog, sizeof log, NULL, log);
        fprintf(stderr, "render: sky link failed: %s\n", log); return false;
    }
    glDeleteShader(yv); glDeleteShader(yf);
    R.skMVP       = glGetUniformLocation(R.sky_prog, "uMVP");
    R.skIntensity = glGetUniformLocation(R.sky_prog, "uIntensity");

    /* terrain program (pos + normal; slots 0/1) */
    GLuint tv = compile(GL_VERTEX_SHADER, TERRAIN_VERT);
    GLuint tf = compile(GL_FRAGMENT_SHADER, TERRAIN_FRAG);
    if (!tv || !tf) return false;
    R.terrain_prog = glCreateProgram();
    glAttachShader(R.terrain_prog, tv);
    glAttachShader(R.terrain_prog, tf);
    glBindAttribLocation(R.terrain_prog, 0, "aPos");
    glBindAttribLocation(R.terrain_prog, 1, "aNormal");
    glBindAttribLocation(R.terrain_prog, 2, "aShade");
    glLinkProgram(R.terrain_prog);
    glGetProgramiv(R.terrain_prog, GL_LINK_STATUS, &ok);
    if (!ok) {
        char log[1024]; glGetProgramInfoLog(R.terrain_prog, sizeof log, NULL, log);
        fprintf(stderr, "render: terrain link failed: %s\n", log); return false;
    }
    glDeleteShader(tv); glDeleteShader(tf);
    R.tMVP  = glGetUniformLocation(R.terrain_prog, "uMVP");
    R.tSun  = glGetUniformLocation(R.terrain_prog, "uSun");
    R.tHaze = glGetUniformLocation(R.terrain_prog, "uHaze");
    R.tSnow = glGetUniformLocation(R.terrain_prog, "uSnow");
    R.tRock = glGetUniformLocation(R.terrain_prog, "uRock");

    glGenBuffers(1, &R.vbo);
    glBindBuffer(GL_ARRAY_BUFFER, R.vbo);
    glBufferData(GL_ARRAY_BUFFER, sizeof QUAD, QUAD, GL_STATIC_DRAW);

    if (m->cfg.floor_on)   build_floor_mesh(m);
    if (m->cfg.sky_on)     build_sky_dome();
    if (m->cfg.terrain_on) build_terrain();

    glEnable(GL_DEPTH_TEST);

    /* build one mesh per screen (flat quad or curved strip) */
    int nbuild = m->n_screen;
    if (nbuild < 0 || nbuild > MIRAGE_MAX_SCREENS) nbuild = 0;
    for (int i = 0; i < nbuild; i++) {
        if (m->cfg.geometry == GEOM_FLAT) build_flat_mesh(m, &m->screen[i]);
        else                              build_curved_mesh(m, &m->screen[i]);
        build_slab_mesh(m, &m->screen[i]);
        build_shadow_mesh(m, &m->screen[i], i);
    }

    /* gaze-mode status plaque (ON green, OFF grey) - same dims for both */
    { const float on[3]  = {0.31f, 0.90f, 0.47f};
      const float off[3] = {0.42f, 0.45f, 0.52f};
      R.label_on  = bake_label("GAZE: ON ", on, &R.label_w, &R.label_h);   /* trailing space: same dims */
      R.label_off = bake_label("GAZE: OFF", off, &R.label_w, &R.label_h); }
    R.fps_val = -1;   /* force the FPS plaque to bake on the first frame */

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
    struct timespec rt0; if (m->profile) clock_gettime(CLOCK_MONOTONIC, &rt0);

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
    mat4 proj = m4_perspective((m->cfg.fov_deg / z) * (float)M_PI/180.0f, aspect, 0.05f, 600.0f);

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

    /* Publish the look direction for the gaze cursor: this is the exact camera
     * orientation we render through (comfort gains + deadband + pan all baked
     * in), so grab.c can map "where the eye points" back to a screen + pixel.
     * Same yaw/pitch convention as layout_focus_angles, so its inverse lands
     * straight on the cursor strip. */
    {
        float groll;
        q_to_euler_ypr(head, &m->gaze_yaw, &m->gaze_pitch, &groll);
        m->gaze_have = true;
    }

    mat4 view = m4_from_quat(q_conj(head));   /* world -> head space */
    mat4 vp   = m4_mul(proj, view);

    /* sky dome first of all: additive, no depth (pure backdrop everything draws
     * over). The eye only rotates, so the origin-centred dome is a correct
     * infinite sky. */
    if (m->cfg.sky_on && R.sky_vbo) {
        glUseProgram(R.sky_prog);
        glUniformMatrix4fv(R.skMVP, 1, GL_FALSE, vp.m);
        glUniform1f(R.skIntensity, SKY_INTENSITY);
        glBindBuffer(GL_ARRAY_BUFFER, R.sky_vbo);
        glEnableVertexAttribArray(R.aPos);
        glVertexAttribPointer(R.aPos, 3, GL_FLOAT, GL_FALSE, 3*sizeof(GLfloat), (void*)0);
        glDisableVertexAttribArray(R.aUV);
        glDisable(GL_DEPTH_TEST);
        glDepthMask(GL_FALSE);
        glEnable(GL_BLEND);
        glBlendFunc(GL_ONE, GL_ONE);
        glDrawArrays(GL_TRIANGLES, 0, R.sky_verts);
        glDisable(GL_BLEND);
        glDepthMask(GL_TRUE);
        glEnable(GL_DEPTH_TEST);
    }

    /* terrain: opaque, depth-tested. The mountain landscape below; dark valleys
     * read as transparent on the optics, lit slopes and snow glow. */
    if (m->cfg.terrain_on && R.terrain_vbo) {
        glUseProgram(R.terrain_prog);
        glUniformMatrix4fv(R.tMVP, 1, GL_FALSE, vp.m);
        glUniform3f(R.tSun, TERR_SUN.x, TERR_SUN.y, TERR_SUN.z);
        glUniform3f(R.tHaze, TERR_HAZE[0], TERR_HAZE[1], TERR_HAZE[2]);
        glUniform1f(R.tSnow, TERR_SNOW_Y);
        glUniform1f(R.tRock, TERR_ROCK_Y);
        glBindBuffer(GL_ARRAY_BUFFER, R.terrain_vbo);
        glEnableVertexAttribArray(R.aPos);
        glVertexAttribPointer(R.aPos, 3, GL_FLOAT, GL_FALSE, 8*sizeof(GLfloat), (void*)0);
        glEnableVertexAttribArray(R.aUV);   /* slot 1 = aNormal here */
        glVertexAttribPointer(R.aUV, 3, GL_FLOAT, GL_FALSE, 8*sizeof(GLfloat), (void*)(3*sizeof(GLfloat)));
        glEnableVertexAttribArray(2);       /* slot 2 = aShade (baked shadow, AO) */
        glVertexAttribPointer(2, 2, GL_FLOAT, GL_FALSE, 8*sizeof(GLfloat), (void*)(6*sizeof(GLfloat)));
        glDrawArrays(GL_TRIANGLES, 0, R.terrain_verts);
        glDisableVertexAttribArray(2);
    }

    /* ground plane next: additive emissive (adds light over the see-through),
     * depth-written so later geometry/shadows sit correctly against it. */
    if (m->cfg.floor_on && R.floor_vbo) {
        glUseProgram(R.floor_prog);
        glUniformMatrix4fv(R.fMVP, 1, GL_FALSE, vp.m);
        glUniform3f(R.fGrass, FLOOR_GRASS[0], FLOOR_GRASS[1], FLOOR_GRASS[2]);
        glUniform3f(R.fHaze, FLOOR_HAZE[0], FLOOR_HAZE[1], FLOOR_HAZE[2]);
        glUniform1f(R.fCell, FLOOR_CELL);
        glBindBuffer(GL_ARRAY_BUFFER, R.floor_vbo);
        glEnableVertexAttribArray(R.aPos);
        glVertexAttribPointer(R.aPos, 3, GL_FLOAT, GL_FALSE, 3*sizeof(GLfloat), (void*)0);
        glDisableVertexAttribArray(R.aUV);
        glEnable(GL_BLEND);
        glBlendFunc(GL_ONE, GL_ONE);
        glDrawArrays(GL_TRIANGLE_STRIP, 0, 4);
        glDisable(GL_BLEND);
    }

    int n = m->n_screen > 0 ? m->n_screen : m->cfg.screen_count;
    if (n > MIRAGE_MAX_SCREENS) n = MIRAGE_MAX_SCREENS;

    /* drop shadows onto the floor: multiply the floor glow down (dst*(1-a)).
     * Depth-tested so slabs in front occlude them, but no depth write so they
     * don't block each other or the panels. */
    if (m->cfg.floor_on && m->cfg.shadows_on) {
        glUseProgram(R.shadow_prog);
        glUniform1f(R.shStrength, SHADOW_STRENGTH);
        glEnable(GL_BLEND);
        glBlendFunc(GL_ZERO, GL_ONE_MINUS_SRC_ALPHA);
        glDepthMask(GL_FALSE);
        for (int i = 0; i < n; i++) {
            screen_t *s = &m->screen[i];
            if (!s->shadow_vbo) continue;
            mat4 mvp = vp;                       /* shadow verts are world-space */
            glUniformMatrix4fv(R.shMVP, 1, GL_FALSE, mvp.m);
            glBindBuffer(GL_ARRAY_BUFFER, s->shadow_vbo);
            glEnableVertexAttribArray(R.aPos);
            glVertexAttribPointer(R.aPos, 3, GL_FLOAT, GL_FALSE, 5*sizeof(GLfloat), (void*)0);
            glEnableVertexAttribArray(R.aUV);
            glVertexAttribPointer(R.aUV, 2, GL_FLOAT, GL_FALSE, 5*sizeof(GLfloat), (void*)(3*sizeof(GLfloat)));
            glDrawArrays(GL_TRIANGLE_STRIP, 0, 4);
        }
        glDepthMask(GL_TRUE);
        glDisable(GL_BLEND);
    }

    glUseProgram(R.prog);
    for (int i = 0; i < n; i++) {
        screen_t *s = &m->screen[i];
        if (!s->mesh_vbo) continue;

        mat4 model = layout_model_matrix(m, i);
        mat4 mvp   = m4_mul(vp, model);
        glUniformMatrix4fv(R.uMVP, 1, GL_FALSE, mvp.m);

        /* slab body: the 5 non-front faces, each a solid colour lit by the fixed
         * key light. Drawn first; the textured front face below sits in front of
         * it (same MVP - the box is in the same local space as the panel). */
        if (s->slab_vbo) {
            glBindBuffer(GL_ARRAY_BUFFER, s->slab_vbo);
            glEnableVertexAttribArray(R.aPos);
            glVertexAttribPointer(R.aPos, 3, GL_FLOAT, GL_FALSE, 5*sizeof(GLfloat), (void*)0);
            glEnableVertexAttribArray(R.aUV);
            glVertexAttribPointer(R.aUV, 2, GL_FLOAT, GL_FALSE, 5*sizeof(GLfloat), (void*)(3*sizeof(GLfloat)));
            glUniform1f(R.uHasTex, 0.0f);
            glUniform1f(R.uYFlip, 0.0f);
            for (int f = 0; f < 5; f++) {
                vec3 wn = m4_dir(&model, SLAB_N[f]);
                float diff = wn.x*SLAB_LIGHT.x + wn.y*SLAB_LIGHT.y + wn.z*SLAB_LIGHT.z;
                if (diff < 0.0f) diff = 0.0f;
                float b = SLAB_AMBIENT + (1.0f - SLAB_AMBIENT) * diff;
                glUniform3f(R.uColor, SLAB_TINT[0]*b, SLAB_TINT[1]*b, SLAB_TINT[2]*b);
                glDrawArrays(GL_TRIANGLES, f*6, 6);
            }
        }

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

    /* Gaze-mode status plaque, floating just under the centre-column screen and
     * pinned to its frame (so it tracks the wall as you pan). Picks the ON/OFF
     * texture from the live gaze_cursor flag (toggled by double-Cmd in grab.c). */
    {
        int cols = m->cfg.screen_cols > 0 ? m->cfg.screen_cols : 3;
        int ci   = (cols - 1) / 2;               /* centre column, bottom row */
        if (ci < n && R.label_on) {
            screen_t *cs = &m->screen[ci];
            float d      = m->cfg.screen_distance_m;
            float ang_w  = m->cfg.screen_arc_deg * (float)M_PI/180.0f;
            float aspect = (cs->width > 0 && cs->height > 0)
                           ? (float)cs->height / (float)cs->width : 9.0f/16.0f;
            float hh     = d * tanf(ang_w * 0.5f) * aspect;   /* screen half-height */
            float fullH  = 0.11f;
            float fullW  = fullH * ((float)R.label_w / (float)R.label_h);
            float yc     = -hh - 0.05f - fullH * 0.5f;        /* just below the edge */

            mat4 local = m4_mul(m4_translate(v3(0.0f, yc, -d)),
                                m4_scale(v3(fullW, fullH, 1.0f)));
            mat4 model = m4_mul(layout_model_matrix(m, ci), local);
            mat4 mvp   = m4_mul(vp, model);
            glUniformMatrix4fv(R.uMVP, 1, GL_FALSE, mvp.m);
            glBindBuffer(GL_ARRAY_BUFFER, R.vbo);
            glEnableVertexAttribArray(R.aPos);
            glVertexAttribPointer(R.aPos, 3, GL_FLOAT, GL_FALSE, 5*sizeof(GLfloat), (void*)0);
            glEnableVertexAttribArray(R.aUV);
            glVertexAttribPointer(R.aUV, 2, GL_FLOAT, GL_FALSE, 5*sizeof(GLfloat), (void*)(3*sizeof(GLfloat)));
            glActiveTexture(GL_TEXTURE0);
            glBindTexture(GL_TEXTURE_2D, m->cfg.gaze_cursor ? R.label_on : R.label_off);
            glUniform1i(R.uTex, 0);
            glUniform1f(R.uHasTex, 1.0f);
            glUniform1f(R.uYFlip, 0.0f);
            glUniform1f(R.uSharpen, 0.0f);
            glDrawArrays(GL_TRIANGLE_STRIP, 0, 4);

            /* FPS counter, one row below the GAZE plaque. Re-bake the digits only
             * when the integer value changes (~once a second); the vbo + vertex
             * attribs are still bound from the GAZE draw above, so we just swap the
             * texture and the MVP. */
            int fv = (int)(m->fps + 0.5f);
            if (fv < 0)   fv = 0;
            if (fv > 999) fv = 999;
            if (fv != R.fps_val) {
                if (R.label_fps) glDeleteTextures(1, &R.label_fps);
                char buf[16]; snprintf(buf, sizeof buf, "FPS %d", fv);
                const float fc[3] = {0.62f, 0.78f, 0.95f};   /* cool blue */
                R.label_fps = bake_label(buf, fc, &R.fps_w, &R.fps_h);
                R.fps_val = fv;
            }
            if (R.label_fps) {
                float fpsW  = fullH * ((float)R.fps_w / (float)R.fps_h);
                float yc2   = yc - fullH - 0.02f;            /* one row lower */
                mat4 local2 = m4_mul(m4_translate(v3(0.0f, yc2, -d)),
                                     m4_scale(v3(fpsW, fullH, 1.0f)));
                mat4 model2 = m4_mul(layout_model_matrix(m, ci), local2);
                mat4 mvp2   = m4_mul(vp, model2);
                glUniformMatrix4fv(R.uMVP, 1, GL_FALSE, mvp2.m);
                glBindTexture(GL_TEXTURE_2D, R.label_fps);
                glUniform1i(R.uTex, 0);
                glUniform1f(R.uHasTex, 1.0f);
                glUniform1f(R.uYFlip, 0.0f);
                glUniform1f(R.uSharpen, 0.0f);
                glDrawArrays(GL_TRIANGLE_STRIP, 0, 4);
            }
        }
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

    if (m->profile) {
        /* glFinish drains the GPU so rt0->tg is pure draw cost (texture sampling
         * included); tg->ts is the present wait. High gpu = render/sampling bound;
         * high swap = compositor/present bound (scanout not engaging). glFinish is
         * harmful in production - only on under MIRAGE_PROFILE. */
        struct timespec tg, ts;
        glFinish();
        clock_gettime(CLOCK_MONOTONIC, &tg);
        eglSwapBuffers(m->edpy, m->esurf);
        clock_gettime(CLOCK_MONOTONIC, &ts);
        m->prof_gpu_ms  = prof_ms(rt0, tg);
        m->prof_swap_ms = prof_ms(tg, ts);
    } else {
        eglSwapBuffers(m->edpy, m->esurf);
    }
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
