#include "mirage.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

#include <wayland-egl.h>

/* profiling timing helper: milliseconds between two monotonic samples. */
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

/* HDRI environment dome: a sphere of world directions around the eye. The vertex
 * position doubles as the look direction; the fragment samples an equirectangular
 * HDRI by that direction, applies exposure + an ACES-ish tonemap, and scales by an
 * intensity. The texture is sqrt-encoded (see load_hdri_rgb8) so it's squared back
 * to linear here. Drawn additively, so dark sky adds nothing on the see-through
 * optics and only the stars glow. */
static const char *DOME_VERT =
    "attribute vec3 aPos;\n"
    "uniform mat4 uMVP;\n"
    "varying highp vec3 vDir;\n"
    "void main() { gl_Position = uMVP * vec4(aPos, 1.0); vDir = aPos; }\n";
static const char *DOME_FRAG =
    "precision highp float;\n"
    "varying highp vec3 vDir;\n"
    "uniform sampler2D uTex;\n"
    "uniform float uExposure;\n"
    "uniform float uIntensity;\n"
    "uniform float uBlack;\n"       /* black point: floor below this -> 0 (true black) */
    "uniform float uSaturation;\n"  /* chroma boost around luma */
    "void main() {\n"
    "  vec3 d = normalize(vDir);\n"
    "  float u = atan(d.x, -d.z) * 0.159154943 + 0.5;\n"   /* 1/(2pi) */
    "  float v = 0.5 - asin(clamp(d.y, -1.0, 1.0)) * 0.318309886;\n"  /* 1/pi */
    "  vec3 c = texture2D(uTex, vec2(u, v)).rgb;\n"
    "  c = c * c;\n"                                       /* decode sqrt -> linear */
    "  c = max(c - uBlack, 0.0) * uExposure;\n"            /* crush the haze floor to black, then expose */
    "  float l = dot(c, vec3(0.2126, 0.7152, 0.0722));\n"
    "  c = mix(vec3(l), c, uSaturation);\n"                /* punch up star colour */
    "  gl_FragColor = vec4(clamp(c, 0.0, 1.0) * uIntensity, 1.0);\n"  /* no filmic lift; let bright stars clip white */
    "}\n";

static struct {
    GLuint prog;
    GLint  aPos, aUV;
    GLint  uMVP, uYFlip, uHasTex, uColor, uTex, uTexel, uSharpen;
    GLuint vbo;

    /* HDRI environment dome */
    GLuint dome_prog, dome_vbo, hdri_tex;
    int    dome_verts;
    GLint  dMVP, dExposure, dIntensity, dBlack, dSat, dTex;

    /* "GAZE: ON/OFF" status plaque below the centre screen (baked text textures,
     * drawn on the unit QUAD via R.prog). */
    GLuint label_on, label_off;
    int    label_w, label_h;
    /* live FPS plaque: re-baked only when the integer value changes */
    GLuint label_fps;
    int    fps_w, fps_h, fps_val;
    /* static multi-line shortcut cheat-sheet, baked once at init */
    GLuint label_help;
    int    help_w, help_h;
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
    float ang = s->arc_deg * (float)M_PI/180.0f;
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
    float ang_w  = s->arc_deg * (float)M_PI/180.0f;
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

/* Box around the front panel, extruded back by slab_depth_m. Front half-extents
 * match the textured mesh so the slab wraps it (curved strips bulge a touch
 * proud of the front rim - reads fine). 5 faces x 2 tris, interleaved x,y,z,u,v
 * (uv unused). No-op when slab depth is 0 (flat panels). */
static void build_slab_mesh(struct mirage *m, screen_t *s) {
    const mirage_config *c = &m->cfg;
    if (c->slab_depth_m <= 0.0f) { s->slab_vbo = 0; s->slab_verts = 0; return; }
    float d      = c->screen_distance_m;
    float ang_w  = s->arc_deg * (float)M_PI/180.0f;
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

/* ---- status plaque text ---------------------------------------------------
 * A 5x7 bitmap font, just the glyphs the "GAZE: ON/OFF" label needs. Each glyph
 * is 7 rows; the low 5 bits of each byte are the pixels, MSB (bit 4) leftmost.
 * We rasterise a whole string into an RGBA texture once at init (two of them:
 * ON in green, OFF in grey), then draw it on a quad below the centre screen. */
#define GLYPH_W 5
#define GLYPH_H 7
static const unsigned char *glyph_rows(char ch) {
    static const unsigned char A[] = {0x0E,0x11,0x11,0x1F,0x11,0x11,0x11};
    static const unsigned char B[] = {0x1E,0x11,0x11,0x1E,0x11,0x11,0x1E};
    static const unsigned char C[] = {0x0E,0x11,0x10,0x10,0x10,0x11,0x0E};
    static const unsigned char D[] = {0x1E,0x11,0x11,0x11,0x11,0x11,0x1E};
    static const unsigned char E[] = {0x1F,0x10,0x10,0x1E,0x10,0x10,0x1F};
    static const unsigned char F[] = {0x1F,0x10,0x10,0x1E,0x10,0x10,0x10};
    static const unsigned char G[] = {0x0E,0x11,0x10,0x17,0x11,0x11,0x0E};
    static const unsigned char H[] = {0x11,0x11,0x11,0x1F,0x11,0x11,0x11};
    static const unsigned char I[] = {0x0E,0x04,0x04,0x04,0x04,0x04,0x0E};
    static const unsigned char J[] = {0x07,0x02,0x02,0x02,0x02,0x12,0x0C};
    static const unsigned char K[] = {0x11,0x12,0x14,0x18,0x14,0x12,0x11};
    static const unsigned char L[] = {0x10,0x10,0x10,0x10,0x10,0x10,0x1F};
    static const unsigned char M[] = {0x11,0x1B,0x15,0x15,0x11,0x11,0x11};
    static const unsigned char N[] = {0x11,0x19,0x15,0x13,0x11,0x11,0x11};
    static const unsigned char O[] = {0x0E,0x11,0x11,0x11,0x11,0x11,0x0E};
    static const unsigned char P[] = {0x1E,0x11,0x11,0x1E,0x10,0x10,0x10};
    static const unsigned char Q[] = {0x0E,0x11,0x11,0x11,0x15,0x12,0x0D};
    static const unsigned char R[] = {0x1E,0x11,0x11,0x1E,0x14,0x12,0x11};
    static const unsigned char S[] = {0x0F,0x10,0x10,0x0E,0x01,0x01,0x1E};
    static const unsigned char T[] = {0x1F,0x04,0x04,0x04,0x04,0x04,0x04};
    static const unsigned char U[] = {0x11,0x11,0x11,0x11,0x11,0x11,0x0E};
    static const unsigned char V[] = {0x11,0x11,0x11,0x11,0x11,0x0A,0x04};
    static const unsigned char W[] = {0x11,0x11,0x11,0x15,0x15,0x15,0x0A};
    static const unsigned char X[] = {0x11,0x11,0x0A,0x04,0x0A,0x11,0x11};
    static const unsigned char Y[] = {0x11,0x11,0x0A,0x04,0x04,0x04,0x04};
    static const unsigned char Z[] = {0x1F,0x01,0x02,0x04,0x08,0x10,0x1F};
    static const unsigned char CO[] = {0x00,0x04,0x04,0x00,0x04,0x04,0x00}; /* colon */
    static const unsigned char PL[] = {0x00,0x04,0x04,0x1F,0x04,0x04,0x00}; /* plus  */
    static const unsigned char MI[] = {0x00,0x00,0x00,0x1F,0x00,0x00,0x00}; /* minus */
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
    case 'A': return A; case 'B': return B; case 'C': return C;
    case 'D': return D; case 'E': return E; case 'F': return F;
    case 'G': return G; case 'H': return H; case 'I': return I;
    case 'J': return J; case 'K': return K; case 'L': return L;
    case 'M': return M; case 'N': return N; case 'O': return O;
    case 'P': return P; case 'Q': return Q; case 'R': return R;
    case 'S': return S; case 'T': return T; case 'U': return U;
    case 'V': return V; case 'W': return W; case 'X': return X;
    case 'Y': return Y; case 'Z': return Z;
    case ':': return CO; case '+': return PL; case '-': return MI;
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
    const int PX = 5, PAD = 6, GAP = 1, LGAP = 3;   /* LGAP: blank font rows between lines */
    /* Measure: tallest is the line count, widest is the longest line (chars). */
    int lines = 1, cur = 0, maxlen = 0;
    for (const char *p = str; *p; p++) {
        if (*p == '\n') { if (cur > maxlen) maxlen = cur; cur = 0; lines++; }
        else cur++;
    }
    if (cur > maxlen) maxlen = cur;
    int tw = PAD*2 + maxlen*GLYPH_W*PX + (maxlen > 0 ? maxlen-1 : 0)*GAP*PX;
    int th = PAD*2 + lines*GLYPH_H*PX + (lines-1)*LGAP*PX;
    unsigned char *px = malloc((size_t)tw*th*4);
    const unsigned char bg[3] = {14, 18, 34};
    unsigned char fc[3] = { (unsigned char)(fg[0]*255), (unsigned char)(fg[1]*255),
                            (unsigned char)(fg[2]*255) };
    for (int i = 0; i < tw*th; i++) {
        px[i*4+0] = bg[0]; px[i*4+1] = bg[1]; px[i*4+2] = bg[2]; px[i*4+3] = 255;
    }
    int line = 0, col = 0;
    for (const char *p = str; *p; p++) {
        if (*p == '\n') { line++; col = 0; continue; }
        const unsigned char *g = glyph_rows(*p);
        int ox = PAD + col*(GLYPH_W+GAP)*PX;
        int oy = PAD + line*(GLYPH_H+LGAP)*PX;
        for (int gy = 0; gy < GLYPH_H; gy++)
            for (int gx = 0; gx < GLYPH_W; gx++) {
                if (!(g[gy] & (1 << (GLYPH_W-1-gx)))) continue;
                for (int yy = 0; yy < PX; yy++)
                    for (int xx = 0; xx < PX; xx++) {
                        int x = ox + gx*PX + xx, y = oy + gy*PX + yy;
                        unsigned char *p2 = &px[(y*tw + x)*4];
                        p2[0] = fc[0]; p2[1] = fc[1]; p2[2] = fc[2]; p2[3] = 255;
                    }
            }
        col++;
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

/* ---- HDRI environment dome ---------------------------------------------------
 * Load a FLAT (non-RLE) Radiance .hdr - the format hdri/exr2hdr.py writes - into an
 * RGB8 buffer. Values are clamped to [0,1] and sqrt-encoded so the 8-bit texture
 * keeps precision in the dark range (faint stars); the dome shader squares it back
 * to linear. Returns a malloc'd w*h*3 buffer (caller frees) or NULL on failure. */
static unsigned char *load_hdri_rgb8(const char *path, int *ow, int *oh) {
    FILE *f = fopen(path, "rb");
    if (!f) { fprintf(stderr, "hdri: cannot open %s\n", path); return NULL; }
    char line[256];
    int w = 0, h = 0;
    while (fgets(line, sizeof line, f))         /* skip header to the "-Y h +X w" line */
        if (line[0] == '-' && (line[1] == 'Y' || line[1] == 'y')) {
            sscanf(line, "-Y %d +X %d", &h, &w); break;
        }
    if (w <= 0 || h <= 0) { fprintf(stderr, "hdri: bad/RLE .hdr %s\n", path); fclose(f); return NULL; }
    size_t n = (size_t)w * h;
    unsigned char *rgbe = malloc(n * 4), *out = malloc(n * 3);
    if (!rgbe || !out || fread(rgbe, 4, n, f) != n) {
        fprintf(stderr, "hdri: short read %s\n", path);
        free(rgbe); free(out); fclose(f); return NULL;
    }
    fclose(f);
    for (size_t i = 0; i < n; i++) {
        int e = rgbe[i*4 + 3];
        float scale = e ? ldexpf(1.0f, e - 136) : 0.0f;   /* RGBE: 2^(e-128)/256 */
        for (int c = 0; c < 3; c++) {
            float v = (float)rgbe[i*4 + c] * scale;
            v = v < 0.0f ? 0.0f : (v > 1.0f ? 1.0f : v);
            out[i*3 + c] = (unsigned char)(sqrtf(v) * 255.0f + 0.5f);
        }
    }
    free(rgbe);
    *ow = w; *oh = h;
    return out;
}

/* A full UV sphere of unit directions (radius scaled large), centred on the eye.
 * Position is reused as the look direction in DOME_FRAG. */
static void build_dome(void) {
    const int NLAT = 32, NLON = 64;
    const float Rr = 50.0f;
    int verts = NLAT * NLON * 6;
    GLfloat *buf = malloc((size_t)verts * 3 * sizeof(GLfloat));
    int k = 0;
    for (int i = 0; i < NLAT; i++) {
        float a0 = -(float)M_PI/2 + (float)M_PI * i     / NLAT;
        float a1 = -(float)M_PI/2 + (float)M_PI * (i+1) / NLAT;
        for (int j = 0; j < NLON; j++) {
            float o0 = 2.0f*(float)M_PI * j     / NLON;
            float o1 = 2.0f*(float)M_PI * (j+1) / NLON;
            #define DV(LAT,LON) do { \
                buf[k++] =  Rr*cosf(LAT)*sinf(LON); \
                buf[k++] =  Rr*sinf(LAT); \
                buf[k++] = -Rr*cosf(LAT)*cosf(LON); } while (0)
            DV(a0,o0); DV(a1,o0); DV(a1,o1);
            DV(a0,o0); DV(a1,o1); DV(a0,o1);
            #undef DV
        }
    }
    glGenBuffers(1, &R.dome_vbo);
    glBindBuffer(GL_ARRAY_BUFFER, R.dome_vbo);
    glBufferData(GL_ARRAY_BUFFER, (GLsizeiptr)(verts*3*sizeof(GLfloat)), buf, GL_STATIC_DRAW);
    R.dome_verts = verts;
    free(buf);
}

/* Build the dome program, load the HDRI into a texture, and build the sphere.
 * Disables the dome (R.dome_prog stays 0) if anything fails, so render_frame skips it. */
static void hdri_init(struct mirage *m) {
    if (!m->cfg.hdri_on) return;
    int w, h;
    unsigned char *px = load_hdri_rgb8(m->cfg.hdri_path, &w, &h);
    if (!px) { fprintf(stderr, "hdri: disabled (load failed)\n"); return; }

    glGenTextures(1, &R.hdri_tex);
    glBindTexture(GL_TEXTURE_2D, R.hdri_tex);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, w, h, 0, GL_RGB, GL_UNSIGNED_BYTE, px);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);          /* equirect wraps in u */
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
    free(px);

    GLuint vs = compile(GL_VERTEX_SHADER, DOME_VERT);
    GLuint fs = compile(GL_FRAGMENT_SHADER, DOME_FRAG);
    if (!vs || !fs) return;
    R.dome_prog = glCreateProgram();
    glAttachShader(R.dome_prog, vs);
    glAttachShader(R.dome_prog, fs);
    glBindAttribLocation(R.dome_prog, 0, "aPos");
    glLinkProgram(R.dome_prog);
    GLint ok = 0; glGetProgramiv(R.dome_prog, GL_LINK_STATUS, &ok);
    if (!ok) { fprintf(stderr, "hdri: dome link failed\n"); R.dome_prog = 0; return; }
    glDeleteShader(vs); glDeleteShader(fs);
    R.dMVP       = glGetUniformLocation(R.dome_prog, "uMVP");
    R.dExposure  = glGetUniformLocation(R.dome_prog, "uExposure");
    R.dIntensity = glGetUniformLocation(R.dome_prog, "uIntensity");
    R.dBlack     = glGetUniformLocation(R.dome_prog, "uBlack");
    R.dSat       = glGetUniformLocation(R.dome_prog, "uSaturation");
    R.dTex       = glGetUniformLocation(R.dome_prog, "uTex");
    build_dome();
    fprintf(stderr, "hdri: dome ready (%dx%d, %s)\n", w, h, m->cfg.hdri_path);
}

bool render_init(struct mirage *m) {
    EGLint major = 0, minor = 0;   /* EGL version, logged below */
    m->edpy = eglGetDisplay((EGLNativeDisplayType)m->display);
    if (m->edpy == EGL_NO_DISPLAY) { fprintf(stderr, "render: no EGL display\n"); return false; }
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
    /* Present every vblank with vsync on. glasses.sh sets the panel mode, so one
     * vblank = the panel period and this hardware-locks us to the refresh (GPU draw
     * is <1ms, so every frame lands with time to spare). NB interval 2 ("every 2nd
     * vblank") is NOT honored on the Hyprland direct-scanout path - the panel mode
     * is what actually fixes the rate. */
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

    glGenBuffers(1, &R.vbo);
    glBindBuffer(GL_ARRAY_BUFFER, R.vbo);
    glBufferData(GL_ARRAY_BUFFER, sizeof QUAD, QUAD, GL_STATIC_DRAW);

    glEnable(GL_DEPTH_TEST);

    /* build one mesh per screen (flat quad or curved strip) + its slab body.
     * Each screen's angular width is its per-screen override (cfg.screen_arc[i]),
     * falling back to the default - so the wide wall and the narrower 16:9 above
     * it each get their own arc. */
    int nbuild = m->n_screen;
    if (nbuild < 0 || nbuild > MIRAGE_MAX_SCREENS) nbuild = 0;
    for (int i = 0; i < nbuild; i++) {
        m->screen[i].arc_deg = m->cfg.screen_arc[i] > 0.0f
                               ? m->cfg.screen_arc[i] : m->cfg.screen_arc_deg;
        if (m->cfg.geometry == GEOM_FLAT) build_flat_mesh(m, &m->screen[i]);
        else                              build_curved_mesh(m, &m->screen[i]);
        build_slab_mesh(m, &m->screen[i]);
    }

    /* gaze-mode status plaque (ON green, OFF grey) - same dims for both */
    { const float on[3]  = {0.31f, 0.90f, 0.47f};
      const float off[3] = {0.42f, 0.45f, 0.52f};
      R.label_on  = bake_label("GAZE: ON ", on, &R.label_w, &R.label_h);   /* trailing space: same dims */
      R.label_off = bake_label("GAZE: OFF", off, &R.label_w, &R.label_h); }
    R.fps_val = -1;   /* force the FPS plaque to bake on the first frame */
    /* static shortcut cheat-sheet, one multi-line plaque baked once */
    { const float hc[3] = {0.66f, 0.72f, 0.82f};
      R.label_help = bake_label(
          "2X CMD: RECENTER\n"
          "2X ALT: GAZE\n"
          "CMD+SCROLL: ZOOM\n"
          "3-SWIPE: WORKSPACE\n"
          "SUPER+SHIFT+Q: QUIT",
          hc, &R.help_w, &R.help_h); }

    hdri_init(m);   /* environment dome (no-op if cfg.hdri_on is false or load fails) */

    fprintf(stderr, "render: EGL %d.%d, GL_RENDERER=%s\n", major, minor,
            (const char*)glGetString(GL_RENDERER));
    return true;
}

/* placeholder tints for screens with no capture yet */
static const float PLACEHOLDER[][3] = {
    {0.20f, 0.10f, 0.30f}, {0.10f, 0.25f, 0.20f}, {0.28f, 0.18f, 0.08f},
    {0.10f, 0.18f, 0.30f}, {0.25f, 0.10f, 0.15f},
};

void render_frame(struct mirage *m, quat head) {
    eglMakeCurrent(m->edpy, m->esurf, m->esurf, m->ectx);
    struct timespec rt0; if (m->profile) clock_gettime(CLOCK_MONOTONIC, &rt0);

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

    /* Publish the look direction for the gaze cursor: this is the exact camera
     * orientation we render through (comfort gains + deadband baked
     * in), so grab.c can map "where the eye points" back to a screen + pixel.
     * Same yaw/pitch convention as layout_focus_angles, so its inverse lands
     * straight on the cursor strip. */
    {
        float groll;
        q_to_euler_ypr(head, &m->gaze_yaw, &m->gaze_pitch, &groll);
        m->gaze_have = true;
    }

    /* Neck model: the eye sits ahead of and above the neck pivot, so a head turn
     * sweeps the eye through an arc -> real translation -> motion parallax (near
     * windows shift against far ones and the fixed star dome). With the camera
     * pinned at the origin this is the only translational depth cue we get from
     * the 3DoF stream. eye_world rotates the fixed local offset by the presented
     * head, so the parallax stays consistent with the orientation we render. */
    vec3 eye_world = q_rotate(head, v3(0.0f, m->cfg.neck_up_m, -m->cfg.neck_fwd_m));
    mat4 view = m4_mul(m4_from_quat(q_conj(head)),     /* world -> head rotation */
                       m4_translate(v3_scale(eye_world, -1.0f)));  /* then -eye  */
    mat4 vp   = m4_mul(proj, view);

    /* HDRI environment dome: drawn first as an infinite, world-fixed backdrop.
     * Additive (dark sky adds nothing on the optics), depth test + write off so the
     * wall and slabs draw cleanly over it. The dome sphere (radius 50 m) dwarfs the
     * neck-model eye shift (~0.1 m), so the stars stay effectively fixed in world
     * space - the far reference the near windows parallax against as you look around. */
    if (R.dome_prog && R.dome_vbo) {
        glUseProgram(R.dome_prog);
        glUniformMatrix4fv(R.dMVP, 1, GL_FALSE, vp.m);
        glUniform1f(R.dExposure,  m->cfg.hdri_exposure);
        glUniform1f(R.dIntensity, m->cfg.hdri_intensity);
        glUniform1f(R.dBlack,     m->cfg.hdri_black);
        glUniform1f(R.dSat,       m->cfg.hdri_saturation);
        glActiveTexture(GL_TEXTURE0);
        glBindTexture(GL_TEXTURE_2D, R.hdri_tex);
        glUniform1i(R.dTex, 0);
        glBindBuffer(GL_ARRAY_BUFFER, R.dome_vbo);
        glEnableVertexAttribArray(R.aPos);
        glVertexAttribPointer(R.aPos, 3, GL_FLOAT, GL_FALSE, 3*sizeof(GLfloat), (void*)0);
        glDisableVertexAttribArray(R.aUV);
        glDisable(GL_DEPTH_TEST);
        glDepthMask(GL_FALSE);
        glEnable(GL_BLEND);
        glBlendFunc(GL_ONE, GL_ONE);
        glDrawArrays(GL_TRIANGLES, 0, R.dome_verts);
        glDisable(GL_BLEND);
        glDepthMask(GL_TRUE);
        glEnable(GL_DEPTH_TEST);
    }

    int n = m->n_screen > 0 ? m->n_screen : m->cfg.screen_count;
    if (n > MIRAGE_MAX_SCREENS) n = MIRAGE_MAX_SCREENS;

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
        /* centre-column, bottom screen: smallest |yaw|, then lowest lift */
        int ci = -1; float best_yaw = 1e30f, best_lift = 1e30f;
        for (int k = 0; k < n; k++) {
            float yw, lf, ar; layout_place(m, k, &yw, &lf, &ar);
            float ay = fabsf(yw);
            if (ay < best_yaw - 1e-4f ||
                (ay < best_yaw + 1e-4f && lf < best_lift)) {
                best_yaw = ay; best_lift = lf; ci = k;
            }
        }
        if (ci >= 0 && ci < n && R.label_on) {
            screen_t *cs = &m->screen[ci];
            float d      = m->cfg.screen_distance_m;
            float ang_w  = cs->arc_deg * (float)M_PI/180.0f;
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

            /* Shortcut cheat-sheet, stacked below the FPS row. Static texture,
             * drawn at a smaller scale so the five lines don't hang too far down. */
            if (R.label_help) {
                float blockH = 0.26f;                            /* whole block height */
                float blockW = blockH * ((float)R.help_w / (float)R.help_h);
                float fpsBot = (yc - fullH - 0.02f) - fullH * 0.5f;  /* FPS plaque bottom */
                float yc3    = fpsBot - 0.03f - blockH * 0.5f;
                mat4 local3  = m4_mul(m4_translate(v3(0.0f, yc3, -d)),
                                      m4_scale(v3(blockW, blockH, 1.0f)));
                mat4 model3  = m4_mul(layout_model_matrix(m, ci), local3);
                mat4 mvp3    = m4_mul(vp, model3);
                glUniformMatrix4fv(R.uMVP, 1, GL_FALSE, mvp3.m);
                glBindTexture(GL_TEXTURE_2D, R.label_help);
                glUniform1i(R.uTex, 0);
                glUniform1f(R.uHasTex, 1.0f);
                glUniform1f(R.uYFlip, 0.0f);
                glUniform1f(R.uSharpen, 0.0f);
                glDrawArrays(GL_TRIANGLE_STRIP, 0, 4);
            }
        }
    }

    if (m->profile) {
        /* glFinish drains the GPU so rt0->tg is pure draw cost (texture sampling
         * included); tg->ts is the present wait. High gpu = render/sampling bound;
         * high swap = compositor/present bound (scanout not engaging). glFinish is
         * harmful in production - only on under profiling. */
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

void render_finish(struct mirage *m) {
    if (m->edpy == EGL_NO_DISPLAY) return;
    eglMakeCurrent(m->edpy, EGL_NO_SURFACE, EGL_NO_SURFACE, EGL_NO_CONTEXT);
    if (m->esurf != EGL_NO_SURFACE) eglDestroySurface(m->edpy, m->esurf);
    if (m->ectx  != EGL_NO_CONTEXT) eglDestroyContext(m->edpy, m->ectx);
    if (m->egl_window) wl_egl_window_destroy(m->egl_window);
    eglTerminate(m->edpy);
}
