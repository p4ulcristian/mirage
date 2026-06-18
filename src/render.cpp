#include "mirage.h"
#include "pose.h"
#include "handle.hpp"
#include "entity.hpp"
#include "camera.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <ctype.h>
#include <vector>
#include <print>

#include "stb_truetype.h"
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
    "uniform float uOpacity;\n"                  /* screen fade (1.0 = opaque) */
    "void main() {\n"
    "  if (uHasTex < 0.5) { gl_FragColor = vec4(uColor, uOpacity); return; }\n"
    "  vec3 e = texture2D(uTex, vUV).rgb;\n"
    "  if (uSharpen > 0.0) {\n"
    "    vec3 b = texture2D(uTex, vUV + vec2(0.0, -uTexel.y)).rgb;\n"
    "    vec3 d = texture2D(uTex, vUV + vec2(-uTexel.x, 0.0)).rgb;\n"
    "    vec3 f = texture2D(uTex, vUV + vec2( uTexel.x, 0.0)).rgb;\n"
    "    vec3 h = texture2D(uTex, vUV + vec2(0.0,  uTexel.y)).rgb;\n"
    "    vec3 s = e + (e - (b + d + f + h) * 0.25) * uSharpen;\n"
    "    e = clamp(s, min(min(b,d), min(f,h)), max(max(b,d), max(f,h)));\n"
    "  }\n"
    "  gl_FragColor = vec4(e, uOpacity);\n"
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
    own::GlProgram prog;
    GLint  aPos, aUV;
    GLint  uMVP, uYFlip, uHasTex, uColor, uTex, uTexel, uSharpen, uOpacity;
    own::GlBuffer vbo;

    /* HDRI environment dome */
    own::GlProgram dome_prog;
    own::GlBuffer  dome_vbo;
    own::GlTexture hdri_tex;
    int    dome_verts;
    GLint  dMVP, dExposure, dIntensity, dBlack, dSat, dTex;

    /* flat/curved toggle captions (one each, baked once at init; the live one is
     * shown on the toggle button below the brightness slider). */
    own::GlTexture label_flat, label_curved;
    int    flat_w, flat_h, curved_w, curved_h;
    /* live FPS plaque: re-baked only when the integer value changes */
    own::GlTexture label_fps;
    int    fps_w, fps_h, fps_val;
    /* static multi-line shortcut cheat-sheet, baked once at init */
    own::GlTexture label_help;
    int    help_w, help_h;
    /* sensitivity slider: static "SENS"/"DEFAULT" captions baked once, plus a live
     * value plaque ("3.0x") re-baked only when the gain changes (like the FPS one). */
    own::GlTexture label_sens_cap, label_default, label_sens;
    int    senscap_w, senscap_h, default_w, default_h, sens_w, sens_h, sens_val;
    own::GlTexture label_bright;            /* "BRIGHT" caption for the env brightness slider */
    int    bright_w, bright_h;
    own::GlTexture label_trans;             /* "TRANS" caption for the transparency slider */
    int    trans_w, trans_h;
    /* layout-switcher button captions (one per named layout), baked once at init */
    own::GlTexture label_layout[MIRAGE_MAX_LAYOUTS];
    int    layout_w[MIRAGE_MAX_LAYOUTS], layout_h[MIRAGE_MAX_LAYOUTS];
    /* environment-switcher button captions (one per MIRAGE_ENVS entry), baked once */
    own::GlTexture label_env[MIRAGE_MAX_ENVS];
    int    env_w[MIRAGE_MAX_ENVS], env_h[MIRAGE_MAX_ENVS];
    /* 3D pointer: a white arrow on black, drawn additively as a billboard at the
     * cursor's wall direction (m->cursor_yaw/pitch). Black adds nothing on the
     * additive optics, so only the arrow glows - over screens and in the gaps. */
    own::GlTexture cursor_tex;
    int    cursor_w, cursor_h;
    /* camera passthrough: the decoded camera frame as a texture (re-uploaded only on a
     * new frame), plus the toggle-button captions. cam_alloc_w/h track the texture's
     * allocated size so we glTexImage2D once and glTexSubImage2D after. */
    own::GlTexture cam_tex;
    int      cam_alloc_w, cam_alloc_h;
    uint64_t cam_seq;
    /* background-mode button captions, indexed by mirage_bg_mode */
    own::GlTexture label_bg[BG_MODE_COUNT];
    int    bg_w[BG_MODE_COUNT], bg_h[BG_MODE_COUNT];
    /* tracking-tier button captions, indexed by mirage_track_mode */
    own::GlTexture label_tk[TRACK_MODE_COUNT];
    int    tk_w[TRACK_MODE_COUNT], tk_h[TRACK_MODE_COUNT];
} R;

/* Banner entities (the clock, status lines, ...): baked panels hung in the curved
 * space alongside the displays. First slice of the scene; Model/Webapp join later. */
static std::vector<ent::Banner> g_banners;

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
        std::print(stderr, "render: shader compile failed: {}\n", log);
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
    std::vector<GLfloat> buf((size_t)verts * 5);
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
    glBufferData(GL_ARRAY_BUFFER, (GLsizeiptr)(verts*5*sizeof(GLfloat)), buf.data(), GL_STATIC_DRAW);
    s->mesh_verts = verts;
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

/* (Re)build every screen's mesh from the current cfg. Called once at init
 * and again whenever a layout switch changes arcs/geometry/distance; old VBOs are
 * released first so a repeated switch doesn't leak GPU buffers. Needs a live ctx. */
void render_rebuild_meshes(struct mirage *m) {
    int n = m->n_screen;
    if (n < 0 || n > MIRAGE_MAX_SCREENS) n = 0;
    for (int i = 0; i < n; i++) {
        screen_t *s = &m->screen[i];
        if (s->mesh_vbo) { glDeleteBuffers(1, &s->mesh_vbo); s->mesh_vbo = 0; }
        /* each screen's angular width is its per-screen override (cfg.screen_arc[i]),
         * falling back to the default - so the wide wall and the narrower 16:9 each
         * get their own arc. */
        s->arc_deg = m->cfg.screen_arc[i] > 0.0f
                     ? m->cfg.screen_arc[i] : m->cfg.screen_arc_deg;
        if (m->cfg.geometry == GEOM_FLAT) build_flat_mesh(m, s);
        else                              build_curved_mesh(m, s);
    }
}

/* ---- status plaque text (stb_truetype) -------------------------------------
 * Real TTF glyphs rasterised into an RGBA plaque texture: a monospace HUD font
 * (assets/hud.ttf, override with $MIRAGE_FONT), antialiased, foreground glyphs
 * over the dark plaque. One texture is baked per label (FPS/cheat-sheet/
 * clock) and the callers are unchanged. Replaces the old hand-rolled 5x7 bitmap
 * font - this gives lowercase, any glyph, and smooth edges. */
#define HUD_PX   44      /* glyph cell pixel height                 */
#define HUD_PAD  8       /* plaque border (px)                      */
#define HUD_LGAP 6       /* extra px between stacked lines          */

struct HudFont {
    bool ok = false;
    stbtt_fontinfo info{};
    std::vector<unsigned char> data;
    float scale = 0.0f;
    int   ascent_px = 0, line_h = 0, advance_px = 0;
};

/* Load the HUD font once (cached). Monospace, so one advance width fits all. */
static const HudFont &hud_font(void) {
    static HudFont F = [] {
        HudFont f;
        const char *paths[] = { getenv("MIRAGE_FONT"), "assets/hud.ttf",
            "/usr/share/fonts/TTF/JetBrainsMonoNerdFontMono-Regular.ttf" };
        for (const char *p : paths) {
            if (!p || !*p) continue;
            FILE *fp = fopen(p, "rb");
            if (!fp) continue;
            fseek(fp, 0, SEEK_END); long n = ftell(fp); fseek(fp, 0, SEEK_SET);
            if (n > 0) {
                f.data.resize((size_t)n);
                if (fread(f.data.data(), 1, (size_t)n, fp) == (size_t)n &&
                    stbtt_InitFont(&f.info, f.data.data(),
                                   stbtt_GetFontOffsetForIndex(f.data.data(), 0)))
                    f.ok = true;
            }
            fclose(fp);
            if (f.ok) { std::print(stderr, "render: HUD font {}\n", p); break; }
        }
        if (!f.ok) { std::print(stderr, "render: no HUD font found (text disabled)\n"); return f; }
        f.scale = stbtt_ScaleForPixelHeight(&f.info, (float)HUD_PX);
        int asc, desc, gap; stbtt_GetFontVMetrics(&f.info, &asc, &desc, &gap);
        f.ascent_px = (int)(asc * f.scale + 0.5f);
        f.line_h    = (int)((asc - desc) * f.scale + 0.5f);
        int adv, lsb; stbtt_GetCodepointHMetrics(&f.info, '0', &adv, &lsb);  /* monospace */
        f.advance_px = (int)(adv * f.scale + 0.5f);
        return f;
    }();
    return F;
}

/* Pixel height of a `lines`-line plaque (matches bake_label's vertical layout). */
static int hud_plaque_h(int lines) {
    const HudFont &f = hud_font();
    return HUD_PAD*2 + lines*f.line_h + (lines > 1 ? lines-1 : 0)*HUD_LGAP;
}

/* Rasterise `str` (may contain '\n') into a fresh RGBA texture: fg glyphs over a
 * dark plaque. Monospace, so stacked lines column-align. Stores dims in *ow,*oh. */
static own::GlTexture bake_label(const char *str, const float fg[3], int *ow, int *oh) {
    const HudFont &f = hud_font();
    int lines = 1, cur = 0, maxlen = 0;
    for (const char *p = str; *p; p++) {
        if (*p == '\n') { if (cur > maxlen) maxlen = cur; cur = 0; lines++; }
        else cur++;
    }
    if (cur > maxlen) maxlen = cur;
    int adv = f.ok ? f.advance_px : 1;
    int tw = HUD_PAD*2 + maxlen*adv;
    int th = hud_plaque_h(lines);
    if (tw < 1) tw = 1;
    if (th < 1) th = 1;
    std::vector<unsigned char> px((size_t)tw*th*4);
    const unsigned char bg[3] = {14, 18, 34};
    unsigned char fc[3] = { (unsigned char)(fg[0]*255), (unsigned char)(fg[1]*255),
                            (unsigned char)(fg[2]*255) };
    for (int i = 0; i < tw*th; i++) {
        px[i*4+0] = bg[0]; px[i*4+1] = bg[1]; px[i*4+2] = bg[2]; px[i*4+3] = 255;
    }
    if (f.ok) {
        int line = 0, col = 0;
        for (const char *p = str; *p; p++) {
            if (*p == '\n') { line++; col = 0; continue; }
            int baseline = HUD_PAD + line*(f.line_h + HUD_LGAP) + f.ascent_px;
            int penx     = HUD_PAD + col*adv;
            int gw, gh, gx, gy;
            unsigned char *bmp = stbtt_GetCodepointBitmap(&f.info, f.scale, f.scale,
                                     (unsigned char)*p, &gw, &gh, &gx, &gy);
            if (bmp) {
                for (int yy = 0; yy < gh; yy++)
                    for (int xx = 0; xx < gw; xx++) {
                        int x = penx + gx + xx, y = baseline + gy + yy;
                        if (x < 0 || y < 0 || x >= tw || y >= th) continue;
                        unsigned int cov = bmp[yy*gw + xx];
                        unsigned char *o = &px[((size_t)y*tw + x)*4];
                        for (int kk = 0; kk < 3; kk++)
                            o[kk] = (unsigned char)((bg[kk]*(255u-cov) + fc[kk]*cov) / 255u);
                    }
                stbtt_FreeBitmap(bmp, nullptr);
            }
            col++;
        }
    }
    own::GlTexture tex; tex.gen();
    glBindTexture(GL_TEXTURE_2D, tex);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, tw, th, 0, GL_RGBA, GL_UNSIGNED_BYTE, px.data());
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
    if (ow) *ow = tw;
    if (oh) *oh = th;
    return tex;
}

/* ---- banner entities ---------------------------------------------------------
 * A Banner is a curved strip of baked text hung in the cylindrical space (same
 * radius = screen distance as the displays). Generic: any number of them, each
 * with its own placement + content. Re-baked only when its key() changes (the
 * clock's second), then drawn each frame at its (yaw, lift). */

static void banner_refresh(ent::Banner &b, float d) {
    int k = b.key ? b.key() : 0;
    if (k == b.last_key && b.verts > 0) return;          /* content unchanged */
    b.last_key = k;
    b.tex = bake_label(b.text ? b.text().c_str() : "", b.color, &b.tw, &b.th);

    float ang = b.arc * (float)M_PI/180.0f;
    float w   = d * ang;                                 /* on-wall width (arc len) */
    float h   = w * (b.tw > 0 ? (float)b.th / (float)b.tw : 0.3f);  /* keep text aspect */
    const int cols = 48;
    int verts = (cols + 1) * 2;
    std::vector<GLfloat> buf((size_t)verts * 5);
    int j = 0;
    for (int ci = 0; ci <= cols; ci++) {
        float u   = (float)ci / (float)cols;
        float phi = -ang * 0.5f + u * ang;               /* swept about the panel's centre */
        float x   =  d * sinf(phi);
        float z   = -d * cosf(phi);
        buf[j++]=x; buf[j++]= h*0.5f; buf[j++]=z; buf[j++]=u; buf[j++]=0.0f;
        buf[j++]=x; buf[j++]=-h*0.5f; buf[j++]=z; buf[j++]=u; buf[j++]=1.0f;
    }
    b.vbo.gen();
    glBindBuffer(GL_ARRAY_BUFFER, b.vbo);
    glBufferData(GL_ARRAY_BUFFER, (GLsizeiptr)(verts*5*sizeof(GLfloat)), buf.data(), GL_STATIC_DRAW);
    b.verts = verts;
}

static void banner_draw(const ent::Banner &b, const mat4 &vp) {
    if (b.verts <= 0) return;
    mat4 model = m4_mul(m4_translate(v3(0.0f, b.lift, 0.0f)),
                        m4_from_quat(q_from_euler_ypr(b.yaw, 0.0f, 0.0f)));
    mat4 mvp = m4_mul(vp, model);
    glUniformMatrix4fv(R.uMVP, 1, GL_FALSE, mvp.m);
    glBindBuffer(GL_ARRAY_BUFFER, b.vbo);
    glEnableVertexAttribArray(R.aPos);
    glVertexAttribPointer(R.aPos, 3, GL_FLOAT, GL_FALSE, 5*sizeof(GLfloat), (void*)0);
    glEnableVertexAttribArray(R.aUV);
    glVertexAttribPointer(R.aUV, 2, GL_FLOAT, GL_FALSE, 5*sizeof(GLfloat), (void*)(3*sizeof(GLfloat)));
    glActiveTexture(GL_TEXTURE0);
    glBindTexture(GL_TEXTURE_2D, b.tex);
    glUniform1i(R.uTex, 0);
    glUniform1f(R.uHasTex, 1.0f);
    glUniform1f(R.uYFlip, 0.0f);
    glUniform1f(R.uSharpen, 0.0f);
    glDrawArrays(GL_TRIANGLE_STRIP, 0, b.verts);
}

/* The clock: the first Banner entity. Two lines (HH:MM:SS over an upper-cased
 * date), centred, warm amber, hung above the wall. Add more banners by pushing
 * more ent::Banner with their own text()/key()/placement. */
static ent::Banner make_clock_banner(void) {
    ent::Banner b;
    b.yaw = 0.0f; b.lift = 2.8f; b.arc = 30.0f;
    b.color[0] = 0.96f; b.color[1] = 0.87f; b.color[2] = 0.62f;
    b.text = [] {
        time_t tt = time(NULL); struct tm lt; localtime_r(&tt, &lt);
        char l1[16], l2[16];
        strftime(l1, sizeof l1, "%H:%M:%S", &lt);
        strftime(l2, sizeof l2, "%a %b %d", &lt);
        for (char *p = l2; *p; p++) *p = (char)toupper((unsigned char)*p);
        std::string a = l1, c = l2;
        size_t w = a.size() > c.size() ? a.size() : c.size();
        auto pad = [&](const std::string &s) {
            size_t l = (w - s.size()) / 2;
            return std::string(l, ' ') + s + std::string(w - s.size() - l, ' ');
        };
        return pad(a) + "\n" + pad(c);
    };
    b.key = [] {
        time_t tt = time(NULL); struct tm lt; localtime_r(&tt, &lt);
        return ((lt.tm_yday*24 + lt.tm_hour)*60 + lt.tm_min)*60 + lt.tm_sec;
    };
    return b;
}

/* ---- HDRI environment dome ---------------------------------------------------
 * Load a FLAT (non-RLE) Radiance .hdr - the format hdri/exr2hdr.py writes - into an
 * RGB8 buffer. Values are clamped to [0,1] and sqrt-encoded so the 8-bit texture
 * keeps precision in the dark range (faint stars); the dome shader squares it back
 * to linear. Returns a malloc'd w*h*3 buffer (caller frees) or NULL on failure. */
static unsigned char *load_hdri_rgb8(const char *path, int *ow, int *oh) {
    FILE *f = fopen(path, "rb");
    if (!f) { std::print(stderr, "hdri: cannot open {}\n", path); return NULL; }
    char line[256];
    int w = 0, h = 0;
    while (fgets(line, sizeof line, f))         /* skip header to the "-Y h +X w" line */
        if (line[0] == '-' && (line[1] == 'Y' || line[1] == 'y')) {
            sscanf(line, "-Y %d +X %d", &h, &w); break;
        }
    if (w <= 0 || h <= 0) { std::print(stderr, "hdri: bad/RLE .hdr {}\n", path); fclose(f); return NULL; }
    size_t n = (size_t)w * h;
    std::vector<unsigned char> rgbe(n * 4);
    unsigned char *out = (unsigned char*)malloc(n * 3);   /* returned; caller frees */
    if (!out || fread(rgbe.data(), 4, n, f) != n) {
        std::print(stderr, "hdri: short read {}\n", path);
        free(out); fclose(f); return NULL;
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
    *ow = w; *oh = h;
    return out;
}

/* A full UV sphere of unit directions (radius scaled large), centred on the eye.
 * Position is reused as the look direction in DOME_FRAG. */
static void build_dome(void) {
    const int NLAT = 32, NLON = 64;
    const float Rr = 50.0f;
    int verts = NLAT * NLON * 6;
    std::vector<GLfloat> buf((size_t)verts * 3);
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
    R.dome_vbo.gen();
    glBindBuffer(GL_ARRAY_BUFFER, R.dome_vbo);
    glBufferData(GL_ARRAY_BUFFER, (GLsizeiptr)(verts*3*sizeof(GLfloat)), buf.data(), GL_STATIC_DRAW);
    R.dome_verts = verts;
}

/* Build the dome program, load the HDRI into a texture, and build the sphere.
 * Disables the dome (R.dome_prog stays 0) if anything fails, so render_frame skips it. */
/* (Re)upload a flat Radiance .hdr into R.hdri_tex (assumes it's been gen'd). Shared by
 * first-time init and runtime environment switches - the texture object is reused, so a
 * switch is just a re-upload (no gen/delete churn). */
static bool hdri_upload(const char *path) {
    int w, h;
    unsigned char *px = load_hdri_rgb8(path, &w, &h);
    if (!px) { std::print(stderr, "hdri: load failed {}\n", path); return false; }
    glBindTexture(GL_TEXTURE_2D, R.hdri_tex);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, w, h, 0, GL_RGB, GL_UNSIGNED_BYTE, px);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);          /* equirect wraps in u */
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
    free(px);
    std::print(stderr, "hdri: loaded {}x{} {}\n", w, h, path);
    return true;
}

static void hdri_init(struct mirage *m) {
    if (!m->cfg.hdri_on) return;
    R.hdri_tex.gen();
    if (!hdri_upload(m->cfg.hdri_path)) { std::print(stderr, "hdri: disabled (load failed)\n"); return; }

    GLuint vs = compile(GL_VERTEX_SHADER, DOME_VERT);
    GLuint fs = compile(GL_FRAGMENT_SHADER, DOME_FRAG);
    if (!vs || !fs) return;
    R.dome_prog.create();
    glAttachShader(R.dome_prog, vs);
    glAttachShader(R.dome_prog, fs);
    glBindAttribLocation(R.dome_prog, 0, "aPos");
    glLinkProgram(R.dome_prog);
    GLint ok = 0; glGetProgramiv(R.dome_prog, GL_LINK_STATUS, &ok);
    if (!ok) { std::print(stderr, "hdri: dome link failed\n"); R.dome_prog.reset(); return; }
    glDeleteShader(vs); glDeleteShader(fs);
    R.dMVP       = glGetUniformLocation(R.dome_prog, "uMVP");
    R.dExposure  = glGetUniformLocation(R.dome_prog, "uExposure");
    R.dIntensity = glGetUniformLocation(R.dome_prog, "uIntensity");
    R.dBlack     = glGetUniformLocation(R.dome_prog, "uBlack");
    R.dSat       = glGetUniformLocation(R.dome_prog, "uSaturation");
    R.dTex       = glGetUniformLocation(R.dome_prog, "uTex");
    build_dome();
    std::print(stderr, "hdri: dome ready ({})\n", m->cfg.hdri_path);
}

/* 3D pointer texture: a classic arrowhead (tip at top-left) filled white on a
 * black, transparent-on-the-optics background. 2x2 supersampled for a clean edge.
 * Drawn additively, so only the white arrow shows. Tip UV is CURSOR_TIP_U/V so the
 * draw can anchor the tip - the hotspot you point with - exactly on the cursor. */
static const float CURSOR_TIP_U = 0.10f, CURSOR_TIP_V = 0.08f;
static own::GlTexture gen_cursor_tex(int *ow, int *oh) {
    const int N = 64;
    std::vector<unsigned char> px((size_t)N * N * 4, 0);
    /* arrowhead triangle in pixel space (y down): tip, down-left heel, right point */
    const float ax[3] = { CURSOR_TIP_U*N, 0.16f*N, 0.74f*N };
    const float ay[3] = { CURSOR_TIP_V*N, 0.86f*N, 0.52f*N };
    auto edge = [&](float fx, float fy, int a, int b) {
        return (fx - ax[a]) * (ay[b] - ay[a]) - (fy - ay[a]) * (ax[b] - ax[a]);
    };
    for (int y = 0; y < N; y++) for (int x = 0; x < N; x++) {
        int cov = 0;
        for (int sy = 0; sy < 2; sy++) for (int sx = 0; sx < 2; sx++) {
            float fx = x + 0.25f + sx*0.5f, fy = y + 0.25f + sy*0.5f;
            float e0 = edge(fx, fy, 0, 1), e1 = edge(fx, fy, 1, 2), e2 = edge(fx, fy, 2, 0);
            bool in = (e0 >= 0 && e1 >= 0 && e2 >= 0) || (e0 <= 0 && e1 <= 0 && e2 <= 0);
            if (in) cov++;
        }
        unsigned char val = (unsigned char)(cov * 255 / 4);
        unsigned char *o = &px[((size_t)y*N + x) * 4];
        o[0] = o[1] = o[2] = o[3] = val;          /* white arrow, black elsewhere */
    }
    own::GlTexture tex; tex.gen();
    glBindTexture(GL_TEXTURE_2D, tex);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, N, N, 0, GL_RGBA, GL_UNSIGNED_BYTE, px.data());
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
    if (ow) *ow = N;
    if (oh) *oh = N;
    return tex;
}

/* Head-locked calibration overlay: a panel pinned to the glasses (built with proj
 * only - no head rotation - so it stays in front of you) showing the current
 * instruction and, while centring, a stillness bar. Additive + depth-off, like the
 * cursor arrow, so it floats over the scene without punching a black hole. */
static void calib_draw(struct mirage *m, mat4 proj) {
    if (!calib_active(m)) return;
    const char *txt = calib_text(m);
    if (!txt || !*txt) return;

    static const char *cached = nullptr;          /* re-bake only on text change */
    static own::GlTexture tex; static int tw = 0, th = 0;
    if (txt != cached) {
        const float fg[3] = {0.85f, 0.92f, 1.0f};
        tex = bake_label(txt, fg, &tw, &th);
        cached = txt;
    }
    if (!tex || th <= 0) return;

    glUseProgram(R.prog);
    glBindBuffer(GL_ARRAY_BUFFER, R.vbo);
    glEnableVertexAttribArray(R.aPos);
    glVertexAttribPointer(R.aPos, 3, GL_FLOAT, GL_FALSE, 5*sizeof(GLfloat), (void*)0);
    glEnableVertexAttribArray(R.aUV);
    glVertexAttribPointer(R.aUV, 2, GL_FLOAT, GL_FALSE, 5*sizeof(GLfloat), (void*)(3*sizeof(GLfloat)));
    glDisable(GL_DEPTH_TEST);
    glEnable(GL_BLEND);
    glBlendFunc(GL_ONE, GL_ONE);
    glUniform1f(R.uYFlip, 0.0f);
    glUniform1f(R.uSharpen, 0.0f);

    const float dist = 1.2f;                      /* panel distance ahead (m) */
    float H = 0.20f, W = H * ((float)tw / (float)th);
    {   /* instruction text */
        mat4 model = m4_mul(m4_translate(v3(0.0f, 0.08f, -dist)), m4_scale(v3(W, H, 1.0f)));
        glUniformMatrix4fv(R.uMVP, 1, GL_FALSE, m4_mul(proj, model).m);
        glActiveTexture(GL_TEXTURE0);
        glBindTexture(GL_TEXTURE_2D, tex);
        glUniform1i(R.uTex, 0);
        glUniform1f(R.uHasTex, 1.0f);
        glDrawArrays(GL_TRIANGLE_STRIP, 0, 4);
    }
    float prog = calib_progress(m);
    if (prog >= 0.0f) {                           /* stillness bar (CENTER step) */
        const float barW = 0.5f, barH = 0.02f, y = -0.10f;
        glUniform1f(R.uHasTex, 0.0f);
        mat4 track = m4_mul(m4_translate(v3(0.0f, y, -dist)), m4_scale(v3(barW, barH, 1.0f)));
        glUniformMatrix4fv(R.uMVP, 1, GL_FALSE, m4_mul(proj, track).m);
        glUniform3f(R.uColor, 0.10f, 0.12f, 0.16f);          /* faint full-width track */
        glDrawArrays(GL_TRIANGLE_STRIP, 0, 4);
        float fw = barW * prog; if (fw < 1e-4f) fw = 1e-4f;
        float x  = -barW*0.5f + fw*0.5f;                     /* left-anchor the fill */
        mat4 fill = m4_mul(m4_translate(v3(x, y, -dist)), m4_scale(v3(fw, barH, 1.0f)));
        glUniformMatrix4fv(R.uMVP, 1, GL_FALSE, m4_mul(proj, fill).m);
        glUniform3f(R.uColor, 0.45f, 0.62f, 0.95f);          /* bright fill */
        glDrawArrays(GL_TRIANGLE_STRIP, 0, 4);
    }
    glDisable(GL_BLEND);
    glEnable(GL_DEPTH_TEST);
}

mirage_status render_init(struct mirage *m) {
    EGLint major = 0, minor = 0;   /* EGL version, logged below */
    m->edpy = eglGetDisplay((EGLNativeDisplayType)m->display);
    if (m->edpy == EGL_NO_DISPLAY) return std::unexpected("no EGL display");
    if (!eglInitialize(m->edpy, &major, &minor))
        return std::unexpected("eglInitialize failed");
    if (!eglBindAPI(EGL_OPENGL_ES_API))
        return std::unexpected("eglBindAPI failed");
    m->ecfg = choose_config(m->edpy);
    if (!m->ecfg) return std::unexpected("no matching EGL config");

    const EGLint ctx_attrs[] = { EGL_CONTEXT_CLIENT_VERSION, 2, EGL_NONE };
    m->ectx = eglCreateContext(m->edpy, m->ecfg, EGL_NO_CONTEXT, ctx_attrs);
    if (m->ectx == EGL_NO_CONTEXT) return std::unexpected("no EGL context");

    m->egl_window = wl_egl_window_create(m->surface, m->glasses_w, m->glasses_h);
    if (!m->egl_window) return std::unexpected("egl_window create failed");
    m->esurf = eglCreateWindowSurface(m->edpy, m->ecfg,
                                      (EGLNativeWindowType)m->egl_window, NULL);
    if (m->esurf == EGL_NO_SURFACE) return std::unexpected("window surface failed");

    if (!eglMakeCurrent(m->edpy, m->esurf, m->esurf, m->ectx))
        return std::unexpected("eglMakeCurrent failed");
    /* Present every vblank with vsync on. glasses.sh sets the panel mode, so one
     * vblank = the panel period and this hardware-locks us to the refresh (GPU draw
     * is <1ms, so every frame lands with time to spare). NB interval 2 ("every 2nd
     * vblank") is NOT honored on the Hyprland direct-scanout path - the panel mode
     * is what actually fixes the rate. */
    eglSwapInterval(m->edpy, 1);

    GLuint vs = compile(GL_VERTEX_SHADER, VERT_SRC);
    GLuint fs = compile(GL_FRAGMENT_SHADER, FRAG_SRC);
    if (!vs || !fs) return std::unexpected("shader compile failed");
    R.prog.create();
    glAttachShader(R.prog, vs);
    glAttachShader(R.prog, fs);
    glBindAttribLocation(R.prog, 0, "aPos");
    glBindAttribLocation(R.prog, 1, "aUV");
    glLinkProgram(R.prog);
    GLint ok = 0; glGetProgramiv(R.prog, GL_LINK_STATUS, &ok);
    if (!ok) {
        char log[1024]; glGetProgramInfoLog(R.prog, sizeof log, NULL, log);
        return std::unexpected(std::string("program link failed: ") + log);
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
    R.uOpacity = glGetUniformLocation(R.prog, "uOpacity");
    glUseProgram(R.prog);
    glUniform1f(R.uOpacity, 1.0f);   /* default opaque; only the screen draw lowers it */

    R.vbo.gen();
    glBindBuffer(GL_ARRAY_BUFFER, R.vbo);
    glBufferData(GL_ARRAY_BUFFER, sizeof QUAD, QUAD, GL_STATIC_DRAW);

    glEnable(GL_DEPTH_TEST);

    render_rebuild_meshes(m);

    /* flat/curved toggle captions (one shown at a time on the toggle button) */
    { const float gc[3] = {0.66f, 0.72f, 0.82f};
      R.label_curved = bake_label("CURVED", gc, &R.curved_w, &R.curved_h);
      R.label_flat   = bake_label("FLAT",   gc, &R.flat_w,   &R.flat_h); }
    /* background-mode button captions (black / hdri / passthrough) */
    { const float pc[3] = {0.82f, 0.74f, 0.66f};
      R.label_bg[BG_BLACK]       = bake_label("BG: BLACK",    pc, &R.bg_w[BG_BLACK],       &R.bg_h[BG_BLACK]);
      R.label_bg[BG_HDRI]        = bake_label("BG: HDRI",     pc, &R.bg_w[BG_HDRI],        &R.bg_h[BG_HDRI]);
      R.label_bg[BG_PASSTHROUGH] = bake_label("BG: PASSTHRU", pc, &R.bg_w[BG_PASSTHROUGH], &R.bg_h[BG_PASSTHROUGH]); }
    /* tracking-tier button captions (3DoF / 3DoF+) */
    { const float pc[3] = {0.82f, 0.74f, 0.66f};
      R.label_tk[TRACK_3DOF] = bake_label("3DoF",  pc, &R.tk_w[TRACK_3DOF], &R.tk_h[TRACK_3DOF]);
      R.label_tk[TRACK_NECK] = bake_label("3DoF+", pc, &R.tk_w[TRACK_NECK], &R.tk_h[TRACK_NECK]); }
    R.cam_alloc_w = R.cam_alloc_h = 0; R.cam_seq = 0;
    R.fps_val = -1;   /* force the FPS plaque to bake on the first frame */
    R.cursor_tex = gen_cursor_tex(&R.cursor_w, &R.cursor_h);   /* 3D pointer arrow */
    /* static shortcut cheat-sheet, one multi-line plaque baked once */
    { const float hc[3] = {0.66f, 0.72f, 0.82f};
      R.label_help = bake_label(
          "2X CMD: RECENTER\n"
          "CMD+SCROLL: ZOOM\n"
          "SUPER+SHIFT+Q: QUIT",
          hc, &R.help_w, &R.help_h); }
    /* sensitivity slider captions (static) + force a value bake on the first frame */
    { const float cap[3] = {0.66f, 0.72f, 0.82f};
      const float dft[3] = {0.80f, 0.86f, 0.95f};
      R.label_sens_cap = bake_label("SENS", cap, &R.senscap_w, &R.senscap_h);
      R.label_default  = bake_label("DEFAULT", dft, &R.default_w, &R.default_h);
      const float bc[3] = {0.72f, 0.88f, 0.78f};   /* green, matches the env row */
      R.label_bright   = bake_label("BRIGHT", bc, &R.bright_w, &R.bright_h);
      const float tc[3] = {0.74f, 0.78f, 0.92f};   /* blue, matches the transparency rail */
      R.label_trans    = bake_label("TRANS", tc, &R.trans_w, &R.trans_h); }
    R.sens_val = -1;
    /* layout-switcher button captions: one per loaded named layout, upper-cased to
     * match the rest of the HUD. Layouts are parsed before render init, so the set
     * is fixed here. */
    { const float lc[3] = {0.72f, 0.80f, 0.92f};
      for (int k = 0; k < m->layouts.n; k++) {
          char nm[32];
          snprintf(nm, sizeof nm, "%s", m->layouts.l[k].name);
          for (char *p = nm; *p; ++p) *p = (char)toupper((unsigned char)*p);
          R.label_layout[k] = bake_label(nm, lc, &R.layout_w[k], &R.layout_h[k]);
      } }
    /* environment-switcher captions: one per MIRAGE_ENVS entry (Space/Forest/...) */
    { const float ec[3] = {0.72f, 0.88f, 0.78f};   /* faint green tint vs the layout row */
      for (int k = 0; k < MIRAGE_ENV_COUNT; k++) {
          char nm[16];
          snprintf(nm, sizeof nm, "%s", MIRAGE_ENVS[k].name);
          for (char *p = nm; *p; ++p) *p = (char)toupper((unsigned char)*p);
          R.label_env[k] = bake_label(nm, ec, &R.env_w[k], &R.env_h[k]);
      } }
    /* banner entities: register them here; the frame loop (banner_refresh) bakes
     * and builds each lazily and re-bakes when its key() changes. The clock is the
     * first; push more ent::Banner for status lines etc. */
    g_banners.clear();
    g_banners.push_back(make_clock_banner());

    hdri_init(m);   /* environment dome (no-op if cfg.hdri_on is false or load fails) */

    std::print(stderr, "render: EGL {}.{}, GL_RENDERER={}\n", major, minor,
            (const char*)glGetString(GL_RENDERER));
    return {};
}

/* placeholder tints for screens with no capture yet */
static const float PLACEHOLDER[][3] = {
    {0.20f, 0.10f, 0.30f}, {0.10f, 0.25f, 0.20f}, {0.28f, 0.18f, 0.08f},
    {0.10f, 0.18f, 0.30f}, {0.25f, 0.10f, 0.15f},
};

/* In-view sensitivity slider: a track + fill + draggable handle and a DEFAULT
 * button, hung under the centre screen with the FPS plaque. Geometry comes
 * from sens_panel_compute (the same numbers grab.c hit-tests), drawn in the centre
 * screen's local frame via layout_model_matrix. Additive + depth-off like the
 * cursor arrow, so it glows on the optics without punching a black hole. */
static void draw_sens_panel(struct mirage *m, mat4 vp) {
    if (calib_active(m)) return;            /* the wizard owns the view */
    sens_panel sp;
    if (!sens_panel_compute(m, &sp)) return;

    mat4 base = layout_model_matrix(m, sp.ci);

    glUseProgram(R.prog);
    glBindBuffer(GL_ARRAY_BUFFER, R.vbo);
    glEnableVertexAttribArray(R.aPos);
    glVertexAttribPointer(R.aPos, 3, GL_FLOAT, GL_FALSE, 5*sizeof(GLfloat), (void*)0);
    glEnableVertexAttribArray(R.aUV);
    glVertexAttribPointer(R.aUV, 2, GL_FLOAT, GL_FALSE, 5*sizeof(GLfloat), (void*)(3*sizeof(GLfloat)));
    glDisable(GL_DEPTH_TEST);
    glEnable(GL_BLEND);
    glBlendFunc(GL_ONE, GL_ONE);
    glUniform1f(R.uYFlip, 0.0f);
    glUniform1f(R.uSharpen, 0.0f);

    auto solid = [&](float cx, float cy, float w, float h, float r, float g, float b) {
        mat4 local = m4_mul(m4_translate(v3(cx, cy, -sp.d)), m4_scale(v3(w, h, 1.0f)));
        glUniformMatrix4fv(R.uMVP, 1, GL_FALSE, m4_mul(vp, m4_mul(base, local)).m);
        glUniform1f(R.uHasTex, 0.0f);
        glUniform3f(R.uColor, r, g, b);
        glDrawArrays(GL_TRIANGLE_STRIP, 0, 4);
    };
    /* a hollow rectangle from four thin bright bars - on additive optics an outline
     * reads as an affordance where a dark fill would just be invisible. */
    auto outline = [&](float cx, float cy, float w, float h, float t,
                       float r, float g, float b) {
        solid(cx, cy + h*0.5f, w, t, r, g, b);   /* top    */
        solid(cx, cy - h*0.5f, w, t, r, g, b);   /* bottom */
        solid(cx - w*0.5f, cy, t, h, r, g, b);   /* left   */
        solid(cx + w*0.5f, cy, t, h, r, g, b);   /* right  */
    };
    auto label = [&](GLuint tex, float cx, float cy, float h, int tw, int th) {
        if (!tex || th <= 0) return;
        float w = h * ((float)tw / (float)th);
        mat4 local = m4_mul(m4_translate(v3(cx, cy, -sp.d)), m4_scale(v3(w, h, 1.0f)));
        glUniformMatrix4fv(R.uMVP, 1, GL_FALSE, m4_mul(vp, m4_mul(base, local)).m);
        glActiveTexture(GL_TEXTURE0);
        glBindTexture(GL_TEXTURE_2D, tex);
        glUniform1i(R.uTex, 0);
        glUniform1f(R.uHasTex, 1.0f);
        glDrawArrays(GL_TRIANGLE_STRIP, 0, 4);
    };

    /* All colours are ADDED light (GL_ONE,GL_ONE): there is no "dark" on the optics,
     * so the rail is a dim-but-present glow, the fill/handle are brighter, and the
     * track extent is marked by bright end ticks rather than a dark box. */
    float trackW = sp.track_x1 - sp.track_x0;
    solid(0.0f, sp.row_y, trackW, sp.track_h, 0.22f, 0.26f, 0.34f);   /* visible rail */
    float fillW = sp.handle_x - sp.track_x0;
    if (fillW > 1e-4f)
        solid(sp.track_x0 + fillW*0.5f, sp.row_y, fillW, sp.track_h, 0.30f, 0.42f, 0.65f);
    /* bright end ticks so the min/max extent reads at a glance */
    float tickH = sp.track_h * 3.0f, tickW = 0.008f;
    solid(sp.track_x0, sp.row_y, tickW, tickH, 0.40f, 0.48f, 0.62f);
    solid(sp.track_x1, sp.row_y, tickW, tickH, 0.40f, 0.48f, 0.62f);
    /* handle */
    solid(sp.handle_x, sp.row_y, sp.handle_w, sp.handle_h, 0.60f, 0.76f, 1.00f);
    /* DEFAULT button: a bright outline (a dark fill would be invisible additively) */
    float def_cx = 0.5f*(sp.def_x0 + sp.def_x1), def_cy = 0.5f*(sp.def_y0 + sp.def_y1);
    float def_w  = sp.def_x1 - sp.def_x0,        def_h  = sp.def_y1 - sp.def_y0;
    outline(def_cx, def_cy, def_w, def_h, 0.006f, 0.34f, 0.40f, 0.54f);

    /* live value plaque ("3.0x"), re-baked only when the gain changes */
    int sv = (int)(sp.gain * 10.0f + 0.5f);
    if (sv != R.sens_val) {
        char buf[16]; snprintf(buf, sizeof buf, "%.1fX", (double)sp.gain);
        const float vc[3] = {0.62f, 0.78f, 0.95f};
        R.label_sens = bake_label(buf, vc, &R.sens_w, &R.sens_h);
        R.sens_val = sv;
    }
    label(R.label_sens_cap, sp.track_x0 - 0.13f, sp.row_y,       0.055f, R.senscap_w, R.senscap_h);
    label(R.label_sens,     0.0f,                sp.row_y + 0.11f, 0.055f, R.sens_w,   R.sens_h);
    label(R.label_default,  def_cx,              def_cy,         0.045f, R.default_w, R.default_h);

    /* layout switcher: a clickable box per named layout, the active one filled and
     * bright-bordered, the rest a dim idle outline. Each caption is scaled down to
     * fit its box so long names ("THEATER") don't spill past the border. */
    for (int k = 0; k < sp.n_layout; k++) {
        float cx = sp.lay_cx[k];
        float cy = 0.5f * (sp.lay_y0 + sp.lay_y1);
        float w  = sp.lay_w, h = sp.lay_y1 - sp.lay_y0;
        if (k == sp.active_layout) {
            solid(cx, cy, w, h, 0.16f, 0.22f, 0.34f);            /* active: filled glow */
            outline(cx, cy, w, h, 0.006f, 0.50f, 0.66f, 0.95f);  /* bright border       */
        } else {
            outline(cx, cy, w, h, 0.006f, 0.30f, 0.36f, 0.48f);  /* idle border         */
        }
        float lh = 0.038f, maxw = w * 0.86f;
        if (R.layout_h[k] > 0) {
            float natw = lh * (float)R.layout_w[k] / (float)R.layout_h[k];
            if (natw > maxw) lh = maxw * (float)R.layout_h[k] / (float)R.layout_w[k];
        }
        label(R.label_layout[k], cx, cy, lh, R.layout_w[k], R.layout_h[k]);
    }

    /* environment switcher: same box style as the layout row, one row below, in a
     * green tint so the two rows read as distinct switchers. Active = filled+bright. */
    for (int k = 0; k < sp.n_env; k++) {
        float cx = sp.env_cx[k];
        float cy = 0.5f * (sp.env_y0 + sp.env_y1);
        float w  = sp.env_w, h = sp.env_y1 - sp.env_y0;
        if (k == sp.active_env) {
            solid(cx, cy, w, h, 0.16f, 0.30f, 0.22f);            /* active: filled glow */
            outline(cx, cy, w, h, 0.006f, 0.46f, 0.86f, 0.60f);  /* bright green border */
        } else {
            outline(cx, cy, w, h, 0.006f, 0.30f, 0.46f, 0.36f);  /* idle border         */
        }
        float lh = 0.038f, maxw = w * 0.86f;
        if (R.env_h[k] > 0) {
            float natw = lh * (float)R.env_w[k] / (float)R.env_h[k];
            if (natw > maxw) lh = maxw * (float)R.env_h[k] / (float)R.env_w[k];
        }
        label(R.label_env[k], cx, cy, lh, R.env_w[k], R.env_h[k]);
    }

    /* environment brightness slider: rail + fill + handle (green to match the env row),
     * with a "BRIGHT" caption to its left. grab.cpp drives the handle. */
    float briW = sp.bri_x1 - sp.bri_x0;
    solid(0.0f, sp.bri_row_y, briW, sp.bri_track_h, 0.20f, 0.30f, 0.24f);   /* rail */
    float briFill = sp.bri_handle_x - sp.bri_x0;
    if (briFill > 1e-4f)
        solid(sp.bri_x0 + briFill*0.5f, sp.bri_row_y, briFill, sp.bri_track_h, 0.26f, 0.46f, 0.34f);
    solid(sp.bri_handle_x, sp.bri_row_y, sp.bri_handle_w, sp.bri_handle_h, 0.52f, 0.86f, 0.64f);
    label(R.label_bright, sp.bri_x0 - 0.14f, sp.bri_row_y, 0.045f, R.bright_w, R.bright_h);

    /* window transparency slider: same style, amber/blue tint, "TRANS" caption left. */
    float trW = sp.tr_x1 - sp.tr_x0;
    solid(0.0f, sp.tr_row_y, trW, sp.tr_track_h, 0.26f, 0.26f, 0.32f);        /* rail */
    float trFill = sp.tr_handle_x - sp.tr_x0;
    if (trFill > 1e-4f)
        solid(sp.tr_x0 + trFill*0.5f, sp.tr_row_y, trFill, sp.tr_track_h, 0.40f, 0.44f, 0.60f);
    solid(sp.tr_handle_x, sp.tr_row_y, sp.tr_handle_w, sp.tr_handle_h, 0.66f, 0.72f, 0.92f);
    label(R.label_trans, sp.tr_x0 - 0.14f, sp.tr_row_y, 0.045f, R.trans_w, R.trans_h);

    /* flat/curved toggle: one button spanning the track, filled + bright-bordered
     * (it always reflects an active choice), captioned with the current geometry.
     * Amber tint so it reads as distinct from the blue/green switcher rows above. */
    {
        float cx = 0.5f * (sp.geo_x0 + sp.geo_x1);
        float cy = 0.5f * (sp.geo_y0 + sp.geo_y1);
        float w  = sp.geo_x1 - sp.geo_x0, h = sp.geo_y1 - sp.geo_y0;
        solid(cx, cy, w, h, 0.30f, 0.24f, 0.12f);            /* filled glow         */
        outline(cx, cy, w, h, 0.006f, 0.90f, 0.70f, 0.40f);  /* bright amber border */
        GLuint tex = sp.geo_flat ? R.label_flat : R.label_curved;
        int tw = sp.geo_flat ? R.flat_w : R.curved_w;
        int th = sp.geo_flat ? R.flat_h : R.curved_h;
        float lh = 0.038f, maxw = w * 0.86f;
        if (th > 0) {
            float natw = lh * (float)tw / (float)th;
            if (natw > maxw) lh = maxw * (float)th / (float)tw;
        }
        label(tex, cx, cy, lh, tw, th);
    }

    /* background-mode button: cycles black / hdri / passthrough; teal when on a live
     * background (hdri/passthrough), dim grey for black. Caption shows the mode. */
    {
        int mode = sp.pt_mode; if (mode < 0 || mode >= BG_MODE_COUNT) mode = BG_HDRI;
        float cx = 0.5f * (sp.pt_x0 + sp.pt_x1);
        float cy = 0.5f * (sp.pt_y0 + sp.pt_y1);
        float w  = sp.pt_x1 - sp.pt_x0, h = sp.pt_y1 - sp.pt_y0;
        if (mode == BG_BLACK) {
            outline(cx, cy, w, h, 0.006f, 0.40f, 0.42f, 0.46f);  /* dim grey border     */
        } else {
            solid(cx, cy, w, h, 0.10f, 0.26f, 0.28f);            /* lit teal fill       */
            outline(cx, cy, w, h, 0.006f, 0.40f, 0.86f, 0.90f);  /* bright teal border  */
        }
        GLuint tex = R.label_bg[mode];
        int tw = R.bg_w[mode], th = R.bg_h[mode];
        float lh = 0.038f, maxw = w * 0.86f;
        if (th > 0) {
            float natw = lh * (float)tw / (float)th;
            if (natw > maxw) lh = maxw * (float)th / (float)tw;
        }
        label(tex, cx, cy, lh, tw, th);
    }

    /* tracking-tier button: cycles 3DoF / 3DoF+ (neck model). Amber when 3DoF+ (position
     * active), dim grey for plain 3DoF. Caption shows the mode. */
    {
        int mode = sp.tk_mode; if (mode < 0 || mode >= TRACK_MODE_COUNT) mode = TRACK_NECK;
        float cx = 0.5f * (sp.tk_x0 + sp.tk_x1);
        float cy = 0.5f * (sp.tk_y0 + sp.tk_y1);
        float w  = sp.tk_x1 - sp.tk_x0, h = sp.tk_y1 - sp.tk_y0;
        if (mode == TRACK_3DOF) {
            outline(cx, cy, w, h, 0.006f, 0.40f, 0.42f, 0.46f);  /* dim grey border   */
        } else {
            solid(cx, cy, w, h, 0.30f, 0.24f, 0.12f);            /* amber fill         */
            outline(cx, cy, w, h, 0.006f, 0.90f, 0.70f, 0.40f);  /* bright amber border*/
        }
        GLuint tex = R.label_tk[mode];
        int tw = R.tk_w[mode], th = R.tk_h[mode];
        float lh = 0.038f, maxw = w * 0.86f;
        if (th > 0) {
            float natw = lh * (float)tw / (float)th;
            if (natw > maxw) lh = maxw * (float)th / (float)tw;
        }
        label(tex, cx, cy, lh, tw, th);
    }

    glDisable(GL_BLEND);
    glEnable(GL_DEPTH_TEST);
}

/* Camera passthrough: start/stop the camera off m->passthrough, pull the newest
 * frame into R.cam_tex, and draw it as a head-locked fullscreen background (the
 * Beast cam is head-mounted, so screen-space IS the correct world-lock). Drawn
 * first, depth-test off, so the windows/wall composite on top. Returns true if a
 * passthrough background was drawn (so the caller can skip the env dome). */
static bool draw_passthrough(struct mirage *m) {
    const char *dev = getenv("MIRAGE_CAM_DEV");
    if (!dev) dev = "/dev/video1";                 /* Beast world-cam (laptop = video0) */

    bool want = (m->bg_mode == BG_PASSTHROUGH);
    if (want && !m->cam) {
        m->cam = cam_start(dev, 1280, 720);
        if (!m->cam) { m->bg_mode = BG_HDRI; std::print(stderr, "passthrough: camera unavailable\n"); }
    } else if (!want && m->cam) {
        cam_stop(m->cam); m->cam = nullptr; R.cam_alloc_w = R.cam_alloc_h = 0;
    }
    if (!want || !m->cam) return false;

    const uint8_t *rgb; int cw, ch;
    if (cam_acquire(m->cam, &rgb, &cw, &ch, &R.cam_seq)) {
        if (!R.cam_tex) R.cam_tex.gen();
        glBindTexture(GL_TEXTURE_2D, R.cam_tex);
        glPixelStorei(GL_UNPACK_ALIGNMENT, 1);
        if (cw != R.cam_alloc_w || ch != R.cam_alloc_h) {
            glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, cw, ch, 0, GL_RGB, GL_UNSIGNED_BYTE, rgb);
            glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
            glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
            glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
            glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
            R.cam_alloc_w = cw; R.cam_alloc_h = ch;
        } else {
            glTexSubImage2D(GL_TEXTURE_2D, 0, 0, 0, cw, ch, GL_RGB, GL_UNSIGNED_BYTE, rgb);
        }
    }
    if (!R.cam_tex || R.cam_alloc_w == 0) return false;  /* no frame yet */

    /* fullscreen quad in NDC: the unit QUAD ([-0.5,0.5]) scaled x2, no projection. */
    glUseProgram(R.prog);
    glBindBuffer(GL_ARRAY_BUFFER, R.vbo);
    glEnableVertexAttribArray(R.aPos);
    glVertexAttribPointer(R.aPos, 3, GL_FLOAT, GL_FALSE, 5*sizeof(GLfloat), (void*)0);
    glEnableVertexAttribArray(R.aUV);
    glVertexAttribPointer(R.aUV, 2, GL_FLOAT, GL_FALSE, 5*sizeof(GLfloat), (void*)(3*sizeof(GLfloat)));
    glDisable(GL_DEPTH_TEST);
    glDisable(GL_BLEND);
    glUniform1f(R.uYFlip, 0.0f);
    glUniform1f(R.uSharpen, 0.0f);
    glUniform1f(R.uHasTex, 1.0f);
    mat4 ndc = m4_scale(v3(2.0f, 2.0f, 1.0f));
    glUniformMatrix4fv(R.uMVP, 1, GL_FALSE, ndc.m);
    glActiveTexture(GL_TEXTURE0);
    glBindTexture(GL_TEXTURE_2D, R.cam_tex);
    glUniform1i(R.uTex, 0);
    glDrawArrays(GL_TRIANGLE_STRIP, 0, 4);
    glEnable(GL_DEPTH_TEST);
    return true;
}

void render_frame(struct mirage *m, quat head) {
    eglMakeCurrent(m->edpy, m->esurf, m->esurf, m->ectx);
    struct timespec rt0; if (m->profile) clock_gettime(CLOCK_MONOTONIC, &rt0);

    /* a runtime layout switch (Alt+1/2/3) swapped m->cfg; rebuild meshes here,
     * on the render thread with the GL context current. */
    if (m->layout_dirty) { render_rebuild_meshes(m); m->layout_dirty = false; }

    glViewport(0, 0, m->glasses_w, m->glasses_h);
    glClearColor(m->cfg.bg[0], m->cfg.bg[1], m->cfg.bg[2], 1.0f);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    /* camera passthrough background (head-locked), drawn when bg_mode == BG_PASSTHROUGH.
     * Manages the camera lifecycle too; the dome draw below is gated on BG_HDRI. */
    draw_passthrough(m);

    float z = m->zoom > 0.0f ? m->zoom : 1.0f;
    float aspect = (float)m->glasses_w / (float)m->glasses_h;
    /* zoom narrows the field of view (zoom in = see less, bigger) */
    mat4 proj = m4_perspective((m->cfg.fov_deg / z) * (float)M_PI/180.0f, aspect, 0.05f, 600.0f);

    /* Reshape the head orientation for comfort, via a swing-twist split instead
     * of euler yaw/pitch/roll. "Swing" is the look direction (combined yaw+pitch
     * off the recenter forward); "twist" is roll about that forward. We amplify
     * the swing so the side screens need less neck turn, and damp the twist so
     * the wall stays level. This is gimbal-lock-free: the old euler version blew
     * up when you looked near straight up/down (lying down), spinning the view.
     * Swing only has a singularity looking dead backwards, which never happens.
     * NOTE: yaw and pitch share one gain here (swing is isotropic); yaw_gain is
     * used and should equal pitch_gain. Identity when look_gain=1, roll_damp=1. */
    quat swing, twist;
    q_swing_twist(head, v3(0.0f, 0.0f, -1.0f), &swing, &twist);
    swing = q_scale_angle(swing, m->cfg.yaw_gain);
    twist = q_scale_angle(twist, m->cfg.roll_damp);
    head  = q_norm(q_mul(swing, twist));

    /* Reading-stability deadband. The comfort gains above amplify head tremor
     * 2.5-3x, so even a still head leaves text shimmering. We hold the last
     * presented orientation and only follow once the camera has moved past a
     * small angle, then follow the EXCESS over that angle - a soft deadband, so
     * crossing the threshold doesn't snap. Sub-threshold tremor is frozen out
     * entirely (text sits dead still), while a real head turn dwarfs the
     * threshold and passes through with only ~deadband worth of lag. */
    if (m->cfg.read_deadband_deg > 0.0f) {
        static quat presented; static bool seeded = false;
        /* A recenter snaps the relative orientation; reseed so the deadband doesn't
         * slew the held pose across to the new forward over several frames. */
        static uint32_t last_recenter_gen = 0;
        uint32_t rg = pose_recenter_gen();
        if (rg != last_recenter_gen) { seeded = false; last_recenter_gen = rg; }
        if (!seeded) { presented = head; seeded = true; }
        else {
            /* Speed-gate the deadband. A fixed deadband freezes tremor when still
             * (good) but during a SLOW pan the gap-to-live hovers right at the
             * threshold, so `follow` flickers around zero -> erratic micro-jumps
             * (the slow-pan jitter). Fade the deadband to zero as soon as the head
             * is actually moving: full freeze when still, smooth 1:1 follow once
             * panning, so there's no unstable threshold band to sit in. Gate on the
             * physical (pre-gain) head speed - what "moving" actually means. */
            float spd = pose_speed();                        /* rad/s, physical */
            const float STILL = 2.0f * (float)(M_PI/180.0);  /* full deadband below 2 deg/s */
            const float MOVE  = 7.0f * (float)(M_PI/180.0);  /* released by 7 deg/s         */
            float mv = (spd - STILL) / (MOVE - STILL);
            if (mv < 0.0f) mv = 0.0f;
            if (mv > 1.0f) mv = 1.0f;
            float dot = presented.w*head.w + presented.x*head.x
                      + presented.y*head.y + presented.z*head.z;
            if (dot < 0.0f) dot = -dot;
            if (dot > 1.0f) dot = 1.0f;
            float ang = 2.0f * acosf(dot);                 /* rad moved this frame */
            float db  = m->cfg.read_deadband_deg * (float)M_PI/180.0f * (1.0f - mv);
            float follow = ang > 1e-6f ? (ang - db) / ang : 0.0f;
            if (follow < 0.0f) follow = 0.0f;              /* inside deadband: freeze */
            presented = q_nlerp(presented, head, follow);
        }
        head = presented;
    }

    /* Publish the look direction for shake-to-gaze: this is the exact camera
     * orientation we render through (comfort gains + deadband baked
     * in), so grab.c can map "where the eye points" back to a screen + pixel.
     * Same yaw/pitch convention as layout_focus_angles, so its inverse lands
     * straight on the cursor strip. */
    {
        float groll;
        q_to_euler_ypr(head, &m->gaze_yaw, &m->gaze_pitch, &groll);
        m->gaze_have = true;
    }

    /* drive the calibration overlay's state machine off the live head pose
     * (stillness -> recenter, etc.); the panel itself is drawn at the end. */
    calib_update(m, head);

    /* Eye translation for motion parallax (near windows shift against far ones and the
     * fixed star dome). Two sources, picked by whether the webcam is live:
     *
     *  - REAL position (facecam): the measured head offset already INCLUDES the neck-arc
     *    translation a head turn produces, so it fully replaces the neck model below -
     *    running both would double-count that arc. Forward-predicted (pose_predict_ms) to
     *    offset the camera's latency. Lateral (x/y) and depth (z) keep separate gains, as
     *    depth is the noisier axis.
     *  - NECK MODEL (fallback): with no webcam signal, synthesise the arc from rotation -
     *    the eye sits ahead/above a neck pivot, so a turn sweeps it through an arc. This is
     *    the only translational depth cue available from the 3DoF stream alone. */
    vec3 eye_world;
    if (m->track_mode == TRACK_3DOF) {
        /* pure orientation: no eye translation at all (screens pinned to a direction) */
        eye_world = v3(0.0f, 0.0f, 0.0f);
    } else if (m->cfg.facecam_enable && pose_position_active()) {
        vec3 hp = pose_position(m->cfg.pose_predict_ms * 0.001f);
        eye_world = v3(hp.x * m->cfg.facecam_lateral_gain,
                       hp.y * m->cfg.facecam_lateral_gain,
                       hp.z * m->cfg.facecam_depth_gain);
    } else {
        /* 3DoF+ neck model: synthesise the eye's neck-arc translation from rotation */
        eye_world = q_rotate(head, v3(0.0f, m->cfg.neck_up_m, -m->cfg.neck_fwd_m));
    }
    mat4 view = m4_mul(m4_from_quat(q_conj(head)),     /* world -> head rotation */
                       m4_translate(v3_scale(eye_world, -1.0f)));  /* then -eye  */
    mat4 vp   = m4_mul(proj, view);

    /* Environment switch (HUD): reload the dome texture once when env_dirty is set.
     * The new dome params (exposure/black/...) are read from cfg below each frame, so
     * only the texture needs swapping. Done here on the GL thread, not in grab. */
    if (m->env_dirty) {
        m->env_dirty = false;
        if (R.dome_prog && m->cfg.hdri_on) hdri_upload(m->cfg.hdri_path);
    }

    /* HDRI environment dome: drawn first as an infinite, world-fixed backdrop.
     * Additive (dark sky adds nothing on the optics), depth test + write off so the
     * wall draws cleanly over it. The dome sphere (radius 50 m) dwarfs the
     * neck-model eye shift (~0.1 m), so the stars stay effectively fixed in world
     * space - the far reference the near windows parallax against as you look around.
     * cfg.hdri_on gates the whole draw so the "Off" environment hides it. The dome
     * only shows in BG_HDRI mode (BG_BLACK = nothing, BG_PASSTHROUGH = the camera). */
    if (R.dome_prog && R.dome_vbo && m->cfg.hdri_on && m->bg_mode == BG_HDRI) {
        glUseProgram(R.dome_prog);
        glUniformMatrix4fv(R.dMVP, 1, GL_FALSE, vp.m);
        /* env_brightness is the HUD slider (1.0 = as tuned); guard the zero-init case. */
        float env_bri = m->env_brightness > 0.0f ? m->env_brightness : 1.0f;
        glUniform1f(R.dExposure,  m->cfg.hdri_exposure);
        glUniform1f(R.dIntensity, m->cfg.hdri_intensity * env_bri);
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
    /* window/screen transparency: alpha-blend the screens over the background (env
     * dome / passthrough / black) at m->screen_opacity. At 1.0 this is a no-op (opaque). */
    bool fade = m->screen_opacity < 0.999f;
    glUniform1f(R.uOpacity, m->screen_opacity);
    if (fade) { glEnable(GL_BLEND); glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA); }
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
    if (fade) glDisable(GL_BLEND);
    glUniform1f(R.uOpacity, 1.0f);   /* restore: plaques/HUD/cursor stay opaque */

    /* Status plaques under the centre-column screen, pinned to its frame so they
     * track the wall as you pan: the FPS counter on top, the shortcut cheat-sheet
     * below it. */
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
        if (ci >= 0 && ci < n) {
            screen_t *cs = &m->screen[ci];
            float d      = m->cfg.screen_distance_m;
            float ang_w  = cs->arc_deg * (float)M_PI/180.0f;
            float aspect = (cs->width > 0 && cs->height > 0)
                           ? (float)cs->height / (float)cs->width : 9.0f/16.0f;
            float hh     = d * tanf(ang_w * 0.5f) * aspect;   /* screen half-height */
            float fullH  = 0.11f;
            float yc     = -hh - 0.05f - fullH * 0.5f;        /* FPS row, just below edge */

            glBindBuffer(GL_ARRAY_BUFFER, R.vbo);
            glEnableVertexAttribArray(R.aPos);
            glVertexAttribPointer(R.aPos, 3, GL_FLOAT, GL_FALSE, 5*sizeof(GLfloat), (void*)0);
            glEnableVertexAttribArray(R.aUV);
            glVertexAttribPointer(R.aUV, 2, GL_FLOAT, GL_FALSE, 5*sizeof(GLfloat), (void*)(3*sizeof(GLfloat)));
            glActiveTexture(GL_TEXTURE0);

            /* FPS counter. Re-bake the digits only when the integer value changes
             * (~once a second); otherwise just swap the texture and the MVP. */
            int fv = (int)(m->fps + 0.5f);
            if (fv < 0)   fv = 0;
            if (fv > 999) fv = 999;
            if (fv != R.fps_val) {
                char buf[16]; snprintf(buf, sizeof buf, "FPS %d", fv);
                const float fc[3] = {0.62f, 0.78f, 0.95f};   /* cool blue */
                R.label_fps = bake_label(buf, fc, &R.fps_w, &R.fps_h);
                R.fps_val = fv;
            }
            if (R.label_fps) {
                float fpsW  = fullH * ((float)R.fps_w / (float)R.fps_h);
                mat4 local  = m4_mul(m4_translate(v3(0.0f, yc, -d)),
                                     m4_scale(v3(fpsW, fullH, 1.0f)));
                mat4 model  = m4_mul(layout_model_matrix(m, ci), local);
                glUniformMatrix4fv(R.uMVP, 1, GL_FALSE, m4_mul(vp, model).m);
                glBindTexture(GL_TEXTURE_2D, R.label_fps);
                glUniform1i(R.uTex, 0);
                glUniform1f(R.uHasTex, 1.0f);
                glUniform1f(R.uYFlip, 0.0f);
                glUniform1f(R.uSharpen, 0.0f);
                glDrawArrays(GL_TRIANGLE_STRIP, 0, 4);
            }

            /* Shortcut cheat-sheet, stacked below the FPS row. Static texture,
             * drawn at a smaller scale so the lines don't hang too far down. */
            if (R.label_help) {
                float blockH = 0.26f;                            /* whole block height */
                float blockW = blockH * ((float)R.help_w / (float)R.help_h);
                float fpsBot = yc - fullH * 0.5f;                /* FPS plaque bottom */
                float yc2    = fpsBot - 0.03f - blockH * 0.5f;
                mat4 local2  = m4_mul(m4_translate(v3(0.0f, yc2, -d)),
                                      m4_scale(v3(blockW, blockH, 1.0f)));
                mat4 model2  = m4_mul(layout_model_matrix(m, ci), local2);
                glUniformMatrix4fv(R.uMVP, 1, GL_FALSE, m4_mul(vp, model2).m);
                glBindTexture(GL_TEXTURE_2D, R.label_help);
                glUniform1i(R.uTex, 0);
                glUniform1f(R.uHasTex, 1.0f);
                glUniform1f(R.uYFlip, 0.0f);
                glUniform1f(R.uSharpen, 0.0f);
                glDrawArrays(GL_TRIANGLE_STRIP, 0, 4);
            }
        }
    }

    /* Banner entities (clock, status lines, ...): refresh each (re-bakes only when
     * its key changes) and draw it at its place on the cylinder. World-fixed, so
     * they pan with the wall as you look around. */
    glUseProgram(R.prog);
    for (auto &b : g_banners) {
        banner_refresh(b, m->cfg.screen_distance_m);
        banner_draw(b, vp);
    }

    /* in-view sensitivity slider, hung under the centre screen (grab.c drives it) */
    draw_sens_panel(m, vp);

    /* 3D pointer: an arrow billboard at the cursor's wall direction (grab.c). It sits
     * on the same cylinder as the screens - position = yaw rotation of (0,0,-d) plus
     * a vertical d*tan(pitch) - so it faces the eye like the panels do. Drawn last,
     * depth test OFF (always on top) and ADDITIVE (black bg adds nothing on the
     * optics; only the white arrow glows), so it shows over screens and in the gaps.
     * The quad is shifted so the arrow's TIP lands exactly on the cursor point. */
    if (m->cursor_have && m->cursor_in_gap && R.cursor_tex && !calib_active(m)) {
        float d   = m->cfg.screen_distance_m;
        float hgt = d * tanf(m->cursor_pitch);
        float sz  = 0.055f;                       /* arrow size on the wall (m) */
        mat4 place = m4_mul(m4_translate(v3(0.0f, hgt, 0.0f)),
                            m4_from_quat(q_from_euler_ypr(m->cursor_yaw, 0, 0)));
        /* tip offset: the quad spans [-0.5,0.5]; tip UV (u,v) sits at local
         * (u-0.5, 0.5-v). Translate by its negative (scaled) so the tip is at -d. */
        float tipx = -(CURSOR_TIP_U - 0.5f) * sz;
        float tipy = -(0.5f - CURSOR_TIP_V) * sz;
        mat4 local = m4_mul(m4_translate(v3(tipx, tipy, -d)), m4_scale(v3(sz, sz, 1.0f)));
        mat4 mvp   = m4_mul(vp, m4_mul(place, local));
        glUniformMatrix4fv(R.uMVP, 1, GL_FALSE, mvp.m);
        glBindBuffer(GL_ARRAY_BUFFER, R.vbo);
        glEnableVertexAttribArray(R.aPos);
        glVertexAttribPointer(R.aPos, 3, GL_FLOAT, GL_FALSE, 5*sizeof(GLfloat), (void*)0);
        glEnableVertexAttribArray(R.aUV);
        glVertexAttribPointer(R.aUV, 2, GL_FLOAT, GL_FALSE, 5*sizeof(GLfloat), (void*)(3*sizeof(GLfloat)));
        glActiveTexture(GL_TEXTURE0);
        glBindTexture(GL_TEXTURE_2D, R.cursor_tex);
        glUniform1i(R.uTex, 0);
        glUniform1f(R.uHasTex, 1.0f);
        glUniform1f(R.uYFlip, 0.0f);
        glUniform1f(R.uSharpen, 0.0f);
        glDisable(GL_DEPTH_TEST);
        glEnable(GL_BLEND);
        glBlendFunc(GL_ONE, GL_ONE);
        glDrawArrays(GL_TRIANGLE_STRIP, 0, 4);
        glDisable(GL_BLEND);
        glEnable(GL_DEPTH_TEST);
    }

    /* calibration overlay last of all, head-locked, on top of the scene. */
    calib_draw(m, proj);

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
    /* Free every GL object while the context is still current - the RAII handles
     * in R would otherwise destruct at static-destruction time, after the
     * eglTerminate below, on a dead context. Also drop the per-screen meshes
     * (built here, not by capture). */
    R.prog.reset();   R.vbo.reset();
    R.dome_prog.reset(); R.dome_vbo.reset(); R.hdri_tex.reset();
    R.label_flat.reset(); R.label_curved.reset(); R.label_fps.reset();
    R.label_help.reset();
    for (int i = 0; i < BG_MODE_COUNT; i++) R.label_bg[i].reset();
    for (int i = 0; i < TRACK_MODE_COUNT; i++) R.label_tk[i].reset();
    R.label_trans.reset(); R.cam_tex.reset();
    if (m->cam) { cam_stop(m->cam); m->cam = nullptr; }   /* stop the capture thread */
    g_banners.clear();   /* frees each banner's vbo/tex while the context is live */
    for (int i = 0; i < m->n_screen; i++) {
        if (m->screen[i].mesh_vbo) { glDeleteBuffers(1, &m->screen[i].mesh_vbo); m->screen[i].mesh_vbo = 0; }
    }
    eglMakeCurrent(m->edpy, EGL_NO_SURFACE, EGL_NO_SURFACE, EGL_NO_CONTEXT);
    if (m->esurf != EGL_NO_SURFACE) eglDestroySurface(m->edpy, m->esurf);
    if (m->ectx  != EGL_NO_CONTEXT) eglDestroyContext(m->edpy, m->ectx);
    if (m->egl_window) wl_egl_window_destroy(m->egl_window);
    eglTerminate(m->edpy);
}
