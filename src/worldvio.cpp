/* worldvio.cpp - integral-projection visual motion -> lean/sway parallax. See worldvio.h. */
#include "worldvio.h"
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>

/* Downsampled working resolution for the projection profiles. Small = fast + robust to
 * noise/texture detail; we only want global motion, not detail. */
#define DSW 160
#define DSH 90
#define SEARCH 20          /* max ± shift searched per axis (downsampled px) */

static struct {
    bool         on;
    worldvio_cfg cfg;
    bool         have_prev;
    float        col_prev[DSW], row_prev[DSH];
    quat         head_prev;
    double       t_prev;
    long         n;
    vec3         pos;       /* integrated eye translation (m), leaked */
    int          trace;     /* MIRAGE_WORLDVIO_TRACE */
    double       trace_t;
} W;

static const worldvio_cfg DEFAULTS = {
    .trans_gain = 0.0025f,  /* px(downsampled) residual -> metres; tune on hardware */
    .leak_tau_s = 0.8f,     /* lean parallax decays over ~0.8s (kills monocular drift) */
    .flow_clamp = 14.0f,    /* reject per-frame shifts bigger than this (blur/scene cut) */
    .invert_x = false, .invert_y = false,
};

void worldvio_start(const worldvio_cfg *cfg){
    memset(&W, 0, sizeof W);
    W.cfg = cfg ? *cfg : DEFAULTS;
    /* live tuning without a rebuild (set in scripts/start-mirage.sh while dialing in) */
    if (const char *e = getenv("MIRAGE_WORLDVIO_GAIN")) W.cfg.trans_gain = atof(e);
    if (const char *e = getenv("MIRAGE_WORLDVIO_LEAK")) W.cfg.leak_tau_s = atof(e);
    if (const char *e = getenv("MIRAGE_WORLDVIO_INVX")) W.cfg.invert_x = atoi(e) != 0;
    if (const char *e = getenv("MIRAGE_WORLDVIO_INVY")) W.cfg.invert_y = atoi(e) != 0;
    W.head_prev = (quat){1,0,0,0};
    W.on = true;
    const char *e = getenv("MIRAGE_WORLDVIO_TRACE"); W.trace = (e && *e) ? 1 : 0;
    fprintf(stderr, "worldvio: gain=%.4f leak=%.2fs invX=%d invY=%d trace=%d\n",
            W.cfg.trans_gain, W.cfg.leak_tau_s, W.cfg.invert_x, W.cfg.invert_y, W.trace);
}
void worldvio_stop(void){ W.on = false; }
void worldvio_reset(void){ W.have_prev = false; W.n = 0; W.pos = (vec3){0,0,0}; }
bool worldvio_active(void){ return W.on && W.n > 4; }
vec3 worldvio_eye_offset(void){ return W.on ? W.pos : (vec3){0,0,0}; }

/* Downsample RGB888 -> grayscale integral-projection profiles (column sums, row sums).
 * Nearest-neighbour sampling on a DSW×DSH grid; cheap and good enough for global motion. */
static void project(const uint8_t *rgb, int w, int h, float col[DSW], float row[DSH]){
    for (int i=0;i<DSW;i++) col[i]=0.0f;
    for (int j=0;j<DSH;j++) row[j]=0.0f;
    for (int dy=0; dy<DSH; dy++){
        int sy = (int)((dy + 0.5f) * h / DSH); if (sy>=h) sy=h-1;
        const uint8_t *line = rgb + (size_t)sy * w * 3;
        float rsum = 0.0f;
        for (int dx=0; dx<DSW; dx++){
            int sx = (int)((dx + 0.5f) * w / DSW); if (sx>=w) sx=w-1;
            const uint8_t *p = line + (size_t)sx*3;
            float g = (p[0]*77 + p[1]*150 + p[2]*29) * (1.0f/256.0f);
            col[dx] += g; rsum += g;
        }
        row[dy] = rsum;
    }
}

/* Best integer shift (cur relative to prev) minimising mean-removed SAD over the overlap,
 * refined to sub-pixel by a parabola through the SAD minimum. Returns shift in px. */
static float corr_shift(const float *prev, const float *cur, int n){
    /* mean-subtract to ignore overall brightness drift */
    float mp=0,mc=0; for(int i=0;i<n;i++){ mp+=prev[i]; mc+=cur[i]; } mp/=n; mc/=n;
    float best=1e30f; int bs=0;
    float sad[2*SEARCH+1];
    for (int s=-SEARCH; s<=SEARCH; s++){
        float acc=0; int cnt=0;
        for (int i=0;i<n;i++){ int j=i+s; if(j<0||j>=n) continue;
            acc += fabsf((cur[i]-mc) - (prev[j]-mp)); cnt++; }
        float v = cnt>0 ? acc/cnt : 1e30f;
        sad[s+SEARCH]=v;
        if (v<best){ best=v; bs=s; }
    }
    /* parabolic sub-pixel refine around the integer minimum */
    float shift=(float)bs;
    int k=bs+SEARCH;
    if (k>0 && k<2*SEARCH){
        float a=sad[k-1], b=sad[k], c=sad[k+1], den=(a-2*b+c);
        if (fabsf(den)>1e-6f){ float d=0.5f*(a-c)/den; if(d>-1&&d<1) shift=bs+d; }
    }
    return shift;
}

void worldvio_feed(const uint8_t *rgb, int w, int h, quat head, double t_sec, float hfov_deg){
    if (!W.on || !rgb || w<8 || h<8) return;
    float col[DSW], row[DSH];
    project(rgb, w, h, col, row);

    if (!W.have_prev){
        memcpy(W.col_prev, col, sizeof col); memcpy(W.row_prev, row, sizeof row);
        W.head_prev = head; W.t_prev = t_sec; W.have_prev = true; W.n = 1;
        return;
    }
    double dt = t_sec - W.t_prev;
    if (dt < 0.004 || dt > 0.25){ /* stall / bad timing: reseed, don't integrate */
        memcpy(W.col_prev, col, sizeof col); memcpy(W.row_prev, row, sizeof row);
        W.head_prev = head; W.t_prev = t_sec; return;
    }

    /* observed global image shift (downsampled px) */
    float dx_obs = corr_shift(W.col_prev, col, DSW);
    float dy_obs = corr_shift(W.row_prev, row, DSH);

    /* IMU-predicted rotational shift: head turned by dq since last frame; the yaw/pitch
     * part sweeps the image. f_ds = focal length in downsampled px. */
    quat dq = q_mul(head, q_conj(W.head_prev));
    float s = (dq.w < 0) ? -1.0f : 1.0f;           /* shortest-arc */
    float yaw_d   = 2.0f * s * dq.y;               /* rad about world up (y)    */
    float pitch_d = 2.0f * s * dq.x;               /* rad about world right (x) */
    float f_ds = (DSW * 0.5f) / tanf(0.5f * hfov_deg * (float)(M_PI/180.0));
    float dx_rot = -yaw_d   * f_ds;                /* yaw right -> features move left */
    float dy_rot =  pitch_d * f_ds;

    /* residual = translation-induced flow (parallax) */
    float rx = dx_obs - dx_rot;
    float ry = dy_obs - dy_rot;

    /* reject implausible jumps (motion blur, scene cuts, correlation failure) */
    bool ok = fabsf(rx) < W.cfg.flow_clamp && fabsf(ry) < W.cfg.flow_clamp;
    if (ok){
        float sx = W.cfg.invert_x ? -1.0f : 1.0f;
        float sy = W.cfg.invert_y ? -1.0f : 1.0f;
        /* head moves right -> features move left (rx<0) -> world +x. head moves up ->
         * features move down (ry>0) -> world +y. */
        W.pos.x += sx * (-rx) * W.cfg.trans_gain;
        W.pos.y += sy * ( ry) * W.cfg.trans_gain;
    }
    /* leak toward rest: monocular flow integration drifts, so bleed it off (a sustained
     * lean decays, but transient sway parallax reads correctly). */
    float leak = expf(-(float)dt / W.cfg.leak_tau_s);
    W.pos.x *= leak; W.pos.y *= leak; W.pos.z = 0.0f;   /* z (depth) not estimated yet */

    if (W.trace && t_sec - W.trace_t > 0.2){
        W.trace_t = t_sec;
        fprintf(stderr, "[worldvio] obs(% 6.2f % 6.2f) rot(% 6.2f % 6.2f) res(% 6.2f % 6.2f)%s "
                "pos(% .3f % .3f) n%ld\n", dx_obs,dy_obs, dx_rot,dy_rot, rx,ry, ok?"":" REJ",
                W.pos.x, W.pos.y, W.n);
    }

    memcpy(W.col_prev, col, sizeof col); memcpy(W.row_prev, row, sizeof row);
    W.head_prev = head; W.t_prev = t_sec; W.n++;
}
