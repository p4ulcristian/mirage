/* worldvio.cpp - world-cam visual motion -> lean/sway parallax (6DoF-lite). See worldvio.h.
 *
 * Two interchangeable backends (pick with $MIRAGE_WORLDVIO = proj | cv):
 *   proj : dependency-free integral projection (correlate row/col intensity profiles of
 *          consecutive downsampled frames -> one global image shift). Cheap, runs inline
 *          in the render thread. Crude: one rigid shift, fooled by moving objects.
 *   cv   : OpenCV sparse optical flow (goodFeaturesToTrack + Lucas-Kanade) on a WORKER
 *          thread (so the heavier CV never blocks rendering). Tracks many points, takes
 *          the robust median flow (outlier-resistant). Logs per-frame CV time so we can
 *          see whether it lags.
 * Both then subtract the IMU-rotation-predicted shift (head-quat delta + cam HFOV) and
 * integrate the translation residual with a leak (bleeds off monocular drift) into a
 * lateral/vertical eye offset.
 */
#include "worldvio.h"
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <ctime>
#include <vector>
#include <algorithm>
#include <thread>
#include <mutex>
#include <condition_variable>

#include <opencv2/imgproc.hpp>
#include <opencv2/video/tracking.hpp>

#define DSW 160
#define DSH 90
#define SEARCH 20          /* proj: max ± shift searched per axis (downsampled px) */

#define CV_PROC_W   640    /* cv: process width (downsampled for speed); aspect kept */
#define CV_MAX_PTS  220
#define CV_MIN_PTS  40

enum Backend { BK_PROJ = 0, BK_CV = 1 };

struct State {
    bool         on = false;
    Backend      backend = BK_PROJ;
    worldvio_cfg cfg{};
    int          trace = 0;
    double       trace_t = 0;
    long         n = 0;

    std::mutex   pos_mtx;
    vec3         pos{0,0,0};        /* integrated eye translation (m), leaked */

    /* proj backend (render thread) */
    bool   have_prev = false;
    float  col_prev[DSW], row_prev[DSH];
    quat   head_prev{1,0,0,0};
    double t_prev = 0;

    /* cv backend (worker thread) */
    std::thread             worker;
    std::mutex              buf_mtx;
    std::condition_variable buf_cv;
    std::vector<uint8_t>    buf;     /* latest RGB frame handed to the worker */
    int    buf_w = 0, buf_h = 0;
    quat   buf_head{1,0,0,0};
    double buf_t = 0;
    float  buf_hfov = 70.0f;
    bool   buf_new = false;
    bool   worker_run = false;
    bool   want_reset = false;
};
static State W;

static const worldvio_cfg DEFAULTS = {
    .trans_gain = 0.0025f, .leak_tau_s = 0.8f, .flow_clamp = 14.0f,
    .invert_x = false, .invert_y = false,
};

static double mono(void){ struct timespec t; clock_gettime(CLOCK_MONOTONIC,&t);
    return t.tv_sec + t.tv_nsec*1e-9; }

/* Common: turn a measured image shift (dx,dy, in `scale` px = focal-px units of the
 * processing image) into an integrated eye offset, with rotation subtracted + leak. */
static void integrate(float dx_obs, float dy_obs, quat head, double dt, float scale_w,
                      float hfov_deg, const char *tag){
    quat dq = q_mul(head, q_conj(W.head_prev));
    float s = (dq.w < 0) ? -1.0f : 1.0f;
    float yaw_d   = 2.0f * s * dq.y;     /* rad about world up (y)    */
    float pitch_d = 2.0f * s * dq.x;     /* rad about world right (x) */
    float f = (scale_w * 0.5f) / tanf(0.5f * hfov_deg * (float)(M_PI/180.0));
    float dx_rot = -yaw_d   * f;
    float dy_rot =  pitch_d * f;
    float rx = dx_obs - dx_rot, ry = dy_obs - dy_rot;

    float clampf = W.cfg.flow_clamp * (scale_w / DSW);   /* clamp scales with resolution */
    bool ok = fabsf(rx) < clampf && fabsf(ry) < clampf;
    float gain = W.cfg.trans_gain * (DSW / scale_w);     /* gain comparable across backends */
    float leak = expf(-(float)dt / W.cfg.leak_tau_s);
    {
        std::lock_guard<std::mutex> lk(W.pos_mtx);
        if (ok){
            float sx = W.cfg.invert_x ? -1.0f : 1.0f;
            float sy = W.cfg.invert_y ? -1.0f : 1.0f;
            W.pos.x += sx * (-rx) * gain;
            W.pos.y += sy * ( ry) * gain;
        }
        W.pos.x *= leak; W.pos.y *= leak; W.pos.z = 0.0f;
    }
    if (W.trace && mono() - W.trace_t > 0.2){ W.trace_t = mono();
        fprintf(stderr,"[worldvio/%s] obs(% 6.2f % 6.2f) rot(% 6.2f % 6.2f) res(% 6.2f % 6.2f)%s pos(% .3f % .3f) n%ld\n",
                tag, dx_obs,dy_obs, dx_rot,dy_rot, rx,ry, ok?"":" REJ", W.pos.x, W.pos.y, W.n);
    }
    W.head_prev = head;
}

/* ---------------- proj backend (integral projection, inline) ---------------- */

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
static float corr_shift(const float *prev, const float *cur, int n){
    float mp=0,mc=0; for(int i=0;i<n;i++){ mp+=prev[i]; mc+=cur[i]; } mp/=n; mc/=n;
    float best=1e30f; int bs=0; float sad[2*SEARCH+1];
    for (int sft=-SEARCH; sft<=SEARCH; sft++){
        float acc=0; int cnt=0;
        for (int i=0;i<n;i++){ int j=i+sft; if(j<0||j>=n) continue;
            acc += fabsf((cur[i]-mc) - (prev[j]-mp)); cnt++; }
        float v = cnt>0 ? acc/cnt : 1e30f; sad[sft+SEARCH]=v;
        if (v<best){ best=v; bs=sft; }
    }
    float shift=(float)bs; int k=bs+SEARCH;
    if (k>0 && k<2*SEARCH){ float a=sad[k-1], b=sad[k], c=sad[k+1], den=(a-2*b+c);
        if (fabsf(den)>1e-6f){ float d=0.5f*(a-c)/den; if(d>-1&&d<1) shift=bs+d; } }
    return shift;
}
static void feed_proj(const uint8_t *rgb, int w, int h, quat head, double t, float hfov){
    float col[DSW], row[DSH];
    project(rgb, w, h, col, row);
    if (!W.have_prev){ memcpy(W.col_prev,col,sizeof col); memcpy(W.row_prev,row,sizeof row);
        W.head_prev=head; W.t_prev=t; W.have_prev=true; W.n=1; return; }
    double dt = t - W.t_prev;
    if (dt < 0.004 || dt > 0.25){ memcpy(W.col_prev,col,sizeof col); memcpy(W.row_prev,row,sizeof row);
        W.head_prev=head; W.t_prev=t; return; }
    float dx = corr_shift(W.col_prev, col, DSW);
    float dy = corr_shift(W.row_prev, row, DSH);
    integrate(dx, dy, head, dt, (float)DSW, hfov, "proj");
    memcpy(W.col_prev,col,sizeof col); memcpy(W.row_prev,row,sizeof row);
    W.t_prev=t; W.n++;
}

/* ---------------- cv backend (OpenCV LK optical flow, worker thread) ---------------- */

static void cv_worker(void){
    cv::Mat prev_gray;
    std::vector<cv::Point2f> prev_pts;
    double prev_t = 0; quat prev_head{1,0,0,0}; bool have = false;

    std::vector<uint8_t> frame; int fw=0, fh=0; quat fhead; double ft; float fhfov;
    while (true){
        {
            std::unique_lock<std::mutex> lk(W.buf_mtx);
            W.buf_cv.wait(lk, []{ return W.buf_new || !W.worker_run; });
            if (!W.worker_run) break;
            frame = W.buf; fw=W.buf_w; fh=W.buf_h; fhead=W.buf_head; ft=W.buf_t; fhfov=W.buf_hfov;
            W.buf_new = false;
            if (W.want_reset){ prev_gray.release(); prev_pts.clear(); have=false; W.want_reset=false; }
        }
        if (fw<8 || fh<8) continue;
        double t0 = mono();

        /* RGB -> gray, downsample to CV_PROC_W for speed */
        cv::Mat rgb(fh, fw, CV_8UC3, frame.data());
        cv::Mat gray; cv::cvtColor(rgb, gray, cv::COLOR_RGB2GRAY);
        int pw = CV_PROC_W, ph = (int)((float)fh * CV_PROC_W / fw);
        cv::Mat g; cv::resize(gray, g, cv::Size(pw, ph));

        if (!have || (int)prev_pts.size() < CV_MIN_PTS){
            cv::goodFeaturesToTrack(g, prev_pts, CV_MAX_PTS, 0.01, 8);
            g.copyTo(prev_gray); prev_t = ft; prev_head = fhead; have = true;
            W.head_prev = fhead;
            if (W.trace) fprintf(stderr,"[worldvio/cv] (re)detect %zu pts, %.1fms\n",
                                 prev_pts.size(), (mono()-t0)*1e3);
            continue;
        }
        std::vector<cv::Point2f> next_pts;
        std::vector<uchar> status; std::vector<float> err;
        cv::calcOpticalFlowPyrLK(prev_gray, g, prev_pts, next_pts, status, err,
                                 cv::Size(21,21), 3);
        /* robust global flow = median of the well-tracked points' displacements */
        std::vector<float> fxs, fys; std::vector<cv::Point2f> kept;
        for (size_t i=0;i<status.size();i++) if (status[i]){
            fxs.push_back(next_pts[i].x - prev_pts[i].x);
            fys.push_back(next_pts[i].y - prev_pts[i].y);
            kept.push_back(next_pts[i]);
        }
        double cvms = (mono()-t0)*1e3;
        double dt = ft - prev_t;
        if (kept.size() >= 8 && dt > 0.004 && dt < 0.25){
            std::nth_element(fxs.begin(), fxs.begin()+fxs.size()/2, fxs.end());
            std::nth_element(fys.begin(), fys.begin()+fys.size()/2, fys.end());
            float mdx = fxs[fxs.size()/2], mdy = fys[fys.size()/2];
            W.head_prev = prev_head;                 /* integrate() reads head_prev */
            integrate(mdx, mdy, fhead, dt, (float)pw, fhfov, "cv");
            W.n++;
        }
        if (W.trace && mono()-W.trace_t < 0.001)   /* piggyback timing onto the trace line */
            fprintf(stderr,"        cv: %zu pts, %.1fms/frame\n", kept.size(), cvms);

        prev_pts = kept; g.copyTo(prev_gray); prev_t = ft; prev_head = fhead;
        /* periodic timing even without full trace */
        static double last_perf = 0;
        if (mono() - last_perf > 2.0){ last_perf = mono();
            fprintf(stderr,"worldvio/cv: %.1f ms/frame, %zu tracked pts\n", cvms, kept.size()); }
    }
}

static void feed_cv(const uint8_t *rgb, int w, int h, quat head, double t, float hfov){
    std::lock_guard<std::mutex> lk(W.buf_mtx);
    size_t need = (size_t)w*h*3;
    if (W.buf.size() != need) W.buf.resize(need);
    memcpy(W.buf.data(), rgb, need);
    W.buf_w=w; W.buf_h=h; W.buf_head=head; W.buf_t=t; W.buf_hfov=hfov; W.buf_new=true;
    W.buf_cv.notify_one();
}

/* ---------------- public API ---------------- */

void worldvio_start(const worldvio_cfg *cfg){
    W.cfg = cfg ? *cfg : DEFAULTS;
    if (const char *e = getenv("MIRAGE_WORLDVIO_GAIN")) W.cfg.trans_gain = atof(e);
    if (const char *e = getenv("MIRAGE_WORLDVIO_LEAK")) W.cfg.leak_tau_s = atof(e);
    if (const char *e = getenv("MIRAGE_WORLDVIO_INVX")) W.cfg.invert_x = atoi(e) != 0;
    if (const char *e = getenv("MIRAGE_WORLDVIO_INVY")) W.cfg.invert_y = atoi(e) != 0;
    const char *bk = getenv("MIRAGE_WORLDVIO");
    W.backend = (bk && (!strcmp(bk,"cv")||!strcmp(bk,"opencv"))) ? BK_CV : BK_PROJ;
    W.trace = []{ const char *e = getenv("MIRAGE_WORLDVIO_TRACE"); return (e&&*e)?1:0; }();
    W.have_prev = false; W.n = 0; W.pos = vec3{0,0,0}; W.head_prev = quat{1,0,0,0};
    W.on = true;
    if (W.backend == BK_CV){ W.worker_run = true; W.worker = std::thread(cv_worker); }
    fprintf(stderr,"worldvio: backend=%s gain=%.4f leak=%.2fs invX=%d invY=%d trace=%d\n",
            W.backend==BK_CV?"cv(OpenCV LK, worker thread)":"proj(integral projection)",
            W.cfg.trans_gain, W.cfg.leak_tau_s, W.cfg.invert_x, W.cfg.invert_y, W.trace);
}

void worldvio_stop(void){
    W.on = false;
    if (W.backend == BK_CV && W.worker.joinable()){
        { std::lock_guard<std::mutex> lk(W.buf_mtx); W.worker_run = false; W.buf_cv.notify_all(); }
        W.worker.join();
    }
}

void worldvio_feed(const uint8_t *rgb, int w, int h, quat head, double t_sec, float hfov_deg){
    if (!W.on || !rgb || w<8 || h<8) return;
    if (W.backend == BK_CV) feed_cv(rgb, w, h, head, t_sec, hfov_deg);
    else                    feed_proj(rgb, w, h, head, t_sec, hfov_deg);
}

vec3 worldvio_eye_offset(void){
    std::lock_guard<std::mutex> lk(W.pos_mtx);
    return W.on ? W.pos : vec3{0,0,0};
}
bool worldvio_active(void){ return W.on && W.n > 4; }
void worldvio_reset(void){
    std::lock_guard<std::mutex> lk(W.buf_mtx);
    W.have_prev = false; W.n = 0; W.want_reset = true;
    std::lock_guard<std::mutex> lk2(W.pos_mtx); W.pos = vec3{0,0,0};
}
