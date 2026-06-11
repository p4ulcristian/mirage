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
#include <opencv2/calib3d.hpp>   /* estimateAffinePartial2D (RANSAC) */

#define DSW 160
#define DSH 90
#define SEARCH 20          /* proj: max ± shift searched per axis (downsampled px) */

#define CV_PROC_W   640    /* cv: process width (downsampled for speed); aspect kept */
#define CV_MAX_PTS  220
#define CV_MIN_PTS  40
#define CV_REFILL   150    /* top up features when the tracked set thins below this (keeps
                            * the set dense -> stable median -> no shimmer from churn) */
#define CV_ERR_MAX  16.0f  /* drop LK points whose tracking error exceeds this */

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
    vec3         pos_smooth{0,0,0}; /* low-passed output (what render reads) - kills shimmer */
    float        conf = 0.0f;       /* 0..1 camera-estimate confidence (RANSAC inliers) */
    /* visual-inertial fusion: IMU accel integrated to fpos/fvel (prediction), corrected
     * toward the camera estimate (pos_smooth). Output = fpos when accel is flowing. */
    vec3         fpos{0,0,0}, fvel{0,0,0};
    bool         imu_on = true;
    float        imu_weight = 1.0f;   /* accel scale (0 = camera-only); tune on hardware */
    float        vel_tau = 0.3f;      /* velocity damping (s) - stops accel runaway */
    float        corr_gain = 0.30f;   /* camera correction pulled into fpos per cam frame */
    double       last_accel_t = 0;    /* for the "no accel -> camera-only" fallback */
    float        deadband = 0.4f;   /* ignore residual flow below this (DSW px) - still-jitter */
    int          method = 0;        /* cv global-motion: 0 = RANSAC similarity, 1 = median */
    /* One-Euro output filter state + params (replaces the fixed EMA) */
    vec3         pos_prev{0,0,0};   /* previous raw pos (for the derivative) */
    float        oe_dx = 0, oe_dy = 0;  /* low-passed derivative */
    float        oe_mincut = 0.7f;  /* cutoff at rest (Hz): lower = steadier/laggier */
    float        oe_beta = 14.0f;   /* speed coupling: higher = snappier on a real sway */
    float        oe_dcut = 1.0f;    /* derivative low-pass cutoff (Hz) */

    /* visual-inertial yaw/pitch DRIFT correction (VIO step 1). The rotation residual
     * (observed flow minus IMU-predicted flow) is integrated SLOWLY into a world-frame
     * yaw/pitch correction: transient lean/parallax is zero-mean and averages out; gyro
     * drift is a DC residual that accumulates. Gated by MIRAGE_VISYAW. */
    int    oc_on = 0;
    float  oc_gain = 0.015f;     /* residual-rotation (rad) -> correction integrated per frame */
    float  oc_clamp = 0.002f;    /* max correction step per frame (rad) - rejects transients */
    float  oc_max = 0.6f;        /* total correction cap (rad, ~34deg) - sanity bound */
    vec3   oricorr{0,0,0};       /* accumulated correction: x = pitch, y = yaw (rad) */

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
    /* trans_gain = 0 -> parallax OFF by default. A view-trace (2026-06-18) measured this
     * monocular optical-flow parallax injecting -26..+10 cm of spurious sideways swim with
     * 245 direction reversals during head turns (the "jumps back and forth" artefact) -
     * single-camera flow can't separate rotation from translation. The neck-model 3DoF+ is
     * stable and swim-free. Opt back in for experiments with MIRAGE_WORLDVIO_GAIN=0.0025. */
    .trans_gain = 0.0f, .leak_tau_s = 3.0f, .flow_clamp = 14.0f,
    .invert_x = false, .invert_y = false,
};

static double mono(void){ struct timespec t; clock_gettime(CLOCK_MONOTONIC,&t);
    return t.tv_sec + t.tv_nsec*1e-9; }

/* One-Euro smoothing factor for a cutoff (Hz) and timestep (s). Higher cutoff -> follow;
 * lower -> hold. The cutoff is raised with signal speed, so it's steady at rest yet snappy
 * in motion - exactly the "solid when still, no lag when moving" we want. */
static float oe_alpha(float cutoff, float dt){
    float tau = 1.0f / (2.0f * (float)M_PI * cutoff);
    return 1.0f / (1.0f + tau / dt);
}

/* Common: turn a measured image shift (dx,dy, in `scale` px = focal-px units of the
 * processing image) into an integrated eye offset, with rotation subtracted + leak. */
static void integrate(float dx_obs, float dy_obs, quat head, double dt, float scale_w,
                      float hfov_deg, const char *tag, float conf){
    quat dq = q_mul(head, q_conj(W.head_prev));
    float s = (dq.w < 0) ? -1.0f : 1.0f;
    float yaw_d   = 2.0f * s * dq.y;     /* rad about world up (y)    */
    float pitch_d = 2.0f * s * dq.x;     /* rad about world right (x) */
    float f = (scale_w * 0.5f) / tanf(0.5f * hfov_deg * (float)(M_PI/180.0));
    float dx_rot = -yaw_d   * f;
    float dy_rot =  pitch_d * f;
    float rx = dx_obs - dx_rot, ry = dy_obs - dy_rot;

    /* rotation residual (PRE-deadband) as a per-frame angle, for the visual yaw/pitch
     * drift correction integrated below. f is focal in processing px, so residual/f = rad. */
    float oc_dyaw   = -rx / f;   /* + = IMU under-reported yaw this frame */
    float oc_dpitch =  ry / f;

    /* deadband: ignore sub-pixel residual (sensor/flow noise) so a still head doesn't
     * shimmer. Scales with the processing resolution. */
    float dead = W.deadband * (scale_w / (float)DSW);
    if (fabsf(rx) < dead) rx = 0.0f;
    if (fabsf(ry) < dead) ry = 0.0f;

    float clampf = W.cfg.flow_clamp * (scale_w / DSW);   /* clamp scales with resolution */
    bool ok = fabsf(rx) < clampf && fabsf(ry) < clampf;
    float gain = W.cfg.trans_gain * (DSW / scale_w);     /* gain comparable across backends */
    /* confidence-adaptive leak: when the camera tracks solidly, hold the lean (long tau);
     * when it starves, decay fast back to the neck model. So a held lean now PERSISTS
     * instead of bleeding off after ~1s. */
    float tau_eff = 0.35f + (W.cfg.leak_tau_s - 0.35f) * conf;
    if (tau_eff < 0.1f) tau_eff = 0.1f;
    float leak = expf(-(float)dt / tau_eff);
    {
        std::lock_guard<std::mutex> lk(W.pos_mtx);
        if (ok){
            float sx = W.cfg.invert_x ? -1.0f : 1.0f;
            float sy = W.cfg.invert_y ? -1.0f : 1.0f;
            /* scale the contribution by confidence: low-confidence frames add little, and
             * the constant leak then fades pos -> 0 (graceful fall back to the neck model). */
            W.pos.x += sx * (-rx) * gain * conf;
            W.pos.y += sy * ( ry) * gain * conf;
        }
        W.pos.x *= leak; W.pos.y *= leak; W.pos.z = 0.0f;
        /* One-Euro output filter: smooths HARD when the head is still (kills residual
         * shimmer to dead-solid) but raises its cutoff with motion speed, so a real sway
         * stays snappy with no lag. The render reads pos_smooth. */
        float dtf = (float)dt; if (dtf < 1e-3f) dtf = 1e-3f;
        float rdx = (W.pos.x - W.pos_prev.x) / dtf;
        float rdy = (W.pos.y - W.pos_prev.y) / dtf;
        float ad = oe_alpha(W.oe_dcut, dtf);
        W.oe_dx += ad * (rdx - W.oe_dx);
        W.oe_dy += ad * (rdy - W.oe_dy);
        float cutx = W.oe_mincut + W.oe_beta * fabsf(W.oe_dx);
        float cuty = W.oe_mincut + W.oe_beta * fabsf(W.oe_dy);
        W.pos_smooth.x += oe_alpha(cutx, dtf) * (W.pos.x - W.pos_smooth.x);
        W.pos_smooth.y += oe_alpha(cuty, dtf) * (W.pos.y - W.pos_smooth.y);
        W.pos_smooth.z = 0.0f;
        W.pos_prev = W.pos;
        /* VI fusion: correct the IMU-integrated fpos toward this camera estimate (kills
         * accel drift). Between camera frames, feed_accel keeps fpos moving at IMU rate. */
        W.fpos.x += W.corr_gain * (W.pos_smooth.x - W.fpos.x);
        W.fpos.y += W.corr_gain * (W.pos_smooth.y - W.fpos.y);
        W.fpos.z = 0.0f;

        /* VIO step 1: integrate the rotation residual SLOWLY into a world-frame yaw/pitch
         * correction. Slow gain + per-frame clamp means zero-mean lean/parallax averages
         * out while the gyro's DC drift accumulates and gets cancelled; scaled by camera
         * confidence so a starved scene stops correcting (and the leak can't run away). */
        if (W.oc_on){
            float kc = W.oc_gain * conf;
            float sy = kc * oc_dyaw, sp = kc * oc_dpitch;
            if (sy >  W.oc_clamp) sy =  W.oc_clamp; else if (sy < -W.oc_clamp) sy = -W.oc_clamp;
            if (sp >  W.oc_clamp) sp =  W.oc_clamp; else if (sp < -W.oc_clamp) sp = -W.oc_clamp;
            W.oricorr.y += sy; W.oricorr.x += sp;
            if (W.oricorr.y >  W.oc_max) W.oricorr.y =  W.oc_max; else if (W.oricorr.y < -W.oc_max) W.oricorr.y = -W.oc_max;
            if (W.oricorr.x >  W.oc_max) W.oricorr.x =  W.oc_max; else if (W.oricorr.x < -W.oc_max) W.oricorr.x = -W.oc_max;
        }
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
    W.conf = 1.0f;   /* projection has no inlier metric; trust it (it's the fallback backend) */
    integrate(dx, dy, head, dt, (float)DSW, hfov, "proj", 1.0f);
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

        if (!have){
            cv::goodFeaturesToTrack(g, prev_pts, CV_MAX_PTS, 0.01, 8);
            g.copyTo(prev_gray); prev_t = ft; prev_head = fhead; have = true;
            W.head_prev = fhead;
            continue;
        }
        std::vector<cv::Point2f> next_pts;
        std::vector<uchar> status; std::vector<float> err;
        cv::calcOpticalFlowPyrLK(prev_gray, g, prev_pts, next_pts, status, err,
                                 cv::Size(21,21), 3);
        /* matched pairs of well-tracked points (status ok + low LK error) */
        std::vector<cv::Point2f> p0, p1;
        for (size_t i=0;i<status.size();i++) if (status[i] && err[i] < CV_ERR_MAX){
            p0.push_back(prev_pts[i]); p1.push_back(next_pts[i]);
        }
        double cvms = (mono()-t0)*1e3;
        double dt = ft - prev_t;
        std::vector<cv::Point2f> kept;
        if (p0.size() >= 12 && dt > 0.004 && dt < 0.25){
            float dx = 0, dy = 0; bool got = false;
            if (W.method == 0){
                /* RANSAC similarity (translation+rot+scale): fits ONE model to all points,
                 * rejecting whole outlier regions (a moving hand, reflections), not just bad
                 * individual vectors. Read the shift at the image CENTRE so head-roll (which
                 * rotates the image about its centre) doesn't leak into translation. */
                std::vector<uchar> inl;
                cv::Mat M = cv::estimateAffinePartial2D(p0, p1, inl, cv::RANSAC, 3.0);
                if (!M.empty()){
                    float cx = pw*0.5f, cy = ph*0.5f;
                    float nx = (float)(M.at<double>(0,0)*cx + M.at<double>(0,1)*cy + M.at<double>(0,2));
                    float ny = (float)(M.at<double>(1,0)*cx + M.at<double>(1,1)*cy + M.at<double>(1,2));
                    dx = nx - cx; dy = ny - cy; got = true;
                    int ni = 0; for (size_t i=0;i<inl.size();i++) if (inl[i]){ kept.push_back(p1[i]); ni++; }
                    /* confidence from the inlier count: <20 -> starving (blank wall/blur),
                     * >70 -> rock solid. Smoothed so it fades, not flickers. */
                    float craw = (ni - 20) / (70.0f - 20.0f); if (craw<0) craw=0; if (craw>1) craw=1;
                    W.conf += 0.15f * (craw - W.conf);
                } else {
                    W.conf += 0.15f * (0.0f - W.conf);   /* RANSAC failed: scene too poor -> fade out */
                }
            }
            if (!got){   /* median (MIRAGE_WORLDVIO_METHOD=median, or RANSAC failed) */
                std::vector<float> fxs, fys;
                for (size_t i=0;i<p0.size();i++){ fxs.push_back(p1[i].x-p0[i].x); fys.push_back(p1[i].y-p0[i].y); }
                std::nth_element(fxs.begin(), fxs.begin()+fxs.size()/2, fxs.end());
                std::nth_element(fys.begin(), fys.begin()+fys.size()/2, fys.end());
                dx = fxs[fxs.size()/2]; dy = fys[fys.size()/2];
            }
            if (kept.empty()) kept = p1;
            if (W.method == 1) W.conf = 1.0f;        /* median A/B has no inlier metric */
            W.head_prev = prev_head;                 /* integrate() reads head_prev */
            integrate(dx, dy, fhead, dt, (float)pw, fhfov, "cv", W.conf);
            W.n++;
        } else {
            kept = p1;
        }

        prev_pts = kept; g.copyTo(prev_gray); prev_t = ft; prev_head = fhead;
        /* keep the tracked set dense so the median stays stable frame-to-frame (kills the
         * churn shimmer). Re-detect a fresh full set for the NEXT frame only - this frame's
         * median already used `kept`, so there's no discontinuity. */
        if ((int)prev_pts.size() < CV_REFILL){
            std::vector<cv::Point2f> fresh;
            cv::goodFeaturesToTrack(g, fresh, CV_MAX_PTS, 0.01, 8);
            if (fresh.size() > prev_pts.size()) prev_pts = fresh;
        }
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
    if (const char *e = getenv("MIRAGE_WORLDVIO_DEAD"))  W.deadband  = atof(e);  /* px (DSW) */
    if (const char *e = getenv("MIRAGE_WORLDVIO_OEMIN")) W.oe_mincut = atof(e);  /* rest cutoff Hz (lower=smoother) */
    if (const char *e = getenv("MIRAGE_WORLDVIO_OEBETA"))W.oe_beta   = atof(e);  /* speed coupling (higher=snappier) */
    if (const char *e = getenv("MIRAGE_WORLDVIO_METHOD")) W.method = strcmp(e,"median")==0 ? 1 : 0;
    if (const char *e = getenv("MIRAGE_WORLDVIO_IMU"))   W.imu_on = atoi(e) != 0;       /* 0 = camera-only */
    if (const char *e = getenv("MIRAGE_WORLDVIO_IMUW"))  W.imu_weight = atof(e);        /* accel scale */
    if (const char *e = getenv("MIRAGE_VISYAW"))         W.oc_on   = atoi(e) != 0;      /* visual yaw/pitch drift correction */
    if (const char *e = getenv("MIRAGE_VISYAW_GAIN"))    W.oc_gain = atof(e);           /* correction integration rate */
    if (const char *e = getenv("MIRAGE_VISYAW_CLAMP"))   W.oc_clamp= atof(e);           /* per-frame correction clamp (rad) */
    const char *bk = getenv("MIRAGE_WORLDVIO");
    W.backend = (bk && (!strcmp(bk,"cv")||!strcmp(bk,"opencv"))) ? BK_CV : BK_PROJ;
    W.trace = []{ const char *e = getenv("MIRAGE_WORLDVIO_TRACE"); return (e&&*e)?1:0; }();
    W.have_prev = false; W.n = 0; W.pos = vec3{0,0,0}; W.pos_smooth = vec3{0,0,0};
    W.pos_prev = vec3{0,0,0}; W.oe_dx = W.oe_dy = 0.0f;
    W.fpos = vec3{0,0,0}; W.fvel = vec3{0,0,0}; W.last_accel_t = 0;
    W.oricorr = vec3{0,0,0};
    W.head_prev = quat{1,0,0,0};
    W.on = true;
    if (W.backend == BK_CV){ W.worker_run = true; W.worker = std::thread(cv_worker); }
    fprintf(stderr,"worldvio: backend=%s method=%s gain=%.4f leak=%.2fs oe(min=%.2f beta=%.1f) dead=%.2f invX=%d invY=%d trace=%d\n",
            W.backend==BK_CV?"cv(OpenCV LK, worker thread)":"proj(integral projection)",
            W.method==0?"ransac":"median",
            W.cfg.trans_gain, W.cfg.leak_tau_s, W.oe_mincut, W.oe_beta, W.deadband,
            W.cfg.invert_x, W.cfg.invert_y, W.trace);
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

void worldvio_feed_accel(vec3 a, double dt){
    if (!W.on || !W.imu_on || dt <= 0 || dt > 0.1) return;
    std::lock_guard<std::mutex> lk(W.pos_mtx);
    float dtf = (float)dt;
    /* integrate accel -> velocity (x/y only; z has no camera correction so we don't trust
     * it), damp the velocity so a bad/biased accel can't run away, integrate -> position. */
    W.fvel.x += a.x * W.imu_weight * dtf;
    W.fvel.y += a.y * W.imu_weight * dtf;
    float vd = expf(-dtf / W.vel_tau);
    W.fvel.x *= vd; W.fvel.y *= vd;
    W.fpos.x += W.fvel.x * dtf;
    W.fpos.y += W.fvel.y * dtf;
    W.fpos.z = 0.0f; W.fvel.z = 0.0f;
    W.last_accel_t = mono();
}

vec3 worldvio_eye_offset(void){
    std::lock_guard<std::mutex> lk(W.pos_mtx);
    if (!W.on) return vec3{0,0,0};
    /* fused output when accel is actually flowing; otherwise transparently fall back to the
     * camera-only estimate (old 4-double bridge, or accel stalled). */
    if (W.imu_on && (mono() - W.last_accel_t) < 0.1) return W.fpos;
    return W.pos_smooth;
}
quat worldvio_ori_correction(void){
    std::lock_guard<std::mutex> lk(W.pos_mtx);
    if (!W.on || !W.oc_on) return quat{1,0,0,0};
    /* world-frame yaw/pitch correction; roll left untouched (gravity handles it) */
    return q_from_euler_ypr(W.oricorr.y, W.oricorr.x, 0.0f);
}
bool worldvio_active(void){ return W.on && W.n > 4; }
float worldvio_confidence(void){ return W.on ? W.conf : 0.0f; }
void worldvio_reset(void){
    std::lock_guard<std::mutex> lk(W.buf_mtx);
    W.have_prev = false; W.n = 0; W.want_reset = true;
    std::lock_guard<std::mutex> lk2(W.pos_mtx);
    W.pos = vec3{0,0,0}; W.pos_smooth = vec3{0,0,0}; W.fpos = vec3{0,0,0}; W.fvel = vec3{0,0,0};
    W.oricorr = vec3{0,0,0};
}
