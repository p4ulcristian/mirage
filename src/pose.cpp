#include "pose.h"
#include "diag.h"
#include "worldvio.h"   /* feed the IMU accel into the world-cam VI fusion */

#include <pthread.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <time.h>
#include <errno.h>
#include <sys/socket.h>
#include <sys/un.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <print>

#define DEG2RAD(d) ((d) * (float)(M_PI / 180.0))

/* Recenter reference: the FULL head orientation at the moment of recenter.
 * conj(ref) * orientation is then identity when you look exactly where you
 * recentered, in ANY posture — sitting, reclined, or flat on your back looking
 * at the ceiling. The old version decomposed the forward vector into yaw+pitch
 * euler, but "yaw" is undefined when forward points straight up/down (lying
 * down): atan2(~0, ~0) picks a garbage heading, which then got amplified into a
 * spin. Taking the whole quaternion has no such singularity — "forward" is just
 * wherever you're looking, no heading to compute. Horizon-levelling is no longer
 * baked into the reference; it's handled downstream by roll_damp, relative to
 * this posture, so a tilted-head recenter no longer forces a gravity horizon. */
static quat recenter_ref(quat q) {
    return q_norm(q);
}

/* One-Euro filter state for a single scalar (used per position axis). Unlike a fixed
 * EMA, its cutoff rises with signal speed, so it holds steady at rest yet doesn't lag
 * a real lean — the right tool for jittery webcam position at 100+ fps. */
struct oneeuro_s { bool init; float xhat, dxhat; };

static struct {
    pose_config cfg;
    pthread_t   thread;
    pthread_mutex_t lock;
    volatile bool running;

    quat raw;        /* latest raw orientation from the device   */
    quat prev_raw;   /* previous raw, for the One-Euro derivative */
    quat smoothed;   /* smoothed raw                              */
    quat reference;  /* recenter reference (conj applied on read) */
    float speed_lp;  /* low-passed angular speed (rad/s)          */
    quat  dvel;      /* smoothed per-sample orientation step (for prediction) */
    float sample_dt; /* EMA of the inter-sample period (s)        */
    bool have_signal;
    bool want_recenter;
    bool smooth_on;       /* runtime A/B toggle; raw passthrough when false  */
    uint64_t last_sample_us;
    uint32_t sample_count; /* raw samples since last pose_take_sample_count() */
    uint32_t recenter_gen; /* bumps on every recenter; lets render reseed its deadband */
    int fd;          /* listening socket / source fd             */

    /* Facecam position channel (independent of the orientation backend above). */
    pthread_t pos_thread;
    int   pos_fd;          /* unix-dgram listening socket for {"pos":[x,y,z]}  */
    bool  pos_have;        /* a position sample has arrived                    */
    bool  pos_ref_set;     /* reference captured (first sample / recenter)     */
    vec3  pos_raw;         /* latest decoded position (camera/world frame, m)  */
    vec3  pos_smoothed;    /* One-Euro-filtered position                       */
    vec3  pos_ref;         /* reference subtracted on read (rest = origin)     */
    uint64_t pos_last_us;  /* timestamp of previous position sample (for dt)   */
    oneeuro_s oe_px, oe_py, oe_pz;  /* per-axis One-Euro filters               */

    /* Visual-inertial complementary filter (the VOR-style fusion): the IMU's linear
     * acceleration carries the FAST position (low latency), the camera anchors the
     * ABSOLUTE position (kills the accel's integration drift). fpos/fvel are the fused
     * world-frame state in metres / m·s⁻¹. */
    bool  fuse_have;       /* at least one accel sample integrated            */
    uint64_t accel_last_us;
    vec3  fvel;            /* fused velocity (mirage world, m/s)              */
    vec3  fpos;            /* fused position (mirage world, m)                */

    /* diagnostics (jumps-while-still log; see diag.h) */
    vec3  diag_prev_fpos;   bool diag_fpos_init;
    vec3  diag_prev_campos; bool diag_campos_init;
    uint64_t diag_last_hb_us; uint32_t diag_hb_count;
} P = {
    .lock = PTHREAD_MUTEX_INITIALIZER,
    .raw = {1,0,0,0}, .prev_raw = {1,0,0,0}, .smoothed = {1,0,0,0},
    .reference = {1,0,0,0},
    .dvel = {1,0,0,0}, .sample_dt = 0.003f,
    .smooth_on = true,
    .fd = -1,
    .pos_fd = -1,
};

/* One-Euro low-pass smoothing factor for a given cutoff (Hz) and timestep (s):
 * alpha = 1 / (1 + tau/dt), tau = 1/(2*pi*fc). Higher fc or dt -> alpha -> 1
 * (follow the signal); lower fc -> alpha -> 0 (hold steady). */
static float oneeuro_alpha(float cutoff_hz, float dt) {
    float tau = 1.0f / (2.0f * (float)M_PI * cutoff_hz);
    return 1.0f / (1.0f + tau / dt);
}

/* Angle (rad) of the shortest rotation between two unit quaternions. */
static float quat_angle(quat a, quat b) {
    float d = a.w*b.w + a.x*b.x + a.y*b.y + a.z*b.z;
    if (d < 0.0f) d = -d;          /* double cover: take the short way */
    if (d > 1.0f) d = 1.0f;
    return 2.0f * acosf(d);
}

/* Microseconds, not milliseconds: at 500 Hz the inter-sample period is 2 ms, so a
 * millisecond clock quantizes consecutive samples into the same tick (dt=0, clamped)
 * or 1/3 ms splits, which makes speed = angle/dt swing wildly and spike the One-Euro
 * cutoff right when the head is still. Microsecond resolution removes that jitter at
 * the source. */
static uint64_t now_us(void) {
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (uint64_t)ts.tv_sec * 1000000 + ts.tv_nsec / 1000;
}

/* Push a freshly-decoded raw orientation into shared state. */
static void submit_raw(quat raw) {
    pthread_mutex_lock(&P.lock);
    if (P.want_recenter) {
        P.reference = recenter_ref(raw);
        P.want_recenter = false;
    }
    uint64_t t = now_us();
    quat prev_smoothed = P.smoothed;   /* before this frame's filter update */
    quat diag_prev_raw = P.prev_raw;   /* before any branch updates it (for the diag) */
    bool had_signal    = P.have_signal;
    if (!P.have_signal) {
        /* first sample: seed every filter state from it, no lerp from identity */
        P.smoothed = raw;
        P.prev_raw = raw;
        P.speed_lp = 0.0f;
    } else if (!P.smooth_on) {
        /* runtime toggle off: pass the raw sample straight through (zero added
         * filter lag). Keep prev_raw current so re-enabling doesn't see a stale
         * derivative and spike the One-Euro cutoff. */
        P.smoothed = raw;
        P.prev_raw = raw;
    } else if (P.cfg.use_oneeuro) {
        /* One-Euro: cutoff (and thus follow speed) rises with head angular
         * speed, so the image locks when you hold still yet keeps up when you
         * turn — the fixed nlerp can only ever be one of those two. */
        float dt = (float)(t - P.last_sample_us) / 1000000.0f;
        if (dt < 1e-4f) dt = 1e-4f;   /* clamp: dup timestamps / clock jumps */
        if (dt > 0.1f)  dt = 0.1f;    /* a long stall shouldn't spike speed */

        float speed = quat_angle(raw, P.prev_raw) / dt;      /* rad/s */
        float ad = oneeuro_alpha(P.cfg.oe_dcutoff, dt);
        P.speed_lp += ad * (speed - P.speed_lp);             /* low-pass it */

        float fc = P.cfg.oe_mincutoff + P.cfg.oe_beta * P.speed_lp;
        float a  = oneeuro_alpha(fc, dt);
        P.smoothed = q_nlerp(P.smoothed, raw, a);
        P.prev_raw = raw;
    } else {
        /* legacy fixed-strength nlerp (selected by --smooth) */
        float s = P.cfg.smoothing;
        if (s <= 0.0f || s >= 1.0f) P.smoothed = raw;
        else P.smoothed = q_nlerp(P.smoothed, raw, s);
    }

    /* Heading-drift compensation. A 6-axis IMU has no absolute heading, so it
     * creeps; the comfort gain (yaw_gain) magnifies that creep into visible
     * drift. When the head is still, fold the slow drift of the smoothed
     * orientation back into the recenter reference: ref <- inc^f * ref, where inc
     * is this frame's world-space change in `smoothed` and f = dt/tau. That holds
     * the *relative* orientation (what we render) steady without yanking it to
     * centre, so an intentional off-centre hold (looking at a side screen) stays
     * put too. Real head motion is far above the stillness gate and is left
     * alone. All quaternion math - no euler - so it works in any posture. */
    float trace_sp = P.speed_lp * 180.0f/(float)M_PI; bool trace_engaged = false;
    if (had_signal && P.cfg.drift_comp_tau > 0.0f) {
        float dt = (float)(t - P.last_sample_us) / 1000000.0f;
        if (dt < 1e-4f) dt = 1e-4f;
        if (dt > 0.1f)  dt = 0.1f;
        /* Gate on the LOW-PASSED head speed (P.speed_lp), not one jittery frame:
         * the old single-frame gate flickered off on sensor noise, so drift leaked
         * straight through and the view never held. Engage SMOOTHLY - full hold
         * when still, fading to nothing before a real head turn - so it locks the
         * view dead-centre yet never fights you when you actually look around. */
        const float STILL = 3.0f  * (float)(M_PI/180.0);   /* full hold below 3 deg/s */
        const float MOVE  = 12.0f * (float)(M_PI/180.0);   /* fully released by 12     */
        float w = (MOVE - P.speed_lp) / (MOVE - STILL);
        if (w < 0.0f) w = 0.0f;
        if (w > 1.0f) w = 1.0f;
        trace_engaged = w > 0.0f;
        if (w > 0.0f) {
            float f = (dt / P.cfg.drift_comp_tau) * w;      /* leak rate, scaled by stillness */
            if (f > 1.0f) f = 1.0f;
            quat inc = q_mul(P.smoothed, q_conj(prev_smoothed)); /* world drift step */
            P.reference = q_norm(q_mul(q_scale_angle(inc, f), P.reference));
        }
    }

    /* Diagnostic trace (set MIRAGE_POSE_TRACE=1): localises drift to the IMU vs
     * mirage. Logs the incoming orientation (post axis-remap), the relative
     * orientation we actually render, the measured speed and whether the drift
     * gate fired. Throttled to ~20 Hz. Remove once drift is understood. */
    static FILE *trc = (FILE *)-1;
    if (trc == (FILE *)-1) {
        const char *e = getenv("MIRAGE_POSE_TRACE");
        trc = (e && *e) ? fopen("/tmp/mirage-pose-trace.log", "w") : NULL;
    }
    if (trc && (P.sample_count % 25 == 0)) {
        quat rel = q_norm(q_mul(q_conj(P.reference), P.smoothed));
        /* dt_us = real inter-sample arrival gap. If these are frequently <1000 or
         * clumped, a millisecond clock would have quantized them into speed spikes
         * (the microsecond timebase fixes that); if they hover near a clean period,
         * the timebase change is just insurance. */
        unsigned long long dt_us = had_signal ? (unsigned long long)(t - P.last_sample_us) : 0;
        fprintf(trc, "in[% .4f % .4f % .4f % .4f] rel[% .4f % .4f % .4f % .4f] "
                "sp=%6.2f deg/s dt=%5lluus gate=%s\n",
                P.smoothed.w, P.smoothed.x, P.smoothed.y, P.smoothed.z,
                rel.w, rel.x, rel.y, rel.z, trace_sp, dt_us, trace_engaged ? "ON" : "off");
        fflush(trc);
    }
    /* Angular velocity for forward-prediction: the per-sample change in the
     * smoothed orientation (world frame), lightly smoothed, stored as the increment
     * quaternion plus its timestep. pose_predicted() extends this to the motion-to-
     * photon horizon with q_scale_angle, so the wall stays nailed to the world while
     * you turn (the difference between readable and smeary text in motion). */
    if (had_signal) {
        float vdt = (float)(t - P.last_sample_us) / 1000000.0f;
        if (vdt < 1e-4f) vdt = 1e-4f;
        if (vdt > 0.1f)  vdt = 0.1f;
        quat dq = q_mul(P.smoothed, q_conj(prev_smoothed));   /* world-frame step */
        P.dvel = q_nlerp(P.dvel, dq, 0.5f);                   /* light smoothing  */
        P.sample_dt += 0.05f * (vdt - P.sample_dt);
    }

    /* --- diagnostics: orientation jumps while (nearly) still, plus a heartbeat ---
     * A still-jump is a per-sample step in the SMOOTHED orientation that is large
     * while the low-passed head speed is low (so it isn't real motion). raw vs
     * smoothed tells whether the glitch came from the IMU or leaked through the
     * filter. The heartbeat records the inbound rate + live state every ~3 s. */
    if (diag_enabled()) {
        P.diag_hb_count++;
        if (had_signal) {
            float spd_deg = P.speed_lp * 180.0f / (float)M_PI;
            float sm_d  = quat_angle(P.smoothed, prev_smoothed) * 180.0f / (float)M_PI;
            float raw_d = quat_angle(raw, diag_prev_raw)        * 180.0f / (float)M_PI;
            if (sm_d > 0.20f && spd_deg < 4.0f) {
                float dt_ms = (float)(t - P.last_sample_us) / 1000.0f;
                diag_logf("ORI", "jump sm=%.3fdeg raw=%.3fdeg spd=%.2fdeg/s dt=%.2fms "
                          "sm=[% .4f % .4f % .4f % .4f]",
                          sm_d, raw_d, spd_deg, dt_ms,
                          P.smoothed.w, P.smoothed.x, P.smoothed.y, P.smoothed.z);
            }
        }
        if (t - P.diag_last_hb_us > 3000000ull) {
            float el = P.diag_last_hb_us ? (float)(t - P.diag_last_hb_us) / 1e6f : 1.0f;
            diag_logf("HB", "imu=%.0fHz spd=%.2fdeg/s fpos=(%.3f,%.3f,%.3f) "
                      "fvel=(%.3f,%.3f,%.3f) cam=(%.3f,%.3f,%.3f) have_cam=%d fuse=%d",
                      (float)P.diag_hb_count / el, P.speed_lp * 180.0f / (float)M_PI,
                      P.fpos.x, P.fpos.y, P.fpos.z, P.fvel.x, P.fvel.y, P.fvel.z,
                      P.pos_smoothed.x, P.pos_smoothed.y, P.pos_smoothed.z,
                      (int)P.pos_have, (int)P.fuse_have);
            P.diag_last_hb_us = t;
            P.diag_hb_count = 0;
        }
    }

    P.raw = raw;
    P.have_signal = true;
    P.last_sample_us = t;
    P.sample_count++;
    pthread_mutex_unlock(&P.lock);
}

/* ---- OpenTrack UDP backend ---- */
static int open_udp(int port) {
    int fd = socket(AF_INET, SOCK_DGRAM, 0);
    if (fd < 0) return -1;
    int one = 1;
    setsockopt(fd, SOL_SOCKET, SO_REUSEADDR, &one, sizeof one);
    struct sockaddr_in a = {0};
    a.sin_family = AF_INET;
    a.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
    a.sin_port = htons((uint16_t)port);
    if (bind(fd, (struct sockaddr*)&a, sizeof a) < 0) {
        std::print(stderr, "pose: bind udp {} failed: {}\n", port, strerror(errno));
        close(fd);
        return -1;
    }
    /* 200ms recv timeout so the thread can observe the stop flag */
    struct timeval tv = {0, 200000};
    setsockopt(fd, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof tv);
    return fd;
}

/* Device (aerospace ZYX: yaw=Z, pitch=Y, roll=X) -> mirage (graphics: yaw=Y,
 * pitch=X, roll=Z) is a fixed axis relabel = a 120-deg rotation about (1,1,1).
 * Conjugating the raw quaternion by it is the gimbal-lock-free equivalent of the
 * old euler round-trip (ahrs_euler -> q_from_euler_ypr), verified to match it to
 * <0.07 deg everywhere AND to stay correct at pitch +-90 where euler spins. */
static quat device_to_mirage(quat dev) {
    const quat B = {0.5f, -0.5f, -0.5f, -0.5f};
    return q_norm(q_mul(q_mul(B, dev), q_conj(B)));
}

/* UDP backend: the rayneo bridge streams the head orientation as a raw
 * quaternion {w,x,y,z} of 4 little-endian doubles. We use the quaternion (not
 * OpenTrack euler) end-to-end precisely so there is no gimbal lock when reclined.
 * device_to_mirage applies the fixed device->mirage axis change; that is all. */
static void submit_accel(vec3 a);   /* defined below, after submit_pos */

static void run_opentrack(void) {
    uint8_t buf[256];
    while (P.running) {
        ssize_t n = recv(P.fd, buf, sizeof buf, 0);
        if (n < (ssize_t)(4 * sizeof(double))) continue;
        size_t have = (size_t)n / sizeof(double);
        double d[7];
        memcpy(d, buf, (have >= 7 ? 7 : 4) * sizeof(double));  /* LE doubles, x86/arm */
        quat dev = {(float)d[0], (float)d[1], (float)d[2], (float)d[3]};
        submit_raw(device_to_mirage(q_norm(dev)));
        /* optional 3 extra doubles (newer bridge) = earth-frame linear acceleration for
         * the visual-inertial complementary filter. Remap earth->mirage world with the
         * SAME B as the orientation, then fuse. Old 4-double bridges just skip this and
         * mirage falls back to the camera-only position path. */
        if (have >= 7) {
            vec3 a_earth = {(float)d[4], (float)d[5], (float)d[6]};
            const quat B = {0.5f, -0.5f, -0.5f, -0.5f};   /* earth -> mirage world (== orientation remap) */
            vec3 a_mir = q_rotate(B, a_earth);
            if (P.cfg.facecam_enable && P.cfg.facecam_fusion) submit_accel(a_mir);
            /* feed the world-cam VI fusion: IMU position prediction between camera frames */
            static uint64_t pa = 0; uint64_t nu = now_us();
            double adt = pa ? (double)(nu - pa) * 1e-6 : 0.0; pa = nu;
            if (adt > 0) worldvio_feed_accel(a_mir, adt);
            /* VALIDATION: the bridge's new world-frame linear-accel feed for the VIO fusion.
             * Should be ~0 at rest and spike (a few m/s^2) on real head motion. Throttled;
             * remove once verified. */
            static double alt = 0; double tt = now_us() * 1e-6;
            if (tt - alt > 0.3) { alt = tt;
                std::print(stderr, "[accel] earth({:6.2f} {:6.2f} {:6.2f}) |a|={:.2f}\n",
                           a_earth.x, a_earth.y, a_earth.z,
                           sqrtf(a_earth.x*a_earth.x + a_earth.y*a_earth.y + a_earth.z*a_earth.z)); }
        }
    }
}

/* ---- JSON unix-dgram backend: {"quat":[w,x,y,z]} ---- */
static int open_unix_dgram(const char *path) {
    int fd = socket(AF_UNIX, SOCK_DGRAM, 0);
    if (fd < 0) return -1;
    struct sockaddr_un a = {0};
    a.sun_family = AF_UNIX;
    strncpy(a.sun_path, path, sizeof a.sun_path - 1);
    unlink(path);
    if (bind(fd, (struct sockaddr*)&a, sizeof a) < 0) {
        std::print(stderr, "pose: bind unix {} failed: {}\n", path, strerror(errno));
        close(fd);
        return -1;
    }
    struct timeval tv = {0, 200000};
    setsockopt(fd, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof tv);
    return fd;
}

static bool parse_quat_json(const char *s, quat *out) {
    const char *p = strstr(s, "\"quat\"");
    if (!p) return false;
    p = strchr(p, '[');
    if (!p) return false;
    float v[4];
    if (sscanf(p, "[ %f , %f , %f , %f ]", &v[0], &v[1], &v[2], &v[3]) != 4)
        return false;
    *out = q_norm((quat){v[0], v[1], v[2], v[3]});
    return true;
}

static void run_json_socket(void) {
    char buf[512];
    while (P.running) {
        ssize_t n = recv(P.fd, buf, sizeof buf - 1, 0);
        if (n <= 0) continue;
        buf[n] = 0;
        quat q;
        if (parse_quat_json(buf, &q)) submit_raw(q);
    }
}

/* ---- Facecam position channel: unix-dgram JSON {"pos":[x,y,z]} ---- */
static bool parse_pos_json(const char *s, vec3 *out) {
    const char *p = strstr(s, "\"pos\"");
    if (!p) return false;
    p = strchr(p, '[');
    if (!p) return false;
    float v[3];
    if (sscanf(p, "[ %f , %f , %f ]", &v[0], &v[1], &v[2]) != 3) return false;
    *out = (vec3){v[0], v[1], v[2]};
    return true;
}

/* One-Euro step for one scalar: low-pass whose cutoff = mincut + beta*|speed|, so it
 * follows fast motion with little lag but smooths hard when nearly still. */
static float oneeuro_step(oneeuro_s *s, float x, float dt, float mincut, float beta) {
    if (!s->init) { s->init = true; s->xhat = x; s->dxhat = 0.0f; return x; }
    float dx = (x - s->xhat) / dt;
    float ad = oneeuro_alpha(1.0f, dt);            /* derivative cutoff = 1 Hz */
    s->dxhat += ad * (dx - s->dxhat);
    float cutoff = mincut + beta * fabsf(s->dxhat);
    float a = oneeuro_alpha(cutoff, dt);
    s->xhat += a * (x - s->xhat);
    return s->xhat;
}

/* Fold a freshly decoded head position into shared state. The first sample (and any
 * recenter) seeds the reference, so pose_position() reads zero at rest. One-Euro
 * filtered per axis (face detection is jittery, especially z's apparent-size depth);
 * depth gets a lower rest cutoff than x/y, so it's held steadier at the cost of a
 * touch more lag. facecam_smooth sets the lateral rest cutoff (Hz, lower = steadier). */
static void submit_pos(vec3 raw) {
    pthread_mutex_lock(&P.lock);
    uint64_t t = now_us();
    if (!P.pos_have) {
        P.pos_smoothed = raw;          /* seed filters, no lerp from a garbage origin */
        oneeuro_step(&P.oe_px, raw.x, 0.01f, 1.0f, 0.0f);
        oneeuro_step(&P.oe_py, raw.y, 0.01f, 1.0f, 0.0f);
        oneeuro_step(&P.oe_pz, raw.z, 0.01f, 1.0f, 0.0f);
        P.pos_have = true;
    } else {
        float dt = (float)(t - P.pos_last_us) / 1000000.0f;
        if (dt < 1e-3f) dt = 1e-3f;
        if (dt > 0.2f)  dt = 0.2f;
        float mincut = P.cfg.facecam_smooth;       /* lateral rest cutoff (Hz) */
        if (mincut <= 0.0f) mincut = 1.2f;
        const float beta = 0.6f;                   /* speed coupling (less lag in motion) */
        P.pos_smoothed.x = oneeuro_step(&P.oe_px, raw.x, dt, mincut, beta);
        P.pos_smoothed.y = oneeuro_step(&P.oe_py, raw.y, dt, mincut, beta);
        P.pos_smoothed.z = oneeuro_step(&P.oe_pz, raw.z, dt, mincut * 0.5f, beta);
    }
    P.pos_raw = raw;
    P.pos_last_us = t;
    if (!P.pos_ref_set) { P.pos_ref = P.pos_smoothed; P.pos_ref_set = true; }
    /* diag: a big step in the SMOOTHED camera position = a facecam misdetection that
     * slipped past the median + spike gate (the prime suspect for a lateral still-jump). */
    if (diag_enabled() && P.diag_campos_init) {
        float d = v3_len(v3_sub(P.pos_smoothed, P.diag_prev_campos));
        if (d > 0.02f)
            diag_logf("CAM", "jump d=%.4fm cam=(%.3f,%.3f,%.3f) raw=(%.3f,%.3f,%.3f)",
                      d, P.pos_smoothed.x, P.pos_smoothed.y, P.pos_smoothed.z,
                      raw.x, raw.y, raw.z);
    }
    P.diag_prev_campos = P.pos_smoothed; P.diag_campos_init = true;
    pthread_mutex_unlock(&P.lock);
}

/* One complementary-filter step from a linear-acceleration sample (mirage world frame,
 * gravity already removed by the bridge). PREDICT: integrate accel -> velocity -> position
 * for the fast, low-latency motion, leaking velocity toward zero so a constant accel bias
 * can't run the position away (bias -> bounded velocity instead of unbounded drift).
 * CORRECT: pull the fused position toward the camera's absolute (One-Euro) position over
 * TAU_C, so the slow-but-drift-free camera anchors the fast-but-drifting inertial estimate.
 * The accel and the orientation arrive in the same packet, so they're already time-synced. */
static void submit_accel(vec3 a) {
    pthread_mutex_lock(&P.lock);
    uint64_t t = now_us();
    float dt;
    if (!P.fuse_have) {
        P.fuse_have = true;
        P.fvel = (vec3){0,0,0};
        P.fpos = P.pos_have ? P.pos_smoothed : (vec3){0,0,0};  /* start at the camera */
        dt = 0.002f;
    } else {
        dt = (float)(t - P.accel_last_us) / 1000000.0f;
        if (dt < 1e-4f) dt = 1e-4f;
        if (dt > 0.05f) dt = 0.05f;        /* a stall shouldn't fling the integrator */
    }
    P.accel_last_us = t;

    const float TAU_V = 0.4f;   /* velocity leak (s): bounds accel-bias drift        */
    const float TAU_C = 0.15f;  /* camera correction time const (s): lower = trust cam */
    P.fvel = v3_add(P.fvel, v3_scale(a, dt));        /* integrate accel -> velocity   */
    P.fvel = v3_scale(P.fvel, expf(-dt / TAU_V));    /* leak                          */
    P.fpos = v3_add(P.fpos, v3_scale(P.fvel, dt));   /* integrate velocity -> position*/
    if (P.pos_have) {                                /* correct toward the camera     */
        float kc = dt / TAU_C;
        if (kc > 1.0f) kc = 1.0f;
        P.fpos = v3_add(P.fpos, v3_scale(v3_sub(P.pos_smoothed, P.fpos), kc));
    }
    /* diag: a big per-sample step in the FUSED position = the accel integration spiked
     * or the complementary filter overshot (the prime suspect from the new VI fusion).
     * 6 mm in one ~2 ms sample is physically impossible head motion, so it's a glitch. */
    if (diag_enabled() && P.diag_fpos_init) {
        float d = v3_len(v3_sub(P.fpos, P.diag_prev_fpos));
        if (d > 0.006f)
            diag_logf("FPOS", "jump d=%.4fm fvel=%.3f accel=%.2f fpos=(%.3f,%.3f,%.3f) cam=(%.3f,%.3f,%.3f)",
                      d, v3_len(P.fvel), v3_len(a),
                      P.fpos.x, P.fpos.y, P.fpos.z,
                      P.pos_smoothed.x, P.pos_smoothed.y, P.pos_smoothed.z);
    }
    P.diag_prev_fpos = P.fpos; P.diag_fpos_init = true;
    pthread_mutex_unlock(&P.lock);
}

static void run_position_socket(void) {
    char buf[512];
    while (P.running) {
        ssize_t n = recv(P.pos_fd, buf, sizeof buf - 1, 0);
        if (n <= 0) continue;       /* 200ms timeout -> re-check P.running */
        buf[n] = 0;
        vec3 pos;
        if (parse_pos_json(buf, &pos)) submit_pos(pos);
    }
}

static void *pos_reader_thread(void *arg) {
    (void)arg;
    run_position_socket();
    return NULL;
}

static void *reader_thread(void *arg) {
    (void)arg;
    switch (P.cfg.backend) {
        case POSE_OPENTRACK_UDP: run_opentrack();   break;
        case POSE_JSON_SOCKET:   run_json_socket();  break;
        case POSE_BREEZY_SHM:
            std::print(stderr, "pose: breezy shm backend not yet implemented\n");
            break;
    }
    return NULL;
}

int pose_start(const pose_config *cfg) {
    P.cfg = *cfg;
    if (P.cfg.smoothing <= 0.0f) P.cfg.smoothing = 1.0f;
    if (P.cfg.oe_mincutoff <= 0.0f) P.cfg.oe_mincutoff = 0.5f;
    if (P.cfg.oe_beta      <= 0.0f) P.cfg.oe_beta      = 1.0f;
    if (P.cfg.oe_dcutoff   <= 0.0f) P.cfg.oe_dcutoff   = 1.0f;

    switch (cfg->backend) {
        case POSE_OPENTRACK_UDP:
            P.fd = open_udp(cfg->udp_port ? cfg->udp_port : 4242);
            break;
        case POSE_JSON_SOCKET:
            P.fd = open_unix_dgram(cfg->socket_path ? cfg->socket_path
                                                    : "/tmp/mirage-pose.sock");
            break;
        case POSE_BREEZY_SHM:
            P.fd = -1;  /* shm-based, no fd */
            break;
    }
    if (cfg->backend != POSE_BREEZY_SHM && P.fd < 0) return -1;

    /* Optional facecam position channel: a second unix-dgram socket that the webcam
     * bridge sends head position to. Independent of the orientation backend; if it
     * fails to bind we log and carry on with rotation-only (3DoF). */
    if (cfg->facecam_enable) {
        P.pos_fd = open_unix_dgram(cfg->facecam_socket ? cfg->facecam_socket
                                                       : "/tmp/mirage-facecam.sock");
        if (P.pos_fd < 0)
            std::print(stderr, "pose: facecam disabled (socket bind failed)\n");
    }

    P.running = true;
    if (pthread_create(&P.thread, NULL, reader_thread, NULL) != 0) {
        P.running = false;
        if (P.fd >= 0) close(P.fd);
        if (P.pos_fd >= 0) { close(P.pos_fd); P.pos_fd = -1; }
        return -1;
    }
    if (P.pos_fd >= 0 && pthread_create(&P.pos_thread, NULL, pos_reader_thread, NULL) != 0) {
        std::print(stderr, "pose: facecam thread failed to start\n");
        close(P.pos_fd); P.pos_fd = -1;
    }
    return 0;
}

void pose_stop(void) {
    if (!P.running) return;
    P.running = false;
    pthread_join(P.thread, NULL);
    if (P.pos_fd >= 0) { pthread_join(P.pos_thread, NULL); close(P.pos_fd); P.pos_fd = -1; }
    if (P.fd >= 0) { close(P.fd); P.fd = -1; }
}

quat pose_latest(void) {
    pthread_mutex_lock(&P.lock);
    quat out = q_norm(q_mul(q_conj(P.reference), P.smoothed));
    pthread_mutex_unlock(&P.lock);
    return out;
}

/* Like pose_latest, but extrapolated horizon_s seconds INTO THE FUTURE along the
 * current angular velocity - so what we render matches where the head will be when
 * the photons land, cancelling motion-to-photon latency. horizon_s <= 0 falls back
 * to the present. The extrapolation is capped so a noisy velocity can't fling the
 * view; when still, dvel ~ identity so this is a no-op. */
quat pose_predicted(float horizon_s) {
    pthread_mutex_lock(&P.lock);
    quat smoothed = P.smoothed, ref = P.reference, dvel = P.dvel;
    float sdt = P.sample_dt;
    bool have = P.have_signal;
    pthread_mutex_unlock(&P.lock);
    if (!have) return q_identity();
    if (horizon_s > 1e-4f && sdt > 1e-5f) {
        float scale = horizon_s / sdt;
        if (scale > 8.0f) scale = 8.0f;             /* cap forward extrapolation */
        quat dq = q_scale_angle(dvel, scale);       /* per-sample step -> horizon */
        smoothed = q_mul(dq, smoothed);             /* world-frame left-multiply  */
    }
    return q_norm(q_mul(q_conj(ref), smoothed));
}

void pose_recenter(void) {
    pthread_mutex_lock(&P.lock);
    P.want_recenter = true;
    /* apply immediately against the most recent raw too */
    P.reference = recenter_ref(P.raw);
    /* zero the facecam lean too: current position becomes the new rest origin */
    if (P.pos_have) P.pos_ref = P.pos_smoothed;
    P.recenter_gen++;   /* signal render to reseed its reading-deadband (no slew) */
    pthread_mutex_unlock(&P.lock);
}

static inline float clampf(float v, float lim) {
    return v > lim ? lim : (v < -lim ? -lim : v);
}

/* World-axis eye-offset (metres) from the rest reference. x/y already in mirage world
 * convention from the bridge; z is camera->head distance (smaller = leaned in), so
 * (smoothed - ref) goes negative on a lean-in -> eye moves -Z toward the wall. Each axis
 * is forward-predicted along its One-Euro velocity (oe.*.dxhat) to offset the camera's
 * latency, with the lead capped so a noisy velocity can't fling the view. {0,0,0} until a
 * sample arrives. */
vec3 pose_position(float horizon_s) {
    pthread_mutex_lock(&P.lock);
    vec3 out = {0,0,0};
    float h = horizon_s > 0.0f ? horizon_s : 0.0f;
    const float PCAP = 0.04f;   /* max predicted lead per axis (m) */
    const float LIM  = 0.40f;   /* max total offset (anti-fling)   */
    /* Prefer the fused (visual-inertial) estimate: it carries the camera's absolute
     * position but with the IMU's low latency. Use it only while accel is actually
     * arriving (else fall back to the camera-only One-Euro path). fpos shares the
     * camera's absolute frame (it's continuously corrected toward pos_smoothed), so the
     * same recenter reference applies to both. */
    bool fusing = P.fuse_have && P.cfg.facecam_fusion && P.pos_ref_set &&
                  (now_us() - P.accel_last_us) < 200000ull;
    if (fusing) {
        float px = P.fpos.x + clampf(P.fvel.x * h, PCAP);   /* real velocity lead */
        float py = P.fpos.y + clampf(P.fvel.y * h, PCAP);
        float pz = P.fpos.z + clampf(P.fvel.z * h, PCAP);
        out.x = clampf(px - P.pos_ref.x, LIM);
        out.y = clampf(py - P.pos_ref.y, LIM);
        out.z = clampf(pz - P.pos_ref.z, LIM);
    } else if (P.pos_have && P.pos_ref_set) {
        float px = P.pos_smoothed.x + clampf(P.oe_px.dxhat * h, PCAP);
        float py = P.pos_smoothed.y + clampf(P.oe_py.dxhat * h, PCAP);
        float pz = P.pos_smoothed.z + clampf(P.oe_pz.dxhat * h, PCAP);
        out.x = clampf(px - P.pos_ref.x, LIM);
        out.y = clampf(py - P.pos_ref.y, LIM);
        out.z = clampf(pz - P.pos_ref.z, LIM);
    }
    pthread_mutex_unlock(&P.lock);
    return out;
}

/* Fresh position sample within ~0.5 s? Lets render fall back to the neck model when the
 * webcam drops out (camera busy, head out of frame, bridge down) instead of freezing on
 * a stale lean. */
bool pose_position_active(void) {
    pthread_mutex_lock(&P.lock);
    bool a = P.pos_have && (now_us() - P.pos_last_us) < 500000ull;
    pthread_mutex_unlock(&P.lock);
    return a;
}

uint32_t pose_recenter_gen(void) {
    pthread_mutex_lock(&P.lock);
    uint32_t g = P.recenter_gen;
    pthread_mutex_unlock(&P.lock);
    return g;
}

uint32_t pose_age_ms(void) {
    pthread_mutex_lock(&P.lock);
    uint32_t age = P.have_signal ? (uint32_t)((now_us() - P.last_sample_us) / 1000)
                                 : UINT32_MAX;
    pthread_mutex_unlock(&P.lock);
    return age;
}

uint32_t pose_take_sample_count(void) {
    pthread_mutex_lock(&P.lock);
    uint32_t c = P.sample_count;
    P.sample_count = 0;
    pthread_mutex_unlock(&P.lock);
    return c;
}

bool pose_toggle_smoothing(void) {
    pthread_mutex_lock(&P.lock);
    P.smooth_on = !P.smooth_on;
    bool s = P.smooth_on;
    pthread_mutex_unlock(&P.lock);
    return s;
}

bool pose_smoothing_enabled(void) {
    pthread_mutex_lock(&P.lock);
    bool s = P.smooth_on;
    pthread_mutex_unlock(&P.lock);
    return s;
}

bool pose_has_signal(void) {
    pthread_mutex_lock(&P.lock);
    bool s = P.have_signal;
    pthread_mutex_unlock(&P.lock);
    return s;
}

/* Low-passed head angular speed (rad/s), the same estimate that drives the One-Euro
 * cutoff. render gates the reading-deadband on this: freeze when still, release when
 * panning. 0 until the first sample / when smoothing is off. */
float pose_speed(void) {
    pthread_mutex_lock(&P.lock);
    float s = P.speed_lp;
    pthread_mutex_unlock(&P.lock);
    return s;
}
