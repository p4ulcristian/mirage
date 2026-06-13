#include "pose.h"

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
    bool have_signal;
    bool want_recenter;
    bool smooth_on;       /* runtime A/B toggle; raw passthrough when false  */
    uint64_t last_sample_ms;
    uint32_t sample_count; /* raw samples since last pose_take_sample_count() */
    int fd;          /* listening socket / source fd             */
} P = {
    .lock = PTHREAD_MUTEX_INITIALIZER,
    .raw = {1,0,0,0}, .prev_raw = {1,0,0,0}, .smoothed = {1,0,0,0},
    .reference = {1,0,0,0},
    .smooth_on = true,
    .fd = -1,
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

static uint64_t now_ms(void) {
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (uint64_t)ts.tv_sec * 1000 + ts.tv_nsec / 1000000;
}

/* Push a freshly-decoded raw orientation into shared state. */
static void submit_raw(quat raw) {
    pthread_mutex_lock(&P.lock);
    if (P.want_recenter) {
        P.reference = recenter_ref(raw);
        P.want_recenter = false;
    }
    uint64_t t = now_ms();
    quat prev_smoothed = P.smoothed;   /* before this frame's filter update */
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
        float dt = (float)(t - P.last_sample_ms) / 1000.0f;
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
        float dt = (float)(t - P.last_sample_ms) / 1000.0f;
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
        fprintf(trc, "in[% .4f % .4f % .4f % .4f] rel[% .4f % .4f % .4f % .4f] "
                "sp=%6.2f deg/s gate=%s\n",
                P.smoothed.w, P.smoothed.x, P.smoothed.y, P.smoothed.z,
                rel.w, rel.x, rel.y, rel.z, trace_sp, trace_engaged ? "ON" : "off");
        fflush(trc);
    }
    P.raw = raw;
    P.have_signal = true;
    P.last_sample_ms = t;
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
static void run_opentrack(void) {
    uint8_t buf[256];
    while (P.running) {
        ssize_t n = recv(P.fd, buf, sizeof buf, 0);
        if (n < (ssize_t)(4 * sizeof(double))) continue;
        double d[4];
        memcpy(d, buf, sizeof d);  /* little-endian doubles, x86/arm LE */
        quat dev = {(float)d[0], (float)d[1], (float)d[2], (float)d[3]};
        submit_raw(device_to_mirage(q_norm(dev)));
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
    if (P.cfg.sign_yaw   == 0.0f) P.cfg.sign_yaw   = 1.0f;
    if (P.cfg.sign_pitch == 0.0f) P.cfg.sign_pitch = 1.0f;
    if (P.cfg.sign_roll  == 0.0f) P.cfg.sign_roll  = 1.0f;

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

    P.running = true;
    if (pthread_create(&P.thread, NULL, reader_thread, NULL) != 0) {
        P.running = false;
        if (P.fd >= 0) close(P.fd);
        return -1;
    }
    return 0;
}

void pose_stop(void) {
    if (!P.running) return;
    P.running = false;
    pthread_join(P.thread, NULL);
    if (P.fd >= 0) { close(P.fd); P.fd = -1; }
}

quat pose_latest(void) {
    pthread_mutex_lock(&P.lock);
    quat out = q_norm(q_mul(q_conj(P.reference), P.smoothed));
    pthread_mutex_unlock(&P.lock);
    return out;
}

void pose_recenter(void) {
    pthread_mutex_lock(&P.lock);
    P.want_recenter = true;
    /* apply immediately against the most recent raw too */
    P.reference = recenter_ref(P.raw);
    pthread_mutex_unlock(&P.lock);
}

uint32_t pose_age_ms(void) {
    pthread_mutex_lock(&P.lock);
    uint32_t age = P.have_signal ? (uint32_t)(now_ms() - P.last_sample_ms)
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
