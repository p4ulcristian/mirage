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

#define DEG2RAD(d) ((d) * (float)(M_PI / 180.0))

/* Recenter reference: neutralise heading (yaw) AND vertical look (pitch) while
 * leaving roll gravity-absolute. Recentering with this brings the screens to
 * your CURRENT eye level (not just heading), yet never tilts them: it's derived
 * from the forward direction, which a roll about forward leaves unchanged, so
 * conj(ref) * orientation collapses to a pure roll at the moment of recenter.
 * (Was yaw-only before, which is why vertical centering didn't work.) */
static quat yaw_pitch_ref(quat q) {
    vec3 f = q_rotate(q, v3(0.0f, 0.0f, -1.0f));   /* where the head is looking */
    float fy = f.y < -1.0f ? -1.0f : (f.y > 1.0f ? 1.0f : f.y);
    float yaw   = atan2f(-f.x, -f.z);
    float pitch = asinf(fy);
    return q_from_euler_ypr(yaw, pitch, 0.0f);
}

static struct {
    pose_config cfg;
    pthread_t   thread;
    pthread_mutex_t lock;
    volatile bool running;

    quat raw;        /* latest raw orientation from the device   */
    quat smoothed;   /* smoothed raw                              */
    quat reference;  /* recenter reference (conj applied on read) */
    bool have_signal;
    bool want_recenter;
    uint64_t last_sample_ms;
    int fd;          /* listening socket / source fd             */
} P = {
    .lock = PTHREAD_MUTEX_INITIALIZER,
    .raw = {1,0,0,0}, .smoothed = {1,0,0,0}, .reference = {1,0,0,0},
    .fd = -1,
};

static uint64_t now_ms(void) {
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (uint64_t)ts.tv_sec * 1000 + ts.tv_nsec / 1000000;
}

/* Push a freshly-decoded raw orientation into shared state. */
static void submit_raw(quat raw) {
    pthread_mutex_lock(&P.lock);
    if (P.want_recenter) {
        P.reference = yaw_pitch_ref(raw);
        P.want_recenter = false;
    }
    if (!P.have_signal) P.smoothed = raw;  /* avoid lerp from identity */
    float s = P.cfg.smoothing;
    if (s <= 0.0f || s >= 1.0f) P.smoothed = raw;
    else P.smoothed = q_nlerp(P.smoothed, raw, s);
    P.raw = raw;
    P.have_signal = true;
    P.last_sample_ms = now_ms();
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
        fprintf(stderr, "pose: bind udp %d failed: %s\n", port, strerror(errno));
        close(fd);
        return -1;
    }
    /* 200ms recv timeout so the thread can observe the stop flag */
    struct timeval tv = {0, 200000};
    setsockopt(fd, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof tv);
    return fd;
}

static void run_opentrack(void) {
    uint8_t buf[256];
    while (P.running) {
        ssize_t n = recv(P.fd, buf, sizeof buf, 0);
        if (n < 48) continue;
        double d[6];
        memcpy(d, buf, sizeof d);  /* little-endian doubles, x86/arm LE */
        float yaw   = DEG2RAD((float)d[3]) * P.cfg.sign_yaw;
        float pitch = DEG2RAD((float)d[4]) * P.cfg.sign_pitch;
        float roll  = DEG2RAD((float)d[5]) * P.cfg.sign_roll;
        submit_raw(q_from_euler_ypr(yaw, pitch, roll));
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
        fprintf(stderr, "pose: bind unix %s failed: %s\n", path, strerror(errno));
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
            fprintf(stderr, "pose: breezy shm backend not yet implemented\n");
            break;
    }
    return NULL;
}

int pose_start(const pose_config *cfg) {
    P.cfg = *cfg;
    if (P.cfg.smoothing <= 0.0f) P.cfg.smoothing = 1.0f;
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
    P.reference = yaw_pitch_ref(P.raw);
    pthread_mutex_unlock(&P.lock);
}

uint32_t pose_age_ms(void) {
    pthread_mutex_lock(&P.lock);
    uint32_t age = P.have_signal ? (uint32_t)(now_ms() - P.last_sample_ms)
                                 : UINT32_MAX;
    pthread_mutex_unlock(&P.lock);
    return age;
}

bool pose_has_signal(void) {
    pthread_mutex_lock(&P.lock);
    bool s = P.have_signal;
    pthread_mutex_unlock(&P.lock);
    return s;
}
