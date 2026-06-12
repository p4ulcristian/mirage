/* mirage-posedump: read the head-pose stream and print it.
 * Use to verify your IMU driver is feeding mirage correctly.
 *
 *   mirage-posedump                 # OpenTrack UDP on 127.0.0.1:4242
 *   mirage-posedump --port 5005     # different UDP port
 *   mirage-posedump --json /tmp/mirage-pose.sock
 */
#include "pose.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <signal.h>
#include <print>

static volatile int stop = 0;
static void on_sig(int s) { (void)s; stop = 1; }

/* Convert quaternion back to yaw/pitch/roll degrees for human-readable output. */
static void quat_to_ypr_deg(quat q, float *yaw, float *pitch, float *roll) {
    /* ZYX-ish extraction matching q_from_euler_ypr */
    float sinp = 2.0f*(q.w*q.x - q.y*q.z);
    if (sinp > 1.0f) sinp = 1.0f;
    if (sinp < -1.0f) sinp = -1.0f;
    *pitch = asinf(sinp) * 180.0f/(float)M_PI;
    *yaw   = atan2f(2.0f*(q.w*q.y + q.x*q.z),
                    1.0f - 2.0f*(q.x*q.x + q.y*q.y)) * 180.0f/(float)M_PI;
    *roll  = atan2f(2.0f*(q.w*q.z + q.x*q.y),
                    1.0f - 2.0f*(q.x*q.x + q.z*q.z)) * 180.0f/(float)M_PI;
}

int main(int argc, char **argv) {
    pose_config cfg = { .backend = POSE_OPENTRACK_UDP, .udp_port = 4242,
                        .smoothing = 1.0f };
    for (int i = 1; i < argc; i++) {
        if (!strcmp(argv[i], "--port") && i+1 < argc)
            cfg.udp_port = atoi(argv[++i]);
        else if (!strcmp(argv[i], "--json") && i+1 < argc) {
            cfg.backend = POSE_JSON_SOCKET;
            cfg.socket_path = argv[++i];
        } else if (!strcmp(argv[i], "--smooth") && i+1 < argc)
            cfg.smoothing = atof(argv[++i]);
    }

    signal(SIGINT, on_sig);
    signal(SIGTERM, on_sig);

    if (pose_start(&cfg) != 0) {
        std::print(stderr, "failed to start pose backend\n");
        return 1;
    }
    std::print(stderr, "listening for pose ({})... move your head; Ctrl-C to quit\n",
            cfg.backend == POSE_OPENTRACK_UDP ? "OpenTrack UDP" : "JSON socket");

    int ticks = 0;
    while (!stop) {
        usleep(100000);  /* 10 Hz */
        quat q = pose_latest();
        float y, p, r;
        quat_to_ypr_deg(q, &y, &p, &r);
        uint32_t age = pose_age_ms();
        std::print("\r{}  quat[{: .3f} {: .3f} {: .3f} {: .3f}]  ypr[{: 7.2f} {: 7.2f} {: 7.2f}]  age={:5}ms  ",
               pose_has_signal() ? "SIGNAL" : "  ----",
               q.w, q.x, q.y, q.z, y, p, r,
               age == UINT32_MAX ? 0u : age);
        fflush(stdout);
        if (++ticks % 50 == 0) std::print("\n");
    }
    std::print("\n");
    pose_stop();
    return 0;
}
