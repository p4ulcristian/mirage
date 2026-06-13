/*
 * rayneo-bridge - feed RayNeo head tracking into mirage.
 *
 * Links the rayneo driver library (../rayneo-air-pro-4), runs its Madgwick AHRS
 * fusion, and streams the resulting head orientation to mirage as OpenTrack UDP
 * packets on 127.0.0.1:4242 (6 little-endian doubles {x,y,z,yaw,pitch,roll},
 * angles in degrees). mirage's default pose backend reads exactly this.
 *
 *   sudo ./rayneo-bridge                 # 9-axis if calibrated, else 6-axis
 *   sudo ./rayneo-bridge --calibrate     # ONE-TIME: wave the figure-8, save magcal
 *   ./rayneo-bridge --port 4242 --6axis  # (no sudo if 99-rayneo.rules installed)
 *   ./rayneo-bridge --dev /dev/hidraw3
 */
#define _GNU_SOURCE
#include "rayneo.h"
#include "magcal.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <signal.h>
#include <math.h>
#include <time.h>
#include <errno.h>
#include <unistd.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>

static volatile sig_atomic_t g_run = 1;
static void on_sigint(int s) { (void)s; g_run = 0; }

static double now_sec(void) {
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return ts.tv_sec + ts.tv_nsec * 1e-9;
}

/* Magnetometer calibration (the figure-8). The raw magnetometer reads an offset,
 * stretched ellipsoid instead of a sphere centred on zero - hard-iron shifts the
 * centre, soft-iron stretches the axes. We sweep the glasses through every
 * orientation, capture the per-axis min/max, then store the centre as the bias and
 * an axis-gain matrix that equalises the three ranges (a diagonal soft-iron fit -
 * enough to make heading absolute and kill yaw drift). Writes the magcal file the
 * bridge loads on its next normal run. */
static int do_calibrate(rayneo_dev *d, const char *cal_path)
{
    fprintf(stderr,
        "\n=== magnetometer calibration ===\n"
        "Slowly wave the glasses through a big figure-8, rolling and tilting them so\n"
        "they point in EVERY direction (up, down, left, right, upside-down). Keep\n"
        "moving for about 25 seconds.\n\n");

    float mn[3] = {  1e30f,  1e30f,  1e30f };
    float mx[3] = { -1e30f, -1e30f, -1e30f };
    const double DUR = 25.0;
    double t0 = now_sec(), last = 0.0;
    long got = 0;

    while (g_run) {
        double t = now_sec(), elapsed = t - t0;
        if (elapsed >= DUR) break;
        rayneo_imu s;
        if (rayneo_read_imu(d, &s, 200) <= 0) continue;
        for (int k = 0; k < 3; k++) {
            if (s.mag[k] < mn[k]) mn[k] = s.mag[k];
            if (s.mag[k] > mx[k]) mx[k] = s.mag[k];
        }
        got++;
        if (t - last >= 0.4) {
            last = t;
            fprintf(stderr, "\r  %3d%%  keep waving...  spans  x=%.0f y=%.0f z=%.0f    ",
                    (int)(elapsed / DUR * 100.0),
                    mx[0]-mn[0], mx[1]-mn[1], mx[2]-mn[2]);
            fflush(stderr);
        }
    }
    fprintf(stderr, "\n");
    if (got < 200) {
        fprintf(stderr, "calibration: only %ld samples - aborted, nothing saved.\n", got);
        return -1;
    }

    float rng[3], avg = 0.0f;
    for (int k = 0; k < 3; k++) { rng[k] = (mx[k] - mn[k]) * 0.5f; avg += rng[k]; }
    avg /= 3.0f;
    for (int k = 0; k < 3; k++) {
        if (avg < 1e-3f || rng[k] < avg * 0.3f) {
            fprintf(stderr, "calibration: uneven coverage (spans %.0f %.0f %.0f). Wave it in\n"
                    "ALL directions - especially the ones that felt awkward - and retry. Not saved.\n",
                    rng[0]*2, rng[1]*2, rng[2]*2);
            return -1;
        }
    }

    rayneo_magcal cal;
    rayneo_magcal_identity(&cal);
    for (int k = 0; k < 3; k++) {
        cal.bias[k]     = (mx[k] + mn[k]) * 0.5f;   /* hard-iron: ellipsoid centre */
        cal.scale[k][k] = avg / rng[k];             /* soft-iron: equalise axis gains */
    }
    if (rayneo_magcal_save(&cal, cal_path) != 0) {
        fprintf(stderr, "calibration: could not write %s\n", cal_path);
        return -1;
    }
    fprintf(stderr, "calibration: saved to %s\n"
            "Done! Start mirage normally - the bridge now runs 9-axis (absolute heading,\n"
            "no yaw drift).\n", cal_path);
    return 0;
}

int main(int argc, char **argv) {
    int port = 4242, force6 = 0, verbose = 0, calibrate = 0;
    const char *dev_path = NULL, *cal_path = NULL, *host = "127.0.0.1";
    /* beta: Madgwick gain. Lower = trust gyro more = steadier (less accel
     * chasing), at the cost of slower gravity re-alignment. 0.05 was visibly
     * jittery on the head; 0.025 is calm and still self-levels fine.
     * deadband: gyro rates below this (deg/s) are noise when the head is still;
     * soft-subtracted so motion still starts smoothly from zero. */
    float beta = 0.025f, deadband_dps = 0.30f;

    for (int i = 1; i < argc; i++) {
        if      (!strcmp(argv[i], "--port") && i+1 < argc) port = atoi(argv[++i]);
        else if (!strcmp(argv[i], "--dev")  && i+1 < argc) dev_path = argv[++i];
        else if (!strcmp(argv[i], "--cal")  && i+1 < argc) cal_path = argv[++i];
        else if (!strcmp(argv[i], "--host") && i+1 < argc) host = argv[++i];
        else if (!strcmp(argv[i], "--beta") && i+1 < argc) beta = atof(argv[++i]);
        else if (!strcmp(argv[i], "--deadband") && i+1 < argc) deadband_dps = atof(argv[++i]);
        else if (!strcmp(argv[i], "--6axis")) force6 = 1;
        else if (!strcmp(argv[i], "--calibrate")) calibrate = 1;
        else if (!strcmp(argv[i], "--verbose") || !strcmp(argv[i], "-v")) verbose = 1;
        else { fprintf(stderr, "usage: %s [--port N] [--dev /dev/hidrawN] "
                       "[--6axis] [--calibrate] [--beta F] [--deadband DPS] [--cal PATH] "
                       "[--host IP] [-v]\n", argv[0]); return 2; }
    }
    const float deadband_rad = deadband_dps * (float)(M_PI / 180.0);
    if (!cal_path) cal_path = rayneo_magcal_default_path();

    signal(SIGINT, on_sigint);
    signal(SIGTERM, on_sigint);

    /* UDP socket to mirage */
    int fd = socket(AF_INET, SOCK_DGRAM, 0);
    if (fd < 0) { perror("socket"); return 1; }
    struct sockaddr_in dst = {0};
    dst.sin_family = AF_INET;
    dst.sin_port = htons((uint16_t)port);
    if (inet_pton(AF_INET, host, &dst.sin_addr) != 1) {
        fprintf(stderr, "bad host %s\n", host); return 1;
    }

    rayneo_dev *d = dev_path ? rayneo_open_path(dev_path) : rayneo_open();
    if (!d) {
        if (errno == ENODEV)
            fprintf(stderr, "No RayNeo glasses found (%04x:%04x).\n", RAYNEO_VID, RAYNEO_PID);
        else if (errno == EACCES)
            fprintf(stderr, "Permission denied on hidraw. Install 99-rayneo.rules "
                            "(sudo make -C ../rayneo-air-pro-4 install-udev) or run with sudo.\n");
        else perror("open RayNeo");
        return 1;
    }
    fprintf(stderr, "rayneo-bridge: opened %s, streaming to %s:%d\n",
            rayneo_devpath(d), host, port);

    if (rayneo_enable_imu(d) != 0) { perror("enable imu"); rayneo_close(d); return 1; }

    /* One-time magnetometer calibration: sweep the figure-8, save, exit. After this
     * a normal run picks up the file and fuses 9-axis (no more yaw drift). */
    if (calibrate) {
        int rc = do_calibrate(d, cal_path);
        rayneo_disable_imu(d);
        rayneo_close(d);
        close(fd);
        return rc == 0 ? 0 : 1;
    }

    rayneo_magcal cal;
    int use9 = 0;
    if (!force6 && rayneo_magcal_load(&cal, cal_path) == 0) {
        use9 = 1;
        fprintf(stderr, "rayneo-bridge: 9-axis fusion (magcal from %s)\n", cal_path);
    } else {
        rayneo_magcal_identity(&cal);
        fprintf(stderr, "rayneo-bridge: 6-axis fusion (yaw will drift; run "
                        "`sudo ./rayneo-bridge --calibrate` once for absolute heading)\n");
    }

    rayneo_ahrs ahrs;
    rayneo_ahrs_init(&ahrs, beta);
    fprintf(stderr, "rayneo-bridge: beta=%.3f deadband=%.2f deg/s, dt from device tick\n",
            beta, deadband_dps);

    rayneo_imu s;
    double last_log = now_sec();
    uint32_t prev_tick = 0;
    long n = 0;
    int idle = 0;
    int seeded = 0;                            /* AHRS seeded from gravity yet? */
    float gyro_bias[3] = {0.0f, 0.0f, 0.0f};   /* auto-zeroed gyro bias (rad/s) */

    while (g_run) {
        int r = rayneo_read_imu(d, &s, 200);
        if (r <= 0) {
            /* transient timeout or read error: never quit. The IMU stops
             * streaming if anything sent IMU-OFF or the link hiccupped, so
             * re-assert IMU-ON about once a second until frames return. */
            if (++idle % 5 == 0) {
                rayneo_enable_imu(d);
                fprintf(stderr, "rayneo-bridge: no frames, re-enabling IMU...\n");
            }
            continue;
        }
        idle = 0;

        /* Seed the orientation from the first gravity reading so pitch/roll start
         * level-correct instead of sliding into place over several seconds. */
        if (!seeded) { rayneo_ahrs_set_from_accel(&ahrs, s.accel); seeded = 1; }

        /* dt from the device tick (10 kHz / 0.1 ms units), which is far steadier
         * than wall-clock read timing (measured: tick jitters <0.3 ms, wall-clock
         * swings 1.7-4.0 ms = +/-50%). Fall back to the 500 Hz nominal when the
         * tick is frozen (the first frames after IMU-ON share a stamp) or jumps. */
        uint32_t dtick = s.tick - prev_tick;        /* unsigned: wraps cleanly */
        prev_tick = s.tick;
        float dt = (float)dtick * 1e-4f;
        if (dtick == 0 || dt < 0.0005f || dt > 0.01f) dt = 0.002f;

        /* Gyro bias auto-zero: at rest the gyro still reads a small constant
         * bias (~1 deg/s here) that integrates straight into heading drift.
         * When the angular rate is low enough to be "still", slowly track that
         * bias with an EMA (gated so real head motion never poisons it), then
         * subtract the WHOLE bias. A fixed deadband can't do this: a bias above
         * the deadband still leaks through. Also follows slow temp drift. */
        {
            float gmag = sqrtf(s.gyro_rad[0]*s.gyro_rad[0]
                             + s.gyro_rad[1]*s.gyro_rad[1]
                             + s.gyro_rad[2]*s.gyro_rad[2]);
            const float still_rad = 2.5f * (float)(M_PI/180.0);   /* <2.5 deg/s = still */
            if (gmag < still_rad) {
                float lr = dt / 1.5f;            /* ~1.5 s to settle while still */
                if (lr > 0.05f) lr = 0.05f;
                for (int k = 0; k < 3; k++)
                    gyro_bias[k] += lr * (s.gyro_rad[k] - gyro_bias[k]);
            }
            for (int k = 0; k < 3; k++) s.gyro_rad[k] -= gyro_bias[k];
        }

        /* Soft gyro deadband: kill the stationary noise floor that would
         * otherwise integrate into creep, without a velocity step at onset. */
        for (int k = 0; k < 3; k++) {
            float g = s.gyro_rad[k];
            if      (g >  deadband_rad) s.gyro_rad[k] = g - deadband_rad;
            else if (g < -deadband_rad) s.gyro_rad[k] = g + deadband_rad;
            else                        s.gyro_rad[k] = 0.0f;
        }

        if (use9) {
            float m[3];
            rayneo_magcal_apply(&cal, s.mag, m);
            rayneo_ahrs_update9(&ahrs, &s, m, dt);
        } else {
            rayneo_ahrs_update(&ahrs, &s, dt);
        }

        float yaw, pitch, roll;
        rayneo_ahrs_euler(&ahrs, &yaw, &pitch, &roll);

        /* Send the AHRS's native quaternion {w,x,y,z}. We deliberately do NOT
         * use OpenTrack's euler {yaw,pitch,roll} wire format: euler goes singular
         * at pitch +-90 deg (looking straight up/down, e.g. reclined) and spins
         * the view. The quaternion carries the same orientation with no gimbal
         * lock. (yaw/pitch/roll above are still computed for --verbose logging.) */
        double pkt[4] = { ahrs.q[0], ahrs.q[1], ahrs.q[2], ahrs.q[3] };
        if (sendto(fd, pkt, sizeof pkt, 0, (struct sockaddr*)&dst, sizeof dst) < 0 && verbose)
            perror("sendto");

        if (verbose) {
            double t = now_sec();
            if (t - last_log >= 0.25) {
                fprintf(stderr, "ypr % 7.1f % 7.1f % 7.1f | acc % 6.2f % 6.2f % 6.2f | "
                        "gyr % 6.2f % 6.2f % 6.2f\n",
                        yaw, pitch, roll,
                        s.accel[0], s.accel[1], s.accel[2],
                        s.gyro_rad[0], s.gyro_rad[1], s.gyro_rad[2]);
                last_log = t;
            }
        }
        n++;
    }

    fprintf(stderr, "\nrayneo-bridge: stopping (%ld packets sent)\n", n);
    rayneo_disable_imu(d);
    rayneo_close(d);
    close(fd);
    return 0;
}
