#define _GNU_SOURCE
#include "rayneo.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <fcntl.h>
#include <unistd.h>
#include <dirent.h>
#include <poll.h>
#include <math.h>
#include <errno.h>

struct rayneo_dev {
    int   fd;
    char  path[256];
};

/* ---- device discovery ---- */

static int check_hidraw(const char *devpath, int vid, int pid)
{
    /* /sys/class/hidraw/hidrawN/device/uevent contains:
     * HID_ID=BBBB:VVVVVVVV:PPPPPPPP  (all hex, no 0x prefix) */
    const char *name = strrchr(devpath, '/');
    if (!name) return 0;
    char uevent[256];
    snprintf(uevent, sizeof uevent, "/sys/class/hidraw%s/device/uevent", name);
    FILE *f = fopen(uevent, "r");
    if (!f) return 0;
    char line[128];
    int found = 0;
    while (fgets(line, sizeof line, f)) {
        unsigned bus, v, p;
        if (sscanf(line, "HID_ID=%x:%x:%x", &bus, &v, &p) == 3) {
            found = (v == (unsigned)vid && p == (unsigned)pid);
            break;
        }
    }
    fclose(f);
    return found;
}

rayneo_dev *rayneo_open(void)
{
    DIR *d = opendir("/dev");
    if (!d) return NULL;
    struct dirent *e;
    char path[256];
    rayneo_dev *dev = NULL;
    while ((e = readdir(d))) {
        if (strncmp(e->d_name, "hidraw", 6)) continue;
        snprintf(path, sizeof path, "/dev/%s", e->d_name);
        if (!check_hidraw(path, RAYNEO_VID, RAYNEO_PID)) continue;
        dev = rayneo_open_path(path);
        if (dev) break;
    }
    closedir(d);
    if (!dev) errno = ENODEV;
    return dev;
}

rayneo_dev *rayneo_open_path(const char *path)
{
    int fd = open(path, O_RDWR | O_NONBLOCK);
    if (fd < 0) return NULL;
    rayneo_dev *d = calloc(1, sizeof *d);
    if (!d) { close(fd); return NULL; }
    d->fd = fd;
    snprintf(d->path, sizeof d->path, "%s", path);
    return d;
}

const char *rayneo_devpath(rayneo_dev *d) { return d->path; }

void rayneo_close(rayneo_dev *d)
{
    if (!d) return;
    close(d->fd);
    free(d);
}

/* ---- IMU control ---- */

int rayneo_enable_imu(rayneo_dev *d)
{
    uint8_t cmd[64] = {0x66, 0x01, 0x00};
    return write(d->fd, cmd, 64) < 0 ? -1 : 0;
}

int rayneo_disable_imu(rayneo_dev *d)
{
    uint8_t cmd[64] = {0x66, 0x02, 0x00};
    return write(d->fd, cmd, 64) < 0 ? -1 : 0;
}

/* ---- IMU read ---- */

/*
 * RayNeo AR Glasses IMU HID report (64 bytes):
 *   [0]    0x99  frame type
 *   [1]    0x65  IMU subtype
 *   [4..15]  accel x,y,z  float32 (m/s²)
 *   [16..27] gyro  x,y,z  float32 (deg/s)
 *   [28..39] mag   x,y,z  float32 (µT or arbitrary units)
 *   [40..43] tick  uint32 (0.1 ms units, 10 kHz)
 */
int rayneo_read_imu(rayneo_dev *d, rayneo_imu *out, int timeout_ms)
{
    struct pollfd pfd = { .fd = d->fd, .events = POLLIN };
    for (;;) {
        int r = poll(&pfd, 1, timeout_ms);
        if (r < 0) return r;
        if (r == 0) return 0;   /* timeout */

        uint8_t buf[64];
        int n = read(d->fd, buf, 64);
        if (n < 0) return n;
        if (n < 44) continue;
        if (buf[0] != 0x99 || buf[1] != 0x65) continue;

        /* Axis remap. The RayNeo IMU's raw frame is x=head left-right, y=up,
         * z=forward (determined empirically: gravity rests on +Y; a head-shake
         * "no" drives gyro Y, a nod "yes" drives gyro X). The Madgwick AHRS and
         * the ZYX euler below assume z=up, y=left-right, x=forward, so rotate
         * every sensor by the same proper rotation (x,y,z) <- (z,x,y). */
        float ax, ay, az, gx, gy, gz, mx, my, mz;
        memcpy(&ax, buf + 4,  4); memcpy(&ay, buf + 8,  4); memcpy(&az, buf + 12, 4);
        memcpy(&gx, buf + 16, 4); memcpy(&gy, buf + 20, 4); memcpy(&gz, buf + 24, 4);
        memcpy(&mx, buf + 28, 4); memcpy(&my, buf + 32, 4); memcpy(&mz, buf + 36, 4);
        const float d2r = (float)(M_PI / 180.0);
        out->accel[0]    = az;        out->accel[1]    = ax;        out->accel[2]    = ay;
        out->gyro_rad[0] = gz * d2r;  out->gyro_rad[1] = gx * d2r;  out->gyro_rad[2] = gy * d2r;
        out->mag[0]      = mz;        out->mag[1]      = mx;        out->mag[2]      = my;

        memcpy(&out->tick, buf + 40, 4);
        return 1;
    }
}

/* ---- Madgwick AHRS ---- */

void rayneo_ahrs_init(rayneo_ahrs *a, float beta)
{
    a->q[0] = 1.0f; a->q[1] = 0.0f; a->q[2] = 0.0f; a->q[3] = 0.0f;
    a->beta = beta;
}

/* 6-axis (gyro + accel only) */
void rayneo_ahrs_update(rayneo_ahrs *a, const rayneo_imu *s, float dt)
{
    float q0 = a->q[0], q1 = a->q[1], q2 = a->q[2], q3 = a->q[3];
    float gx = s->gyro_rad[0], gy = s->gyro_rad[1], gz = s->gyro_rad[2];
    float ax = s->accel[0],    ay = s->accel[1],    az = s->accel[2];

    /* normalise accel */
    float norm = sqrtf(ax*ax + ay*ay + az*az);
    if (norm < 1e-6f) goto integrate;
    norm = 1.0f / norm;
    ax *= norm; ay *= norm; az *= norm;

    /* gradient descent on f_g */
    float _2q0 = 2.0f*q0, _2q1 = 2.0f*q1, _2q2 = 2.0f*q2, _2q3 = 2.0f*q3;
    float _4q0 = 4.0f*q0, _4q1 = 4.0f*q1, _4q2 = 4.0f*q2;
    float _8q1 = 8.0f*q1, _8q2 = 8.0f*q2;
    float q0q0 = q0*q0, q1q1 = q1*q1, q2q2 = q2*q2, q3q3 = q3*q3;

    float s0 = _4q0*q2q2 + _2q2*ax + _4q0*q1q1 - _2q1*ay;
    float s1 = _4q1*q3q3 - _2q3*ax + 4.0f*q0q0*q1 - _2q0*ay - _4q1 + _8q1*q1q1 + _8q1*q2q2 + _4q1*az;
    float s2 = 4.0f*q0q0*q2 + _2q0*ax + _4q2*q3q3 - _2q3*ay - _4q2 + _8q2*q1q1 + _8q2*q2q2 + _4q2*az;
    float s3 = 4.0f*q1q1*q3 - _2q1*ax + 4.0f*q2q2*q3 - _2q2*ay;

    norm = 1.0f / sqrtf(s0*s0 + s1*s1 + s2*s2 + s3*s3);
    s0 *= norm; s1 *= norm; s2 *= norm; s3 *= norm;

    gx -= a->beta * s1;
    gy -= a->beta * s2;
    gz -= a->beta * s3;
    (void)s0;

integrate:;
    float qDot0 = 0.5f*(-q1*gx - q2*gy - q3*gz);
    float qDot1 = 0.5f*( q0*gx + q2*gz - q3*gy);
    float qDot2 = 0.5f*( q0*gy - q1*gz + q3*gx);
    float qDot3 = 0.5f*( q0*gz + q1*gy - q2*gx);

    q0 += qDot0 * dt; q1 += qDot1 * dt;
    q2 += qDot2 * dt; q3 += qDot3 * dt;

    norm = 1.0f / sqrtf(q0*q0 + q1*q1 + q2*q2 + q3*q3);
    a->q[0] = q0*norm; a->q[1] = q1*norm;
    a->q[2] = q2*norm; a->q[3] = q3*norm;
}

/* 9-axis (gyro + accel + mag) */
void rayneo_ahrs_update9(rayneo_ahrs *a, const rayneo_imu *s,
                         const float mag[3], float dt)
{
    float q0 = a->q[0], q1 = a->q[1], q2 = a->q[2], q3 = a->q[3];
    float gx = s->gyro_rad[0], gy = s->gyro_rad[1], gz = s->gyro_rad[2];
    float ax = s->accel[0],    ay = s->accel[1],    az = s->accel[2];
    float mx = mag[0],         my = mag[1],         mz = mag[2];

    float norm;

    norm = sqrtf(ax*ax + ay*ay + az*az);
    if (norm < 1e-6f) { rayneo_ahrs_update(a, s, dt); return; }
    norm = 1.0f/norm; ax*=norm; ay*=norm; az*=norm;

    norm = sqrtf(mx*mx + my*my + mz*mz);
    if (norm < 1e-6f) { rayneo_ahrs_update(a, s, dt); return; }
    norm = 1.0f/norm; mx*=norm; my*=norm; mz*=norm;

    float _2q0mx = 2.0f*q0*mx, _2q0my = 2.0f*q0*my, _2q0mz = 2.0f*q0*mz;
    float _2q1mx = 2.0f*q1*mx;
    float _2q0 = 2.0f*q0, _2q1 = 2.0f*q1, _2q2 = 2.0f*q2, _2q3 = 2.0f*q3;
    float _2q0q2 = 2.0f*q0*q2, _2q2q3 = 2.0f*q2*q3;
    float q0q0 = q0*q0, q0q1 = q0*q1, q0q2 = q0*q2, q0q3 = q0*q3;
    float q1q1 = q1*q1, q1q2 = q1*q2, q1q3 = q1*q3;
    float q2q2 = q2*q2, q2q3 = q2*q3, q3q3 = q3*q3;

    /* reference direction of earth's magnetic field */
    float hx = mx*(q0q0+q1q1-q2q2-q3q3) + 2.0f*my*(q1q2-q0q3) + 2.0f*mz*(q1q3+q0q2);
    float hy = 2.0f*mx*(q1q2+q0q3) + my*(q0q0-q1q1+q2q2-q3q3) + 2.0f*mz*(q2q3-q0q1);
    float _2bx = sqrtf(hx*hx + hy*hy), _2bz = 2.0f*mx*(q1q3-q0q2)
                + 2.0f*my*(q2q3+q0q1) + mz*(q0q0-q1q1-q2q2+q3q3);
    float _4bx = 2.0f*_2bx, _4bz = 2.0f*_2bz;

    float s0 = -_2q2*(2.0f*q1q3 - _2q0q2 - ax) + _2q1*(2.0f*q0q1 + _2q2q3 - ay)
              - _2bz*q2*(_2bx*(0.5f - q2q2 - q3q3) + _2bz*(q1q3 - q0q2) - mx)
              + (-_2bx*q3 + _2bz*q1)*(_2bx*(q1q2 - q0q3) + _2bz*(q0q1 + q2q3) - my)
              + _2bx*q2*(_2bx*(q0q2 + q1q3) + _2bz*(0.5f - q1q1 - q2q2) - mz);
    float s1 =  _2q3*(2.0f*q1q3 - _2q0q2 - ax) + _2q0*(2.0f*q0q1 + _2q2q3 - ay)
              - 4.0f*q1*(1.0f - 2.0f*q1q1 - 2.0f*q2q2 - az)
              + _2bz*q3*(_2bx*(0.5f - q2q2 - q3q3) + _2bz*(q1q3 - q0q2) - mx)
              + (_2bx*q2 + _2bz*q0)*(_2bx*(q1q2 - q0q3) + _2bz*(q0q1 + q2q3) - my)
              + (_2bx*q3 - _4bz*q1)*(_2bx*(q0q2 + q1q3) + _2bz*(0.5f - q1q1 - q2q2) - mz);
    float s2 = -_2q0*(2.0f*q1q3 - _2q0q2 - ax) + _2q3*(2.0f*q0q1 + _2q2q3 - ay)
              - 4.0f*q2*(1.0f - 2.0f*q1q1 - 2.0f*q2q2 - az)
              + (-_4bx*q2 - _2bz*q0)*(_2bx*(0.5f - q2q2 - q3q3) + _2bz*(q1q3 - q0q2) - mx)
              + (_2bx*q1 + _2bz*q3)*(_2bx*(q1q2 - q0q3) + _2bz*(q0q1 + q2q3) - my)
              + (_2bx*q0 - _4bz*q2)*(_2bx*(q0q2 + q1q3) + _2bz*(0.5f - q1q1 - q2q2) - mz);
    float s3 =  _2q1*(2.0f*q1q3 - _2q0q2 - ax) + _2q2*(2.0f*q0q1 + _2q2q3 - ay)
              + (-_4bx*q3 + _2bz*q1)*(_2bx*(0.5f - q2q2 - q3q3) + _2bz*(q1q3 - q0q2) - mx)
              + (-_2bx*q0 + _2bz*q2)*(_2bx*(q1q2 - q0q3) + _2bz*(q0q1 + q2q3) - my)
              + _2bx*q1*(_2bx*(q0q2 + q1q3) + _2bz*(0.5f - q1q1 - q2q2) - mz);

    norm = 1.0f / sqrtf(s0*s0 + s1*s1 + s2*s2 + s3*s3);
    s0*=norm; s1*=norm; s2*=norm; s3*=norm;

    gx -= a->beta*s1; gy -= a->beta*s2; gz -= a->beta*s3;
    (void)s0;

    float qDot0 = 0.5f*(-q1*gx - q2*gy - q3*gz);
    float qDot1 = 0.5f*( q0*gx + q2*gz - q3*gy);
    float qDot2 = 0.5f*( q0*gy - q1*gz + q3*gx);
    float qDot3 = 0.5f*( q0*gz + q1*gy - q2*gx);

    q0 += qDot0*dt; q1 += qDot1*dt; q2 += qDot2*dt; q3 += qDot3*dt;
    norm = 1.0f / sqrtf(q0*q0 + q1*q1 + q2*q2 + q3*q3);
    a->q[0]=q0*norm; a->q[1]=q1*norm; a->q[2]=q2*norm; a->q[3]=q3*norm;

    /* suppress unused warnings */
    (void)_2q0mx; (void)_2q0my; (void)_2q0mz; (void)_2q1mx;
    (void)_2q0; (void)_2q1; (void)_2q2; (void)_2q3;
    (void)q0q1; (void)q0q2; (void)q0q3; (void)q1q2; (void)q1q3; (void)q2q3;
    (void)_4bx; (void)_4bz;
}

/* ZYX Euler angles in degrees: rotate NED quaternion to yaw/pitch/roll */
void rayneo_ahrs_euler(const rayneo_ahrs *a,
                       float *yaw, float *pitch, float *roll)
{
    float q0=a->q[0], q1=a->q[1], q2=a->q[2], q3=a->q[3];
    float r2d = (float)(180.0 / M_PI);
    *yaw   =  atan2f(2.0f*(q0*q3 + q1*q2), 1.0f - 2.0f*(q2*q2 + q3*q3)) * r2d;
    *pitch =  asinf( 2.0f*(q0*q2 - q3*q1)) * r2d;
    *roll  =  atan2f(2.0f*(q0*q1 + q2*q3), 1.0f - 2.0f*(q1*q1 + q2*q2)) * r2d;
}
