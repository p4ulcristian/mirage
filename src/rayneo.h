#pragma once
#include <stdint.h>

#define RAYNEO_VID 0x1bbb
#define RAYNEO_PID 0xaf50

typedef struct rayneo_dev rayneo_dev;

typedef struct {
    float gyro_rad[3];
    float accel[3];
    float mag[3];
    uint32_t tick;
} rayneo_imu;

/* quaternion-based Madgwick AHRS */
typedef struct {
    float q[4];   /* w x y z */
    float beta;
} rayneo_ahrs;

rayneo_dev *rayneo_open(void);
rayneo_dev *rayneo_open_path(const char *path);
const char *rayneo_devpath(rayneo_dev *d);
int         rayneo_enable_imu(rayneo_dev *d);
int         rayneo_disable_imu(rayneo_dev *d);
int         rayneo_read_imu(rayneo_dev *d, rayneo_imu *out, int timeout_ms);
void        rayneo_close(rayneo_dev *d);

void rayneo_ahrs_init(rayneo_ahrs *a, float beta);
void rayneo_ahrs_set_from_accel(rayneo_ahrs *a, const float accel[3]);  /* seed from gravity */
void rayneo_ahrs_update(rayneo_ahrs *a, const rayneo_imu *s, float dt);
void rayneo_ahrs_update9(rayneo_ahrs *a, const rayneo_imu *s,
                         const float mag[3], float dt);
void rayneo_ahrs_euler(const rayneo_ahrs *a,
                       float *yaw, float *pitch, float *roll);
