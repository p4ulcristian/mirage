/* vqf_shim.h - C-callable wrapper around dlaidig/vqf (VQF, MIT, C++).
 * Lets the C viture bridge use the VQF AHRS without going C++ itself.
 * Quaternions are scalar-first [w,x,y,z]; gyro rad/s, accel m/s^2, mag arbitrary
 * (consistent) units. Construct with the fixed IMU sample period in seconds. */
#ifndef VQF_SHIM_H
#define VQF_SHIM_H
#ifdef __cplusplus
extern "C" {
#endif

typedef struct vqf_handle vqf_handle;

vqf_handle *vqf_create(double sample_dt_sec);
void vqf_destroy(vqf_handle *h);

/* 6-axis (gyro+accel) and 9-axis (gyro+accel+mag) updates. Call one per sample. */
void vqf_update6(vqf_handle *h, const double gyr[3], const double acc[3]);
void vqf_update9(vqf_handle *h, const double gyr[3], const double acc[3], const double mag[3]);

/* Read back the fused orientation (scalar-first [w,x,y,z]). 6D = gyro+accel only
 * (yaw unreferenced), 9D = with magnetometer heading. */
void vqf_quat6(vqf_handle *h, double out_wxyz[4]);
void vqf_quat9(vqf_handle *h, double out_wxyz[4]);

/* Diagnostics for tuning/verbose. */
void vqf_get_bias(vqf_handle *h, double out_rad_s[3]);  /* current gyro-bias estimate */
int  vqf_rest_detected(vqf_handle *h);                  /* 1 = "still" detector active */
int  vqf_mag_dist_detected(vqf_handle *h);              /* 1 = magnetometer being rejected */

#ifdef __cplusplus
}
#endif
#endif
