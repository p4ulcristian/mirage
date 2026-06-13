/* pose.h - head-pose input layer for mirage.
 *
 * Abstracts where head orientation comes from. A background thread reads the
 * selected backend and keeps the latest orientation available lock-protected.
 *
 * Contract with the IMU driver (you write the driver, mirage consumes pose):
 *   POSE_OPENTRACK_UDP : driver sends OpenTrack packets to 127.0.0.1:<port>.
 *                        Packet = 6 little-endian doubles {x,y,z,yaw,pitch,roll}
 *                        with yaw/pitch/roll in DEGREES. (translation ignored
 *                        for 3DoF.) This is what XRLinuxDriver --opentrack-app
 *                        already emits, so a RayNeo plugin into that driver
 *                        works with zero glue.
 *   POSE_JSON_SOCKET   : driver writes newline-delimited JSON to a unix dgram
 *                        socket: {"quat":[w,x,y,z]}  (any extra keys ignored).
 *   POSE_BREEZY_SHM    : reads /dev/shm/breezy_desktop_imu (quaternion). [TODO]
 */
#ifndef MIRAGE_POSE_H
#define MIRAGE_POSE_H

#include "math3d.h"
#include <stdbool.h>
#include <stdint.h>

typedef enum {
    POSE_OPENTRACK_UDP,
    POSE_JSON_SOCKET,
    POSE_BREEZY_SHM,
} pose_backend;

typedef struct {
    pose_backend backend;
    int   udp_port;        /* for POSE_OPENTRACK_UDP (default 4242)        */
    const char *socket_path; /* for POSE_JSON_SOCKET                       */
    float smoothing;       /* 0..1 nlerp factor per update; 1 = no smoothing */
    /* One-Euro adaptive filter (the default). Steady when still, low-lag when
     * moving: the nlerp cutoff rises with angular speed. Set use_oneeuro=false
     * to fall back to the fixed `smoothing` nlerp above (e.g. for comparison).
     *   oe_mincutoff [Hz]  cutoff at rest; lower = steadier/more lag (try ~0.5)
     *   oe_beta            speed coupling; higher = less lag in motion (try ~1)
     * oe_dcutoff is the derivative low-pass cutoff (1 Hz is fine, not exposed). */
    bool  use_oneeuro;
    float oe_mincutoff, oe_beta, oe_dcutoff;
    /* per-axis sign for OpenTrack euler input (+1 or -1); 0 treated as +1.
     * Lets you flip a device whose yaw/pitch/roll runs opposite to head motion. */
    float sign_yaw, sign_pitch, sign_roll;
    /* Heading-drift compensation time constant (s); 0 = off. A 6-axis IMU has no
     * absolute heading so it slowly creeps, and the comfort gain magnifies it.
     * When the head is still, the recenter reference is leaked toward the current
     * orientation with this tau, cancelling the creep while leaving real head
     * motion (well above the stillness gate) untouched. ~1 s is a good default. */
    float drift_comp_tau;
} pose_config;

/* Start the reader thread. Returns 0 on success, -1 on error. */
int  pose_start(const pose_config *cfg);

/* Stop the reader thread and release resources. */
void pose_stop(void);

/* Latest head orientation, already re-centered against the reference set by
 * pose_recenter(). Returns identity until first packet arrives. */
quat pose_latest(void);

/* Same, but extrapolated horizon_s seconds ahead along the current angular
 * velocity, to cancel motion-to-photon latency (the wall stays nailed to the world
 * while you turn). horizon_s <= 0 == pose_latest(). */
quat pose_predicted(float horizon_s);

/* Set the current raw orientation as the new "looking straight ahead" zero. */
void pose_recenter(void);

/* Monotonic milliseconds since the last fresh sample (UINT32_MAX if none).
 * NB: this is sender staleness only — it does NOT include the smoothing-filter
 * lag or the render/present pipeline. The inbound sample RATE (below) is the
 * real "is the head pose even 120 Hz fresh?" number. */
uint32_t pose_age_ms(void);

/* Number of raw samples received since the previous call, and resets the
 * counter. Divide by the elapsed wall time to get the inbound pose rate (Hz):
 * if the source only emits 60/s you render 120 fps but only half the frames
 * carry a new head sample. */
uint32_t pose_take_sample_count(void);

/* Runtime A/B toggle for the smoothing filter. With smoothing OFF the raw
 * orientation is passed straight through (jittery but zero added filter lag),
 * which is the quick test for "is text unreadable-while-turning a LATENCY
 * problem?" — if it reads better raw, the filter lag is the culprit, not fps.
 * pose_toggle_smoothing() returns the new enabled state. */
bool pose_toggle_smoothing(void);
bool pose_smoothing_enabled(void);

/* Has at least one sample been received? */
bool pose_has_signal(void);

#endif /* MIRAGE_POSE_H */
