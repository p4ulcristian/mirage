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
    /* per-axis sign for OpenTrack euler input (+1 or -1); 0 treated as +1.
     * Lets you flip a device whose yaw/pitch/roll runs opposite to head motion. */
    float sign_yaw, sign_pitch, sign_roll;
} pose_config;

/* Start the reader thread. Returns 0 on success, -1 on error. */
int  pose_start(const pose_config *cfg);

/* Stop the reader thread and release resources. */
void pose_stop(void);

/* Latest head orientation, already re-centered against the reference set by
 * pose_recenter(). Returns identity until first packet arrives. */
quat pose_latest(void);

/* Set the current raw orientation as the new "looking straight ahead" zero. */
void pose_recenter(void);

/* Monotonic milliseconds since the last fresh sample (UINT32_MAX if none). */
uint32_t pose_age_ms(void);

/* Has at least one sample been received? */
bool pose_has_signal(void);

#endif /* MIRAGE_POSE_H */
