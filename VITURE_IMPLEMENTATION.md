# Viture Glasses Implementation Guide

Reverse-engineered from SpaceWalker 1.8.4 on macOS. This documents how to implement Viture XR glasses (N6, N6P, P6, R6) support in mirage.

---

## Overview

Viture glasses use a 3-tier tracking system:
1. **IMU** (240 Hz) - Raw gyro/accel over USB HID
2. **VIO client** - Complementary/Kalman filter fusion
3. **SLAM** - Full Visual-Inertial Odometry with anchor support

For mirage, we only need tier 1 (IMU) + a Madgwick AHRS, identical to the RayNeo approach.

---

## USB/HID Interface

### Device Identification

```c
// Viture glasses USB identifiers
#define VITURE_VID  0x2D7F    // Viture vendor ID

// Known PIDs (check for multiple - different models)
// The exact PID varies by model (N6, N6P, P6, R6)
// Discover via: system_profiler SPUSBDataType | grep -i viture
```

### HID Report Structure

SpaceWalker receives IMU data via HID reports. Based on the source paths in the binary:
- `VTIMUManager.swift` - Main IMU handling
- `VTGetIMUReportFQHIDMsg.swift` - IMU frequency/data messages

**IMU Report Format** (derived from SpaceWalker strings):
```c
struct viture_imu_report {
    uint8_t  report_id;       // HID report identifier
    uint8_t  subtype;         // Message subtype
    // ... padding
    float    accel[3];        // Accelerometer (m/s^2)
    float    gyro[3];         // Gyroscope (deg/s or rad/s)
    float    mag[3];          // Magnetometer (optional)
    uint32_t timestamp;       // Device timestamp
};
```

**Enable IMU streaming:**
SpaceWalker sends HID commands to start/stop IMU. Pattern likely similar to RayNeo:
```c
// Start IMU (hypothetical - needs verification via USB capture)
uint8_t cmd_start[64] = {0x66, 0x01, ...};
write(fd, cmd_start, 64);

// Stop IMU
uint8_t cmd_stop[64] = {0x66, 0x02, ...};
```

---

## IMU Configuration (from slam_config_N6_N6P.yaml)

```yaml
device:
  imu_rate: 240                          # Hz
  gravity_norm: 9.80665                  # m/s^2
  position_pivot_head: [0.0, 0.13, 0.09] # Offset from IMU to eyes (meters)

orientation_tracker:
  complementary_filter_options:
    online_calibrate_gyr_bias: true
    apply_still_gyr_correction: true
    push_forward_time: 12.0              # ms prediction
    atw_correction: 7.2                  # Async Time Warp correction (ms)

  kalman_filter_options:
    online_calibrate_gyr_bias: true
    apply_still_yaw_correction: true
    estimate_gyr_bias: true
    atw_correction: 27.2
```

**Key takeaways:**
- 240 Hz IMU rate (vs RayNeo's ~500 Hz)
- ATW correction of 7-27 ms for motion-to-photon latency
- Automatic gyro bias calibration when still

---

## Axis Mapping

From SpaceWalker's coordinate system (viture_override_config):
- Display-to-IMU transforms: `T_left_imu`, `T_right_imu`
- The transformation includes a 10-degree tilt (0.173648 ≈ sin(10°))

**Expected axis convention:**
```c
// Raw Viture IMU frame (hypothesis based on SpaceWalker)
// X = left/right, Y = up, Z = forward (similar to RayNeo)

// Remap to mirage (Z-up graphics convention):
// (mirage_x, mirage_y, mirage_z) = (viture_z, viture_x, viture_y)
```

Verify empirically:
1. Gravity at rest should be +9.8 on the "up" axis
2. Nodding "yes" = pitch (rotation about the left-right axis)
3. Shaking "no" = yaw (rotation about the up axis)

---

## Anchoring / Recenter Mechanism

SpaceWalker's `VitureSlamRecenterOrientation` captures the current orientation and subtracts it from all future readings.

**Algorithm (from reverse-engineering `ImuHandler::Recenter`):**

```c
void recenter(quat *reference, quat current) {
    // Extract current yaw from quaternion
    float yaw = atan2(
        2.0f * (current.x * current.y + current.z * current.w),
        1.0f - 2.0f * (current.y * current.y + current.z * current.z)
    );

    // Create anchor quaternion (yaw-only rotation)
    float half_yaw = yaw * 0.5f;
    reference->w = cosf(half_yaw);
    reference->x = 0.0f;
    reference->y = 0.0f;
    reference->z = sinf(half_yaw);
}

// Apply anchor to get relative orientation
quat get_relative_orientation(quat raw, quat reference) {
    return quat_multiply(quat_conjugate(reference), raw);
}
```

**mirage already does this correctly** in `pose.cpp`:
```cpp
static quat recenter_ref(quat q) {
    return q_norm(q);  // Full quaternion reference (no gimbal lock)
}

quat pose_latest(void) {
    return q_norm(q_mul(q_conj(P.reference), P.smoothed));
}
```

---

## SLAM Library APIs

SpaceWalker bundles these native libraries:

### libSlam.dylib (public API)
```c
void* VitureSlamCreateHandle(void);
void  VitureSlamDestroyHandle(void* handle);
int   VitureSlamStart(void* handle);
int   VitureSlamStop(void* handle);
int   VitureSlamGetPoseState(void* handle, PoseState* out);
int   VitureSlamGetPredictPoseState(void* handle, PoseState* out, uint64_t timestamp);
int   VitureSlamRecenterOrientation(void* handle);
int   VitureSlamUpdateIMU(void* handle, ImuData* data);
int   VitureSlamLockRoll(void* handle, bool lock);
int   VitureSlamSetZeroDoF(void* handle, bool enabled);
void  VitureSlamRegisterCallback(void* handle, SlamCallback cb);
void  VitureSlamSetParameters(void* handle, ...);
```

### libcarina_vio.dylib (internal VIO)
```c
int carina_vio_feed_imu(void* ctx, ...);
int carina_vio_get_anchor(void* ctx, double* position, double* quaternion);
int carina_vio_get_imu_pose(void* ctx, ...);
int carina_vio_get_gl_pose(void* ctx, ...);      // OpenGL-ready pose
int carina_vio_get_eyes_pose(void* ctx, ...);    // Per-eye transforms
int carina_vio_set_pose_callback(void* ctx, ...);
```

### Pose Data Structure

```c
// PoseState output (estimated from disassembly)
struct PoseState {
    double position[3];      // x, y, z in meters
    double velocity[3];      // vx, vy, vz in m/s
    double quaternion[4];    // w, x, y, z orientation
    uint64_t timestamp;      // microseconds
};
```

---

## Implementation Plan for mirage

### Option A: Simple (IMU-only, like RayNeo)

Create `src/viture.c` and `src/viture.h` mirroring the RayNeo pattern:

```c
// viture.h
#define VITURE_VID 0x2D7F

typedef struct viture_dev viture_dev;

typedef struct {
    float accel[3];      // m/s^2, remapped to mirage frame
    float gyro_rad[3];   // rad/s, remapped
    float mag[3];        // magnetometer (optional)
    uint32_t tick;       // device timestamp
} viture_imu;

typedef struct {
    float q[4];          // w, x, y, z
    float beta;          // Madgwick gain
} viture_ahrs;

viture_dev* viture_open(void);
viture_dev* viture_open_path(const char *path);
void        viture_close(viture_dev *d);
int         viture_enable_imu(viture_dev *d);
int         viture_disable_imu(viture_dev *d);
int         viture_read_imu(viture_dev *d, viture_imu *out, int timeout_ms);

void viture_ahrs_init(viture_ahrs *a, float beta);
void viture_ahrs_update(viture_ahrs *a, const viture_imu *s, float dt);
void viture_ahrs_euler(const viture_ahrs *a, float *yaw, float *pitch, float *roll);
```

### Option B: Use SpaceWalker's VIO

Dynamically load libSlam.dylib and call the high-level API:
```c
void *handle = dlopen("/Applications/SpaceWalker.app/.../libSlam.dylib", RTLD_LAZY);
auto create = dlsym(handle, "VitureSlamCreateHandle");
auto start = dlsym(handle, "VitureSlamStart");
auto getPose = dlsym(handle, "VitureSlamGetPoseState");
// etc.
```

**Pros:** Drift-free SLAM, anchor persistence, roll lock
**Cons:** macOS only, depends on SpaceWalker installation, reverse-engineered ABI

### Option C: Hybrid

Use SpaceWalker's VIO when available, fall back to Madgwick AHRS on Linux.

---

## Bridge Script (scripts/viture_bridge.sh)

```bash
#!/usr/bin/env bash
# viture_bridge.sh - Stream Viture IMU to mirage via OpenTrack UDP

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

# Build viture-bridge if needed
if [ ! -x "$HERE/viture-bridge" ]; then
    echo "[viture] building viture-bridge..."
    make viture-bridge
fi

# Find the hidraw device
HIDRAW=$(ls /dev/hidraw* 2>/dev/null | while read dev; do
    if grep -q "2D7F" "/sys/class/hidraw/$(basename $dev)/device/uevent" 2>/dev/null; then
        echo "$dev"
        break
    fi
done)

if [ -z "$HIDRAW" ]; then
    echo "[viture] no Viture glasses found on hidraw"
    exit 1
fi

echo "[viture] using $HIDRAW"

# May need sudo for hidraw access until udev rule is set
if [ ! -r "$HIDRAW" ]; then
    echo "[viture] need root for hidraw (add udev rule to fix)"
    exec sudo "$HERE/viture-bridge" "$HIDRAW"
else
    exec "$HERE/viture-bridge" "$HIDRAW"
fi
```

---

## USB Sniffing to Verify

To capture actual HID traffic and verify the protocol:

### macOS
```bash
# Use Wireshark with USBPcap or the built-in USB tracing
sudo log stream --predicate 'subsystem == "com.apple.iokit.IOUSBFamily"' --info
```

### Linux
```bash
# modprobe usbmon first
sudo cat /sys/kernel/debug/usb/usbmon/0u | grep -A20 "2d7f"
```

### Cross-platform
Run SpaceWalker with USB logging:
```bash
# Check ~/Library/Application Support/com.viture.spacewalker/spacewalker.log
# for IMU frequency and data patterns
tail -f spacewalker.log | grep -i imu
```

---

## Display Configuration (from viture_override_config_N6.yaml)

```yaml
display:
  resolution: [1920, 1080]
  frustum_left: [-0.0158967, 0.0158967, -0.008942, 0.008942, 0.050800, 100.0]
  frustum_right: [-0.0158967, 0.0158967, -0.008942, 0.008942, 0.050800, 100.0]

  # Transform from IMU to left/right eye (includes 10-degree downtilt)
  T_left_imu:
    [1.0,         0.0,          0.0,  0.0315,    # 31.5mm IPD/2
     0.0, 0.984807753, -0.173648178,     0.0,    # cos(10°), -sin(10°)
     0.0, 0.173648178,  0.984807753,     0.0,
     0.0,         0.0,          0.0,     1.0]
```

**For mirage:**
- IPD: ~63mm (0.0315 * 2)
- Panel downtilt: 10 degrees (baked into T_*_imu)
- Near plane: 50.8mm, Far: 100m

---

## Checklist

- [ ] Identify exact USB VID:PID for target Viture model
- [ ] Capture HID report structure via USB sniffing
- [ ] Implement `viture_open()` - scan hidraw for VID:PID
- [ ] Implement `viture_enable_imu()` - send start command
- [ ] Implement `viture_read_imu()` - parse HID reports
- [ ] Determine axis mapping empirically (gravity test)
- [ ] Integrate with mirage's `pose.cpp` OpenTrack backend
- [ ] Test recenter (double-tap Cmd)
- [ ] Tune Madgwick beta for Viture's IMU noise characteristics
- [ ] Add udev rule for non-root hidraw access
- [ ] Update `scripts/bridge.sh` to auto-detect Viture vs RayNeo

---

## References

- SpaceWalker 1.8.4: `/Applications/SpaceWalker.app`
- SLAM config: `SpaceWalker.app/Contents/Resources/slam_config_N6_N6P.yaml`
- Display config: `SpaceWalker.app/Contents/Resources/viture_override_config_N6.yaml`
- Logs: `~/Library/Application Support/com.viture.spacewalker/spacewalker.log`
- Madgwick AHRS: https://x-io.co.uk/open-source-imu-and-ahrs-algorithms/
