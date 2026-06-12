# Viture Glasses Implementation Guide

Reverse-engineered from SpaceWalker 1.8.4 on macOS and [mgschwan/viture_virtual_display](https://github.com/mgschwan/viture_virtual_display).

This documents how to implement Viture XR glasses (One, Pro, N6, N6P, P6, R6) support in mirage.

---

## Beast XR (R6) Critical Findings (2025-06-12)

After extensive reverse-engineering with SIP disabled, we discovered:

### 1. 6DOF VIO Runs ON THE GLASSES, Not Host

**The Beast XR glasses have an on-device VIO processor (A1088 chip).** The camera frames are NOT exposed to the host via USB. The VIO/SLAM computation happens inside the glasses.

Evidence:
- No UVC camera device exposed (checked `system_profiler SPCameraDataType`)
- USB interfaces are HID-only (no video bulk endpoints)
- libcarina_vio.dylib functions like `_carina_a1088_viture_init` exist but are NOT called in 3DOF mode
- The `get_cam_param` functions exist to retrieve intrinsics from device, but never called in 3DOF

### 2. 3DOF Mode = IMU Only

When you enable "3DOF" in SpaceWalker:
- `dofMode: dof3` is set via HID command
- **No VIO library functions are called**
- Host just receives pre-fused euler angles (roll/pitch/yaw) via HID
- The glasses do the IMU fusion internally

### 3. What This Means for Linux

**Good news:** 3DOF tracking is simple - just read IMU euler angles from HID.

**Challenge:** For 6DOF (positional) tracking, you'd need:
- Either: A way to request 6DOF pose data from the glasses (if supported)
- Or: External cameras/SLAM system on the host

### 4. USB Device Details

```
VITURE Beast XR Glasses
  VID: 0x35CA (13770)
  PID: 0x1201 (4609)
  USB Speed: 480 Mbps (High Speed)
  Device Class: 0xEF (Miscellaneous) with IAD
  Serial: R6PMCC61301YG
  Firmware: 20.0.01.025_20260608
```

### 4b. Beast HID Protocol (Reverse-Engineered 2025-06-12)

**IMPORTANT:** Beast uses a DIFFERENT protocol than older Viture glasses!

**Packet format:**
```
Header: 10 00 XX YY [data...]  (64 bytes total)
        ^^ ^^ ^^ ^^
        |  |  |  +-- Sub-command/parameter
        |  |  +----- Command type
        |  +-------- Always 0x00
        +----------- Report ID 0x10
```

**NOT `FF FC`/`FF FE` like older glasses!**

**Commands observed (via SpaceWalker trace):**
```
10 00 03 30 - Query (serial/version?)
10 00 02 30 - Query (status?)
10 00 01 32 - Get firmware version
10 00 22 31 - Get display mode
10 00 05 30 - Get brightness
10 00 28 31 - Unknown query
10 00 24 31 - Get display size/mode
10 00 00 34 - Unknown (init?)
10 00 01 34 08 00 1c 00 07 00 01 02 03 04 05 06 - Configuration
10 00 02 34 07 00 15 00 06 00 01 02 03 04 05    - Configuration
10 00 00 04 04 00 7e 01 9a 4f 2b 6a             - Unknown (timestamp?)
```

**IMU data:** Likely delivered via HID input reports (async), not SetReport/GetReport.
The input callback registration didn't fire in our trace - SpaceWalker may use
IOHIDQueueCreate or a different async mechanism.

### 5. SpaceWalker Display Modes

From HID messages:
- `mode2D60In120OutR6` - 2D mode, 60Hz in, 120Hz out
- `ultrawide_60Hz` - Wide strip layout
- `dofMode: dof0` = No tracking
- `dofMode: dof3` = 3DOF (rotation only)

### 6. Native Mode Fix (2025-06-12)

**BUG FOUND:** The `--native` mode in `viture_bridge.c` was only logging pose data, not sending it to mirage!

**FIX:** Updated `on_pose()` callback to actually forward the firmware-fused quaternion via UDP when `--native` is used.

**To test on Linux with factory-calibrated tracking:**
```bash
# This uses the Beast's on-device fusion - same quality as SpaceWalker
sudo -E VITURE_SDK=viture-sdk ./viture-bridge --native -v
```

**Why native mode is better:**
- Firmware has factory-calibrated IMU parameters
- No drift from imperfect host-side fusion
- Same tracking quality as SpaceWalker on macOS
- Skips all Madgwick/VQF processing

---

## TEST THIS ON LINUX (VIO Anchor Fix)

The VIO was crashing because it used `pinhole` camera model, but the Beast has a **fisheye** camera.
Fixed in commit `36eb644` - now uses `equidistant` (Kannala-Brandt fisheye).

### Step 1: Pull and check camera resolution

```bash
git pull
v4l2-ctl --list-formats-ext -d /dev/video1
```

### Step 2: If resolution differs from 640x480

Edit `src/viture_vio.cpp` around line 168:
```yaml
intrinsics: [285.0, 285.0, WIDTH/2, HEIGHT/2]
resolution: [WIDTH, HEIGHT]
```

Example for 1280x720:
```yaml
intrinsics: [350.0, 350.0, 640.0, 360.0]
resolution: [1280, 720]
```

### Step 3: Build and run

```bash
make viture-vio
sudo -E LD_LIBRARY_PATH=viture-sdk ./viture-vio --run -v
```

### Step 4: If it still fails

Tell me:
1. The exact error message
2. Output of `v4l2-ctl --list-formats-ext -d /dev/video1`

We'll adjust intrinsics or distortion coefficients.

---

## Why We Couldn't Extract Exact Intrinsics from Mac

**macOS SIP (System Integrity Protection)** blocks:
- Attaching lldb to signed apps (SpaceWalker)
- DYLD_INSERT_LIBRARIES injection
- dtrace on protected processes

### Option: Disable SIP Temporarily (if you want exact intrinsics)

**Step 1: Boot to Recovery Mode (M2 Mac)**
1. Shut down completely (Apple menu → Shut Down, wait 10 sec)
2. Press and HOLD power button until you see "Loading startup options"
3. Click **Options** → Click **Continue**
4. If asked, select your user and enter password
5. You're now in Recovery Mode

**Step 2: Disable SIP**
1. From menu bar: **Utilities → Terminal**
2. Type exactly:
```bash
csrutil disable
```
3. Press Enter. It should say "Successfully disabled System Integrity Protection"
4. Type:
```bash
reboot
```

**Step 3: Verify after reboot**
```bash
csrutil status
# Should say: "System Integrity Protection status: disabled"
```

**Step 3: Capture the config**
```bash
cd /Users/paul/mirage/tools
bash capture_carina_config.sh
# Config saved to /tmp/carina_config_captured.yaml
```

**Step 4: Re-enable SIP (important!)**
```bash
# Boot to Recovery Mode again
csrutil enable
reboot
```

**Is it worth it?** Probably not for now - try the estimated fisheye intrinsics first. If tracking is wobbly/drifty, then either:
1. Disable SIP and capture exact config, OR
2. Calibrate with a checkerboard (more accurate anyway)

---

## Quick Summary

**The easy path**: Use the existing reverse-engineered code from `viture_virtual_display`:
- Copy `viture_connection.c` and `viture_connection.h`
- IMU data arrives as **3 floats: roll, pitch, yaw** (in degrees)
- Yaw needs to be **negated**
- Send to mirage via OpenTrack UDP or the JSON socket

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
#define VITURE_VID  0x35CA    // Viture vendor ID (13770 decimal)

// Known PIDs:
// 0x1201 (4609) - Beast XR Glasses (Pro)
// Others vary by model - use VID match with PID=0

// Two HID interfaces:
#define IMU_INTERFACE  0      // For IMU data stream
#define MCU_INTERFACE  1      // For commands (enable IMU, brightness, etc.)
```

### HID Packet Structure

All packets use a common format with CRC-16-CCITT:

```c
// 64-byte HID packet structure
struct viture_packet {
    uint8_t  header[2];       // 0xFF 0xFE (MCU) or 0xFF 0xFC (IMU)
    uint16_t crc;             // CRC-16-CCITT over bytes 4+
    uint16_t payload_len;     // Length of payload (from this field)
    uint8_t  reserved[8];     // Usually zeros, may contain timestamp
    uint16_t cmd_id;          // Command/event ID
    uint8_t  reserved2[2];    // More reserved bytes
    uint8_t  data[46];        // Actual payload data (max 46 bytes)
};
```

### IMU Data Format (THE KEY PART)

IMU packets arrive with header `0xFF 0xFC`. After parsing, the data payload contains:

```c
// IMU data is 3 floats = 12 bytes (euler angles in degrees)
float roll  = *(float*)(data + 0);    // bytes 0-3
float pitch = *(float*)(data + 4);    // bytes 4-7
float yaw   = -*(float*)(data + 8);   // bytes 8-11, NEGATED!
```

**Note**: Yaw must be negated to match expected convention.

### Enable/Disable IMU

Send command `0x15` via the MCU interface:

```c
// Enable IMU streaming
uint32_t set_imu(bool enable) {
    // cmd_id = 0x15, data = 0x01 (on) or 0x00 (off)
    return native_mcu_exec(0x15, enable ? 1 : 0);
}
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
