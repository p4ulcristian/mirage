# mirage — head-tracked virtual displays for Hyprland + AR glasses

Turns a headless virtual output into one 32:9 ultrawide screen, curved into a 3D
wall floating in front of you, captured live and rendered onto your AR glasses
with the camera driven by your head pose. Built for Hyprland on Wayland; verified
on RayNeo glasses (DP-1) with an Apple M2 GPU (Asahi).

It runs as an ordinary Wayland app: a fullscreen, opaque window on the glasses
output that Hyprland page-flips straight to the panel (direct scanout) at the
panel's native 120 Hz — no compositor, lease, or kernel changes.

There are no command-line flags or environment knobs. The one configuration lives
in `src/config.c` (curved wall, 1:1 world-fixed tracking, sharpening);
edit it and rebuild to change the scene.

## Status

| Piece | State |
|-------|-------|
| 32:9 virtual display (Hyprland headless) | ✅ working |
| Live capture → GL textures (zero-copy dmabuf, ext-image-copy-capture) | ✅ working |
| Curved wall + perspective on the glasses, direct scanout @120 Hz | ✅ working |
| Head-pose camera (RayNeo IMU → OpenTrack UDP) | ✅ working |
| Trackpad cursor confined to the wall + Cmd-scroll zoom | ✅ working |
| Quit / recenter hotkeys | ✅ working |

## Quick start

```bash
make && make bridge            # build mirage, the pose tool, and the RayNeo bridge
bash scripts/start-mirage.sh   # register hotkeys, then run on the glasses (blocks until quit)
```

`start-mirage.sh` is the launcher (also safe from a `.desktop` entry). It registers
the Hyprland keybinds and hands off to `scripts/glasses.sh`, which enables direct
scanout, brings up the virtual display + RayNeo bridge + window sweep, runs mirage
fullscreen, and restores the desktop when you quit.

The glasses must be present as the extended `DP-1` SmartGlasses output (not mirrored).

## Head pose

The **RayNeo bridge** (`scripts/bridge.sh` → `rayneo-bridge`) reads the glasses' IMU
over hidraw, runs a Madgwick AHRS, and streams head orientation to mirage as
OpenTrack UDP on `127.0.0.1:4242` (6 little-endian doubles `{x, y, z, yaw, pitch,
roll}`, angles in degrees). mirage auto-recenters on the first sample, so "straight
ahead" = wherever you're looking at launch (or press Alt+C any time).

It runs **6-axis** (gyro + accel): the bridge's gyro-bias auto-zero makes it
essentially drift-free at a desk, where the magnetometer's heading is distorted by
the laptop's local field. hidraw is root-only until a udev rule applies on a glasses
replug, so the bridge uses `sudo` until then.

Verify the pose stream without the renderer:

```bash
./mirage-posedump              # prints live quat + yaw/pitch/roll
```

## Controls (Hyprland keybinds)

| Key | Action |
|-----|--------|
| `SUPER+SHIFT+Q` | quit mirage, restore windows, remove the virtual display |
| `ALT+C` | recenter head pose (look straight ahead, then press) |

These are compositor-level binds, so they fire even while mirage covers the glasses.
Re-run `scripts/keybinds.sh` after a Hyprland reload. Trackpad capture is on from the
first frame; while captured, the trackpad drives a cursor across the wall and Cmd+scroll
zooms (telephoto). Double-tap Alt toggles the gaze-follow cursor, and a **3-finger swipe**
switches workspaces on the wall (creating a new one past the end) — mirage detects the
gesture itself, since its trackpad grab hides it from Hyprland.

## Architecture

```
RayNeo IMU ──(rayneo-bridge: AHRS)──▶ OpenTrack UDP :4242 ──▶ pose.c ──▶ head quaternion
Hyprland VIRT1 ──(ext-image-copy-capture + dmabuf)──▶ capture.c ──▶ GL textures
                                                            │
   layout.c (wall placement) ─┐                             ▼
   pose (camera) ─────────────┴──▶ render.c ──▶ fullscreen window on the glasses
```

- `src/main.c`     Wayland setup, the frame loop, fullscreen window on the glasses
- `src/pose.c`     head-pose input (OpenTrack UDP), One-Euro smoothing, recenter
- `src/capture.c`  ext-image-copy-capture → gbm/dmabuf → EGLImage → GLES2 texture
- `src/render.c`   EGL/GLES2: curved textured wall, perspective camera, sharpen
- `src/layout.c`   where the screen sits on the arc
- `src/grab.c`     trackpad capture, the arc cursor, gaze cursor, zoom
- `src/config.c`   the one hardcoded scene/tracking configuration
- `src/math3d.h`   vec/quat/mat4 (no glm dependency)
- `src/rayneo*.c`, `src/magcal.c`  the RayNeo IMU bridge (Madgwick AHRS, mag calibration)
- `protocol/`      vendored wlroots/hyprland protocol XML
- `scripts/`       start-mirage, glasses, bridge, setup/teardown-displays, sweep, keybinds, stop
```
