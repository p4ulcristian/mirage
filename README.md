# mirage

**A head-tracked ultrawide monitor for your AR glasses.**

mirage takes a headless 32:9 virtual display, curves it into a wall floating two
metres in front of you, and renders it onto AR glasses with the camera locked to
your head pose — so the wall stays nailed in space like a real monitor while you
look around it. Your existing windows are swept onto it automatically, and the
trackpad drives a cursor across it.

Built for **Hyprland** on Wayland; developed on **RayNeo Air** glasses driven by an
**Apple M2 GPU (Asahi)**.

```
            ┌───────────────────────────────────────────────┐
            │   your terminals · browser · editor, on a      │   ← 5120×1440 curved wall,
            │   curved 32:9 wall that holds still in space    │     world-fixed, 120 Hz
            └───────────────────────────────────────────────┘
                         ▲ head pose (RayNeo IMU)
```

It runs as an ordinary Wayland client: one fullscreen, opaque window on the glasses
output, which Hyprland page-flips straight to the panel (**direct scanout**) at the
native **120 Hz** — no compositor fork, no DRM leasing, no kernel patches.

---

## Highlights

- **World-fixed ultrawide** — one 5120×1440 curved wall, 1:1 head tracking, so it
  behaves like a physical monitor hung in the air.
- **Zero-copy capture** — `ext-image-copy-capture` + dmabuf/EGLImage straight into
  GLES2 textures; idle windows cost ~nothing (damage-driven).
- **Smooth & readable** — 120 Hz direct scanout, One-Euro pose filter, a reading
  deadband that freezes sub-degree tremor, and contrast-adaptive text sharpening.
- **Real input on the wall** — the trackpad becomes a cursor confined to the wall,
  with telephoto zoom and a gaze-follow mode.
- **Workspaces** — your windows sweep onto the wall on launch, laid out across
  the virtual displays exactly as you'd arrange them on the desktop.
- **One config, no knobs** — no flags, no env vars. Everything lives in
  `src/config.c`; edit and rebuild.

## Quick start

```bash
make && make bridge            # build mirage, mirage-posedump, and the RayNeo bridge
bash scripts/start-mirage.sh   # run on the glasses (blocks until you quit)
```

`start-mirage.sh` is the launcher (also safe to wire to a `.desktop` entry). It
registers the keybinds and hands off to `scripts/glasses.sh`, which:

1. enables direct scanout and pins an opaque/fullscreen rule for mirage,
2. brings up the `VIRT1` virtual display and the RayNeo head-tracking bridge,
3. sweeps your open windows onto the wall,
4. runs mirage fullscreen on the glasses, then restores the desktop on quit.

The glasses must be present as the extended `DP-1` SmartGlasses output (not mirrored).

## Controls

**Hotkeys** (Hyprland binds — they fire even while mirage covers the glasses):

| Key | Action |
|-----|--------|
| `Super+Shift+Q` | quit mirage, restore windows, remove the virtual display |

**Trackpad** (capture is on from the first frame):

| Gesture | Action |
|---------|--------|
| move / click / two-finger scroll | cursor / click / scroll the focused window |
| **Cmd + scroll** | telephoto zoom (narrows the field of view) |
| **double-tap Cmd** | recenter head pose — look straight ahead, then double-tap |
| **double-tap Alt** | toggle the gaze-follow cursor |

> **Note on `keyd`:** recenter is **double-tap Cmd**, read straight off libinput by
> mirage's trackpad grab, so it never depends on Hyprland seeing the keystroke. mirage
> auto-detects the trackpad and *every* Meta-capable keyboard by evdev capability, so it
> works whether keyd is grabbing the raw keyboard or passing it through.

## How it works

```
RayNeo IMU ──(rayneo-bridge: Madgwick AHRS)──▶ OpenTrack UDP :4242 ──▶ pose.c ──┐
                                                                               ▼ head quaternion
Hyprland VIRT1 ──(ext-image-copy-capture + dmabuf)──▶ capture.c ──▶ GL textures │
                                                            │                  │
       layout.c (wall placement) ───────────┐              ▼                  ▼
                                             └──────▶ render.c ──▶ fullscreen window on the glasses
                                                            ▲
       grab.c (trackpad → cursor/zoom) ─────────────────────┘
```

mirage captures the `VIRT1` headless output zero-copy, lays it out as a curved
equidistant wall, and draws it through a perspective camera rotated by the live
head pose. The trackpad is grabbed at the evdev level and re-injected as a virtual
pointer onto the wall.

## Head pose

The **RayNeo bridge** (`scripts/bridge.sh` → `rayneo-bridge`) reads the glasses' IMU
over hidraw, runs a Madgwick AHRS, and streams orientation as OpenTrack UDP on
`127.0.0.1:4242` (6 little-endian doubles `{x, y, z, yaw, pitch, roll}`, degrees) —
exactly what `pose.c` consumes. mirage auto-recenters on the first sample, so
"straight ahead" is wherever you're looking at launch (or double-tap Cmd any time).

- **6-axis** (gyro + accel) by default: the bridge's gyro-bias auto-zero makes it
  essentially drift-free at a desk, where the magnetometer's heading is corrupted by
  the laptop's field. 9-axis is available with a clean-environment mag calibration.
- **Axis frame:** the RayNeo IMU sits with `Y=up, X=left/right, Z=forward`, but the
  AHRS/euler convention expects `Z=up`. `rayneo.c` remaps every sensor by the proper
  rotation `(x,y,z) ← (z,x,y)` so head pitch lands on pitch (not roll).
- hidraw is root-only until a udev rule applies on a glasses replug, so the bridge
  uses `sudo` until then.

Check the stream without the renderer:

```bash
./mirage-posedump              # prints live quat + yaw/pitch/roll
```

## Configuration

There are no flags or environment variables — the single blessed configuration is in
[`src/config.c`](src/config.c): wall size & distance, curvature, FOV, the One-Euro
filter, reading deadband, sharpening, and the head-tracking gains. Edit and rebuild.

## Repository layout

| Path | What |
|------|------|
| `src/main.c` | Wayland setup, the frame loop, the fullscreen glasses window |
| `src/pose.c` | head-pose input (OpenTrack UDP), One-Euro smoothing, recenter |
| `src/capture.c` | `ext-image-copy-capture` → gbm/dmabuf → EGLImage → GLES2 texture |
| `src/render.c` | EGL/GLES2: curved textured wall, perspective camera, sharpen |
| `src/layout.c` | where the wall sits on the arc |
| `src/grab.c` | trackpad capture, arc cursor, zoom, gaze cursor |
| `src/config.c` | the one hardcoded scene/tracking configuration |
| `src/math3d.h` | vec / quat / mat4 (no glm dependency) |
| `src/rayneo*.c`, `src/magcal.c` | the RayNeo IMU bridge (Madgwick AHRS, mag calibration) |
| `scripts/` | start-mirage, glasses, bridge, setup/teardown-displays, sweep, keybinds, stop |
| `protocol/` | vendored wlroots/Hyprland protocol XML |

## Notes

- **Hyprland 0.55+** uses the Lua config parser, so the scripts drive it via
  `hyprctl eval` (`hl.monitor`, `hl.config`, `hl.dispatch`) rather than the legacy
  `hyprctl keyword`.
- **Asahi / Apple M2:** the glasses come up as a normal desktop output and are driven
  by direct scanout — no leasing or per-session DCP reboot.
- mirage holds an exclusive grab on the trackpad while running; quitting
  (`Super+Shift+Q`) releases it and restores your windows.
