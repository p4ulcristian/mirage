# mirage — head-tracked virtual displays for Hyprland + AR glasses

Turns N headless virtual outputs into floating screens arranged in a 3D arc,
captured live and rendered onto your AR glasses, with the camera driven by your
head pose. Built for Hyprland on Wayland; verified on RayNeo glasses (DP-1) with
an Apple M2 GPU (Asahi).

## Status

| Piece | State |
|-------|-------|
| 3 virtual displays (Hyprland headless) | ✅ working |
| Live screencopy → GL textures (zero-copy dmabuf) | ✅ working |
| 3D arc layout + perspective on the glasses | ✅ working |
| Head-pose camera (OpenTrack UDP / JSON socket) | ✅ working |
| Quit hotkeys | ✅ working |
| Super+G input grab/confine | ⏳ todo (grab.c stub) |
| RayNeo IMU driver | 🧑‍💻 you are writing this |

## Pose contract (how your IMU driver feeds mirage)

mirage consumes head orientation; your driver produces it. Pick one:

1. **OpenTrack UDP** (default) → `127.0.0.1:4242`, 6 little-endian doubles
   `{x, y, z, yaw, pitch, roll}`, angles in **degrees**. This is exactly what
   `xr_driver_cli --opentrack-app` already emits, so a RayNeo IMU plugin into
   XRLinuxDriver needs zero glue.
2. **JSON unix dgram socket** → `{"quat":[w,x,y,z]}` newline-delimited.

If turning your head feels reversed, flip an axis: `--invert-yaw` /
`--invert-pitch` / `--invert-roll`. mirage auto-recenters on the first sample,
so "straight ahead" = wherever you're looking at launch.

## Quick start

```bash
make && make bridge               # build mirage, posedump, and the RayNeo bridge
bash scripts/setup-displays.sh    # create/configure VIRT1..3
bash scripts/keybinds.sh          # register quit hotkeys
bash scripts/bridge.sh            # RayNeo head tracking -> OpenTrack UDP :4242
bash scripts/run.sh --3d          # head-tracked AR on the glasses
#   flat capture-only (no tracking):  scripts/run.sh --windowed
#   tuning:  scripts/run.sh --3d --fov 50 --spacing 22 --invert-yaw
```

The **RayNeo bridge** (`scripts/bridge.sh`) links the driver in
`../rayneo-air-pro-4`, runs its Madgwick AHRS, and streams head pose to mirage.
For drift-free yaw, calibrate the magnetometer once:
`sudo ../rayneo-air-pro-4/rayneo-track --calibrate` (then the bridge auto-loads
it for 9-axis fusion).

Verify your driver's pose stream without the renderer:

```bash
./mirage-posedump                 # prints live quat + yaw/pitch/roll
```

## Controls (Hyprland keybinds)

| Key | Action |
|-----|--------|
| `SUPER+SHIFT+Q` | quit mirage (clean shutdown) |
| `SUPER+SHIFT+X` | quit + close demo terminals + remove virtual displays |

These are compositor-level binds, so they fire even while the mirage overlay
covers the glasses. Re-run `scripts/keybinds.sh` after a Hyprland reload.

## Tuning (CLI flags)

```
--output NAME     render target (default: auto-detect glasses by description)
--port N          OpenTrack UDP port (default 4242)
--fov DEG         glasses vertical FOV (default 26)
--distance M      screen distance in metres (default 2.0)
--spacing DEG     yaw between adjacent screens (default 38)
--smooth F        pose smoothing 0..1 (default 0.35)
--screens N       expected virtual screen count (default 3)
--invert-yaw/pitch/roll
```

## Architecture

```
your IMU driver ──(OpenTrack UDP / JSON)──▶ pose.c ──▶ head quaternion
Hyprland VIRT1..3 ──(wlr-screencopy + dmabuf)──▶ capture.c ──▶ GL textures
                                                        │
   layout.c (arc placement) ─┐                          ▼
   pose (camera) ────────────┴──▶ render.c ──▶ layer-shell overlay on glasses
```

- `src/pose.c`      head-pose input (OpenTrack UDP / JSON socket), smoothing, recenter
- `src/capture.c`   wlr-screencopy → gbm/dmabuf → EGLImage → GLES2 texture
- `src/render.c`    EGL/GLES2, 3 textured quads, perspective camera from pose
- `src/layout.c`    where each screen sits on the arc
- `src/grab.c`      input confine (todo)
- `src/math3d.h`    vec/quat/mat4 (no glm dependency)
- `protocol/`       vendored wlroots/hyprland protocol XML
- `scripts/`        setup-displays, teardown-displays, run, stop, keybinds
```
