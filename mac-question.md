# mac-question.md — stuff to grab from the Mac (for the Beast anchor fix)

Hey — read this on the Mac. Context first, then the exact commands. Should take ~5 min.

## Why I need this

The "won't stay anchored in space" drift on the Beast = we're only running the IMU
(6-axis), which has no absolute heading reference, so yaw slowly walks. The real fix
is VITURE's **Carina VIO** (camera-fused SLAM) — the same thing that makes SpaceWalker
rock-solid on your Mac.

We already have that engine on Linux: `viture-sdk/libcarina_vio.so`. It **boots fine**.
But when we feed it our hand-written camera-calibration config, its `CameraUndistort`
step **rejects it and calls `exit()`** — so the VIO never starts.

It's the *same engine* on Mac and Linux. The only difference is the **config** SpaceWalker
feeds it (one it accepts) vs the one I guessed at. So if I can see SpaceWalker's real
config + calibration, I can match the format exactly and get past the wall.

## What to run on the Mac

Paste each block into Terminal. Then either paste the output back to me, or save it into
the repo (see "Where to put it" at the bottom) and bring it over.

### 1. The config YAMLs (THE important one)

```bash
ls -la "/Applications/SpaceWalker.app/Contents/Resources/"*.yaml
echo "============================================================"
for f in "/Applications/SpaceWalker.app/Contents/Resources/"*.yaml; do
  echo "===== $f ====="; cat "$f"; echo
done
```

I most want these two (the doc only half-quoted them):
- `slam_config_*.yaml`           ← the VIO/SLAM params
- `viture_override_config_*.yaml` ← camera intrinsics + `T_cam_imu` (the bit being rejected)

### 2. What config/calibration it actually LOADS at runtime

Plug the Beast into the Mac, launch SpaceWalker, move your head a bit so tracking starts,
then:

```bash
tail -100 ~/Library/Application\ Support/com.viture.spacewalker/spacewalker.log
```

(If that path is empty, find the real one:)
```bash
find ~/Library -iname "*spacewalker*" 2>/dev/null
find ~/Library -iname "*.log" -path "*iture*" 2>/dev/null
```

### 3. Any per-device calibration files (the Beast's fisheye numbers)

```bash
# anything that looks like calibration the app cached for the connected glasses
find ~/Library -iname "*calib*" 2>/dev/null | grep -i -E "viture|spacewalk" 
ls -la "/Applications/SpaceWalker.app/Contents/Resources/" | grep -iE "calib|cam|imu|N6|beast|carina|\.bin|\.yaml|\.json"
```

### 4. (only if quick) the exact camera-model vocabulary the engine expects

```bash
# the dylib version of the same engine — confirms the camera_model / distortion_model strings
find "/Applications/SpaceWalker.app" -iname "libcarina_vio*" -o -iname "libSlam*" 2>/dev/null
# if found, e.g.:
# strings <path-to-libcarina_vio.dylib> | grep -iE "pinhole|fisheye|equidistant|radtan|atan|kannala|camera_model|distortion_model" | sort -u
```

### 5. The COMPLETE Resources tree (so we miss nothing)

```bash
find "/Applications/SpaceWalker.app/Contents/Resources" -maxdepth 3 -type f \
  | sed "s|/Applications/SpaceWalker.app/Contents/Resources/||" | sort
echo "----- sizes of the interesting ones -----"
find "/Applications/SpaceWalker.app/Contents/Resources" -type f \
  \( -iname "*.yaml" -o -iname "*.json" -o -iname "*.bin" -o -iname "*.txt" \
     -o -iname "*vocab*" -o -iname "*orb*" -o -iname "*dbow*" -o -iname "*calib*" \) \
  -exec ls -la {} \;
```

### 6. ORB vocabulary / map database (THIS is what makes anchoring *persistent*)

Pure VIO still drifts slowly over minutes; the thing that makes SpaceWalker lock
truly solid is loop-closure against an ORB vocabulary (DBoW) + map. Find it:

```bash
find "/Applications/SpaceWalker.app" -type f \
  \( -iname "*vocab*" -o -iname "*orb*" -o -iname "*dbow*" -o -iname "*.bin" \) -exec ls -la {} \;
```
If there's a vocab/`.bin` of a few MB, that's it — I need the file itself (copy it into
`mac-dump/`, see bottom). Note its exact filename/path so I can set `orb_database_path`.

### 7. Mono or stereo? what frames does it feed the VIO?

Carina's config `resolution:` must exactly match the camera frames we push in. Need to
know if the Beast VIO is mono or stereo and at what size. The override yaml usually shows
`cam0:` (and `cam1:` if stereo) with a `resolution:`. Also check the log for camera open
lines:

```bash
grep -iE "cam|stereo|mono|resolution|width|height|fisheye|undistort" \
  ~/Library/Application\ Support/com.viture.spacewalker/spacewalker.log | tail -40
```

### 8. The engine binary itself (confirm it matches ours + dump its config schema)

```bash
find "/Applications/SpaceWalker.app" -iname "libcarina_vio*" -o -iname "libSlam*"
# for whichever path it prints:
LIB="<paste libcarina_vio path here>"
strings "$LIB" | grep -iE "pinhole|fisheye|equidistant|radtan|atan|kannala|camera_model|distortion_model|T_cn_cnm1|T_imu_cam|T_cam_imu|orb_database|resolution" | sort -u
```

## What I'll do with it

- Match `viture-vio`'s inline config (`src/viture_vio.cpp`, the `CONFIG_YAML` string) to
  the real SpaceWalker format so `CameraUndistort` accepts it.
- Plug in the Beast's actual fisheye intrinsics + `T_cam_imu` extrinsics.
- Then we test the live VIO anchor once the Beast is back on the Linux box.

## Where to put it (so it comes back here)

Easiest: just paste the terminal output back to me in chat.

Or, if the repo is on the Mac too, drop the files into a folder and commit:
```bash
mkdir -p mac-dump
# copy the yamls in:
cp "/Applications/SpaceWalker.app/Contents/Resources/"*.yaml mac-dump/ 2>/dev/null
# paste the log tail + any findings into mac-dump/notes.txt
git add mac-dump && git commit -m "mac-dump: SpaceWalker Carina configs for Beast VIO"
```

Thanks 🙏 — this is the missing piece.
