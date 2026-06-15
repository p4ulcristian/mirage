#!/usr/bin/env bash
# facecam.sh - start the webcam head-POSITION bridge (6DoF).
#
# The RayNeo IMU gives mirage head orientation (3DoF); this adds the missing
# position (lean in / slide sideways) by watching you through the laptop webcam and
# streaming head position to mirage over /tmp/mirage-facecam.sock. mirage fuses the
# two: rotation from the IMU, position from here -> real motion parallax.
#
# No marker needed: a face detector locates your head; the dark lenses don't matter
# because we only take POSITION from the camera (the IMU still owns rotation).
set -euo pipefail
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
BIN="$HERE/facecam-bridge"
MODEL="$HERE/assets/face_detection_yunet_2023mar.onnx"

[ -x "$BIN" ] || { echo "build it: make facecam"; exit 1; }

# already running?
if pgrep -f facecam-bridge >/dev/null 2>&1; then
    echo "facecam-bridge already running"; exit 0
fi

# Needs read access to the webcam. If the user isn't in the 'video' group this
# fails loudly in the log rather than silently giving no parallax.
echo "starting facecam-bridge..."
# setsid + </dev/null + disown fully detaches it so it outlives this launcher.
# --mirror is OFF by default; if leaning right shifts the wall the wrong way, either
# add --mirror here or flip facecam_lateral_gain in src/config.cpp.
setsid "$BIN" --model "$MODEL" --verbose >/tmp/facecam.log 2>&1 </dev/null &
disown 2>/dev/null || true
sleep 1.0
if pgrep -f facecam-bridge >/dev/null 2>&1; then
    echo "facecam running -> /tmp/mirage-facecam.sock   (log: /tmp/facecam.log)"
    grep -E "detector|fps" /tmp/facecam.log | head -2 || true
else
    echo "facecam failed to start:"; cat /tmp/facecam.log
    exit 1
fi
