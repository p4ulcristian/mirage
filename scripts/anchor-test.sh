#!/usr/bin/env bash
# anchor-test.sh - world-fixed boresight reticle stability test.
#
# Renders a single point/cross FIXED IN SPACE at dead-ahead so you can see how
# fixed the anchor really is: line the cross up on a real-world point, then move
# your head and watch it drift. Two reticles are drawn from the same eye:
#
#   GREEN = the shipped view  (forward-prediction + yaw/pitch gain + read
#           deadband baked in) - exactly what your anchored screens experience.
#   RED   = the RAW IMU pose  (no prediction / no gain / no deadband).
#
# Both are additive, so where they overlap you see YELLOW. Reading it:
#   * overlapped & rock-still            -> anchor is solid.
#   * RED still but GREEN swims/lags     -> the COMFORT pipeline is the cause
#                                           (tune pose_predict_ms / yaw_gain /
#                                            pitch_gain / read_deadband_deg).
#   * RED itself jitters or drifts       -> it's UPSTREAM (AHRS/IMU fusion).
# Tick marks sit at +-2/5/8 deg so any split is readable in degrees.
#
#   bash scripts/anchor-test.sh
#
# Recenter (double-Cmd) while looking straight at the cross so it starts aligned.
# Everything else (screens, HUD, pet) is hidden for a clean read; the star dome
# stays as a second world-fixed reference. Quit with Super+Shift+Q as usual.
#
# Optional extra isolation (the reticle already strips these for RED, but these
# also change what GREEN does so you can bisect on the glasses directly):
#   MIRAGE_PREDICT_MS=0    bash scripts/anchor-test.sh   # kill forward-prediction
#   MIRAGE_WORLDVIO_GAIN=0 bash scripts/anchor-test.sh   # kill optical-flow parallax
set -uo pipefail
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
export MIRAGE_ANCHOR_TEST=1
exec bash "$HERE/scripts/glasses.sh"
