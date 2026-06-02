#!/usr/bin/env bash
# bridge.sh - start the RayNeo head-tracking bridge (driver -> OpenTrack UDP).
# Streams head pose from the glasses to mirage on 127.0.0.1:4242.
#
# hidraw is root-only until the udev rule applies on a glasses replug, so this
# uses sudo. After you replug the glasses once, plain ./rayneo-bridge works.
set -euo pipefail
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
BIN="$HERE/rayneo-bridge"

[ -x "$BIN" ] || { echo "build it: make bridge"; exit 1; }

# don't compete with the (RayNeo-unaware) XRLinuxDriver
systemctl --user stop xr-driver 2>/dev/null || true

# already running?
if pgrep -f rayneo-bridge >/dev/null 2>&1; then
    echo "rayneo-bridge already running"; exit 0
fi

SUDO=""
[ -r /dev/hidraw3 ] || SUDO="sudo -n"
echo "starting rayneo-bridge ${SUDO:+(via sudo)}..."
# Default to 6-axis. The magnetometer (9-axis) is unreliable at a desk: the
# laptop's local field distorts "north" and drags heading around, drifting
# WORSE than 6-axis even after calibration. The bridge's gyro-bias auto-zero
# makes 6-axis essentially drift-free (measured ~0.00 deg/s at rest), so the mag
# buys nothing here. Override with AXIS=9 to use it (needs a good
# `rayneo-track --calibrate` done in a magnetically clean environment).
AXIS="${AXIS:-6}"; SIXFLAG=""; [ "$AXIS" = "6" ] && SIXFLAG="--6axis"
# setsid + </dev/null + disown fully detaches it so it outlives this launcher
setsid $SUDO "$BIN" -v $SIXFLAG "$@" >/tmp/bridge.log 2>&1 </dev/null &
disown 2>/dev/null || true
sleep 1.5
if pgrep -f rayneo-bridge >/dev/null 2>&1; then
    echo "bridge running -> 127.0.0.1:4242   (log: /tmp/bridge.log)"
    grep -E "fusion|opened" /tmp/bridge.log | head -2 || true
else
    echo "bridge failed to start:"; cat /tmp/bridge.log
    exit 1
fi
