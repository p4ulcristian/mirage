#!/usr/bin/env bash
# viture-bridge.sh - start the VITURE (Beast) head-tracking bridge -> mirage UDP.
# The RayNeo equivalent is bridge.sh.
#
# Needs VITURE's v2.0.0 aarch64 SDK (libglasses.so + libcarina_vio.so). It isn't
# publicly downloadable; grab it from wheaney/XRLinuxDriver (lib/aarch64/viture/)
# and put it where this script can find it - set VITURE_SDK, or drop it in
# ./viture-sdk/ at the repo root.
#
# Runs under sudo: the SDK drives the glasses via libusb (needs /dev/bus/usb) and
# we must unbind the kernel cdc_acm driver from the Beast's control interfaces
# first (the bridge does that itself, but it needs root).
set -euo pipefail
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
BIN="$HERE/viture-bridge"
[ -x "$BIN" ] || { echo "build it: make viture"; exit 1; }

# Find the SDK dir (holds libglasses.so).
SDK="${VITURE_SDK:-}"
if [ -z "$SDK" ]; then
    for d in "$HERE/viture-sdk" "$HERE" /usr/local/lib/viture /usr/lib/viture; do
        [ -e "$d/libglasses.so" ] && { SDK="$d"; break; }
    done
fi
[ -n "$SDK" ] || { echo "libglasses.so not found - set VITURE_SDK to the aarch64 SDK dir"; exit 1; }

systemctl --user stop xr-driver 2>/dev/null || true   # don't fight XRLinuxDriver
LOG=/tmp/viture-bridge.log
# Keep a HEALTHY bridge, replace only a WEDGED one. The Beast's IMU can be fragile:
# a fresh SDK start sometimes comes up stuck (no raw callbacks) and re-cycling it
# rapidly only makes that worse, so never kill a bridge that's actually streaming.
# Detect streaming by the sample counter ("... n NNNN") advancing over ~1s.
# (exact-match the binary name; pgrep -f would also match this script + the greps)
if pgrep -x viture-bridge >/dev/null 2>&1; then
    # NB: grep exits 1 when the log has no "n NNNN" sample line - i.e. exactly when
    # the bridge is WEDGED. Under `set -o pipefail` that 1 propagates and `set -e`
    # would abort the script HERE, before we ever restart the dead bridge (the cold
    # -boot failure mode). The `|| true` keeps us going so the wedged path runs.
    n1=$(tail -2 "$LOG" 2>/dev/null | grep -a -o 'n [0-9]*' | tail -1 || true)
    sleep 1
    n2=$(tail -2 "$LOG" 2>/dev/null | grep -a -o 'n [0-9]*' | tail -1 || true)
    # Don't keep a stale binary: if viture-bridge was rebuilt AFTER the running one
    # started, the healthy-bridge keep below would silently run the OLD code (so a
    # fresh build's changes never take effect). Force a restart when $BIN is newer.
    rpid=$(pgrep -x viture-bridge | head -1)
    binm=$(stat -c %Y "$BIN" 2>/dev/null || echo 0)
    et=$(ps -o etimes= -p "$rpid" 2>/dev/null | tr -d ' ' || true); et=${et:-0}
    started=$(( $(date +%s) - et ))
    if [ -n "$n1" ] && [ "$n1" != "$n2" ] && [ "$binm" -le "$started" ]; then
        echo "viture-bridge already streaming ($n2) - keeping it"
        exit 0
    fi
    [ "$binm" -gt "$started" ] && echo "viture-bridge binary is newer than the running one - restarting to pick it up..."
    echo "existing viture-bridge is wedged (no samples) - restarting..."
    sudo -n pkill -x viture-bridge 2>/dev/null || pkill -x viture-bridge 2>/dev/null || true
    for _ in $(seq 1 20); do pgrep -x viture-bridge >/dev/null 2>&1 || break; sleep 0.2; done
    sleep 1   # let cdc_acm/USB settle before re-claiming (rapid re-claim wedges it)
fi

echo "starting viture-bridge (sudo; sdk: $SDK)..."
# setsid + </dev/null + disown so it outlives this launcher. sudo -E keeps VITURE_SDK;
# pass LD_LIBRARY_PATH so libglasses.so's NEEDED libcarina_vio.so resolves.
setsid sudo -n -E env VITURE_SDK="$SDK" LD_LIBRARY_PATH="$SDK" \
    "$BIN" --lib "$SDK" -v >/tmp/viture-bridge.log 2>&1 </dev/null &
disown 2>/dev/null || true
sleep 2
if pgrep -f viture-bridge >/dev/null 2>&1; then
    echo "bridge running -> 127.0.0.1:4242   (log: /tmp/viture-bridge.log)"
    grep -E "streaming|unbound|loaded|no IMU|failed" /tmp/viture-bridge.log | head -4 || true
else
    echo "bridge failed to start:"; tail -8 /tmp/viture-bridge.log
    exit 1
fi
