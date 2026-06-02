#!/usr/bin/env bash
# run.sh - launch the mirage AR compositor in the background.
# Passes through any extra args (e.g. --fov 50 --spacing 18).
set -euo pipefail
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
BIN="$HERE/mirage"
PIDFILE=/tmp/mirage.pid
LOG=/tmp/mirage.log

[ -x "$BIN" ] || { echo "build first: make"; exit 1; }

# --3d needs head tracking: make sure the RayNeo bridge is running first.
# (bridge.sh is a no-op if it's already up.)
if [[ " $* " == *" --3d "* ]] || [[ " $* " == *" --3d" ]]; then
    bash "$HERE/scripts/bridge.sh" 2>&1 | sed 's/^/  [bridge] /' || true
fi

# In windowed scene-setup mode, Hyprland would otherwise tile the window to a
# sliver. We render via a normal window because this compositor/Asahi DCP setup
# does NOT publish the glasses as a bindable wl_output (mirage's grab-the-output
# mode can't see it). A window doesn't need a wl_output - Hyprland places it on
# the glasses by name - so we send it there fullscreen at native 120Hz.
if [[ " $* " == *" --windowed "* ]] || [[ " $* " == *" --windowed" ]]; then
    # auto-detect the glasses output (by description), fall back to DP-1
    GLASSES="$(hyprctl monitors all -j | python3 -c '
import json,sys
mons=json.load(sys.stdin)
for m in mons:
    if "SmartGlasses" in (m.get("description") or "") or "RayNeo" in (m.get("description") or ""):
        print(m["name"]); break
else:
    print(next((m["name"] for m in mons if m["name"].startswith("DP-")), "DP-1"))
')"
    hyprctl keyword windowrulev2 "monitor $GLASSES,class:^(mirage)$" >/dev/null
    hyprctl keyword windowrulev2 "fullscreen,class:^(mirage)$"       >/dev/null
fi

# don't stack instances
if [ -f "$PIDFILE" ] && kill -0 "$(cat "$PIDFILE")" 2>/dev/null; then
    echo "mirage already running (pid $(cat "$PIDFILE")). Use scripts/stop.sh first."
    exit 1
fi

# Ensure the virtual displays exist (idempotent: skips any already present).
# stop.sh tears them down on quit, so we recreate them on every launch -
# otherwise the first start after a quit would have nothing to capture.
# VR=120 gives the glasses a 120 Hz source; override with VR=… if needed.
VR="${VR:-120}" bash "$HERE/scripts/setup-displays.sh" 2>&1 | sed 's/^/  [displays] /' || true

# Sweep every real window onto the arc (snapshotted; stop.sh --restore puts them
# back). Non-fatal: if the sweep fails we still launch the renderer.
bash "$HERE/scripts/sweep.sh" sweep 2>&1 | sed 's/^/  [sweep] /' || true

"$BIN" "$@" >"$LOG" 2>&1 &
echo $! > "$PIDFILE"
echo "mirage started (pid $(cat "$PIDFILE")), log: $LOG"

# Auto-grab the mouse/keyboard onto the arc once the renderer is up. This sends
# the same SIGUSR2 toggle that Super+G uses, after waiting for the first frame
# so the grab subsystem is ready. Override with AUTOGRAB=0 to start ungrabbed.
if [ "${AUTOGRAB:-1}" = "1" ]; then
    ( for _ in $(seq 1 50); do
          grep -q -e 'fps' -e 'capture\[' "$LOG" 2>/dev/null && break
          sleep 0.1
      done
      pkill -USR2 -x mirage ) >/dev/null 2>&1 &
    echo "  (auto-grab armed; press Super+G to release input back to the laptop)"
fi
