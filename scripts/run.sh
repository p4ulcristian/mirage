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
# sliver. These rules make it open floating, sized, and on the glasses output.
if [[ " $* " == *" --windowed "* ]] || [[ " $* " == *" --windowed" ]]; then
    hyprctl keyword windowrulev2 "float,class:^(mirage)$"        >/dev/null
    hyprctl keyword windowrulev2 "size 1500 760,class:^(mirage)$" >/dev/null
    hyprctl keyword windowrulev2 "center,class:^(mirage)$"       >/dev/null
fi

# don't stack instances
if [ -f "$PIDFILE" ] && kill -0 "$(cat "$PIDFILE")" 2>/dev/null; then
    echo "mirage already running (pid $(cat "$PIDFILE")). Use scripts/stop.sh first."
    exit 1
fi

"$BIN" "$@" >"$LOG" 2>&1 &
echo $! > "$PIDFILE"
echo "mirage started (pid $(cat "$PIDFILE")), log: $LOG"
