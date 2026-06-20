#!/usr/bin/env bash
# start-mirage.sh - one-shot launcher for the wofi/start-menu app entry.
# Registers the Hyprland keybinds, then starts mirage via glasses.sh - the
# DIRECT-SCANOUT path (locked ~120Hz), not the slower layer-shell overlay.
# glasses.sh is self-contained: it sets render:direct_scanout + fullscreen/opaque
# windowrules, pauses HyprPanel off the glasses output + parks the cursor (both
# restored on quit), brings up the VIRT displays + head-tracking bridge (VITURE
# Beast or RayNeo, per glasses.sh) + window sweep,
# then runs mirage fullscreen and BLOCKS until you quit it (Super+Shift+Q), at
# which point its trap restores the desktop.
#
# Safe to launch from a .desktop file: it inherits the Hyprland session env, so
# hyprctl works; it logs to /tmp. Glasses are optional — with them you get the
# direct-scanout/head-tracking path, without them the same scene runs windowed.
set -uo pipefail
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
LOG=/tmp/mirage-launch.log
exec >>"$LOG" 2>&1
echo "=== launch $(cat /proc/uptime | cut -d' ' -f1)s uptime ==="

notify() { command -v notify-send >/dev/null && notify-send "3D Workspaces" "$1" || true; }

# Glasses are OPTIONAL. If a DP output matching the RayNeo ("SmartGlasses") or the
# VITURE Beast ("VITURE") is present, glasses.sh takes the direct-scanout + head-
# tracking path. If not, it runs the SAME scene windowed on the laptop, driven by
# trackpad swipes instead of the IMU. Either way we continue — one launcher, both
# modes (put the glasses on / take them off; relaunch picks the right path).
if hyprctl monitors all -j | grep -qE 'SmartGlasses|VITURE'; then
    echo "glasses detected — direct-scanout path"
else
    echo "no glasses — windowed desktop mode on the laptop"
    notify "No glasses — running Mirage windowed on the laptop (trackpad swipes to look)."
fi

# Clean start: if a previous session is still up, stop it and wait for the VIRT
# displays to tear down first, otherwise this instance dies in the teardown race.
if pgrep -x mirage >/dev/null; then
    echo "existing mirage running; stopping it before relaunch..."
    pkill -x mirage
    for _ in $(seq 1 20); do pgrep -x mirage >/dev/null || break; sleep 0.5; done
    sleep 2
fi

bash "$HERE/scripts/keybinds.sh"
# notify BEFORE glasses.sh - it runs mirage in the foreground and blocks here
# until you quit, then restores the desktop.
notify "Started — double-Cmd recenter · Super+Shift+Q quit (trackpad capture is always on)"
# Capture is always-on (mirage grabs the trackpad from the first frame); quit with
# Super+Shift+Q.
bash "$HERE/scripts/glasses.sh"
