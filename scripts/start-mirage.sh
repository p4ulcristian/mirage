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

# Glasses are OPTIONAL and HOT-PLUGGABLE. glasses.sh is now a supervisor: with the
# VITURE Beast / RayNeo plugged it runs the direct-scanout + head-tracking path; without
# it the SAME scene runs windowed on the laptop (trackpad swipes to look) - and it flips
# between the two LIVE as you unplug/replug mid-session. So this is just an opening status
# line; the real decision (and every later one) happens inside glasses.sh.
if hyprctl monitors all -j | grep -qE 'SmartGlasses|VITURE'; then
    echo "glasses detected — starting in the direct-scanout path (hot-plug aware)"
else
    echo "no glasses — starting windowed on the laptop (plug the glasses in any time)"
    notify "Mirage windowed on the laptop — plug the glasses in any time to switch to head tracking."
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
