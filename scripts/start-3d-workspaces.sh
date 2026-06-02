#!/usr/bin/env bash
# start-3d-workspaces.sh - one-shot launcher for the wofi/start-menu app entry.
# Registers the Hyprland keybinds, then starts mirage in 3D head-tracked mode
# (run.sh brings up the bridge + virtual displays + window sweep itself).
#
# Safe to launch from a .desktop file: it inherits the Hyprland session env, so
# hyprctl works; it logs to /tmp and notifies on the obvious failure (glasses
# unplugged) rather than dying silently.
set -uo pipefail
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
LOG=/tmp/3d-workspaces-launch.log
exec >>"$LOG" 2>&1
echo "=== launch $(cat /proc/uptime | cut -d' ' -f1)s uptime ==="

notify() { command -v notify-send >/dev/null && notify-send "3D Workspaces" "$1" || true; }

# Glasses must be present as the DP-1 SmartGlasses output, extended (not
# mirrored). If only eDP-1 is advertised, there's nothing to render onto.
if ! hyprctl monitors all -j | grep -q SmartGlasses; then
    echo "glasses (DP-1 SmartGlasses) not found"
    notify "Glasses not detected — plug in the RayNeo and make sure they're extended, not mirrored."
    exit 1
fi

bash "$HERE/scripts/keybinds.sh"
bash "$HERE/scripts/run.sh" --3d
notify "Started — Super+G grab · Super+Shift+C recenter · Super+Shift+Q quit"
