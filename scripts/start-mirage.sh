#!/usr/bin/env bash
# start-mirage.sh - one-shot launcher for the wofi/start-menu app entry.
# Registers the Hyprland keybinds, then starts mirage via glasses.sh - the
# DIRECT-SCANOUT path (locked ~120Hz), not the slower layer-shell overlay.
# glasses.sh is self-contained: it sets render:direct_scanout + fullscreen/opaque
# windowrules, pauses HyprPanel off the glasses output + parks the cursor (both
# restored on quit), brings up the VIRT displays + RayNeo bridge + window sweep,
# then runs mirage fullscreen and BLOCKS until you quit it (Super+Shift+Q), at
# which point its trap restores the desktop.
#
# Safe to launch from a .desktop file: it inherits the Hyprland session env, so
# hyprctl works; it logs to /tmp and notifies on the obvious failure (glasses
# unplugged) rather than dying silently.
set -uo pipefail
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
LOG=/tmp/mirage-launch.log
exec >>"$LOG" 2>&1
echo "=== launch $(cat /proc/uptime | cut -d' ' -f1)s uptime ==="

notify() { command -v notify-send >/dev/null && notify-send "3D Workspaces" "$1" || true; }

# Glasses must be present as the DP-1 SmartGlasses output, extended (not mirrored).
# (The non-desktop quirk is reverted, so they come up as a normal desktop output
# again and direct-scanout drives them - no leasing, no per-session DCP reboot.)
# If only eDP-1 is advertised, there's nothing to render onto.
if ! hyprctl monitors all -j | grep -q SmartGlasses; then
    echo "glasses (DP-1 SmartGlasses) not found"
    notify "Glasses not detected — plug in the RayNeo and make sure they're extended, not mirrored."
    exit 1
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
