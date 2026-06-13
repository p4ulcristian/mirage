#!/usr/bin/env bash
# calibrate-if-needed.sh - run the calibration pre-flight (mirage-cal) ONCE, on
# first launch. Safe to call from a Hyprland bind / .desktop with NO terminal
# attached: mirage-cal is a TUI, so when there's no controlling terminal we open
# a terminal emulator for it and BLOCK until you finish (save + quit). No-op once
# a profile exists, so it costs nothing on every later start.
set -uo pipefail
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$HERE"

PROFILE="${MIRAGE_PROFILE:-${XDG_CONFIG_HOME:-$HOME/.config}/mirage/profile.toml}"
[ -f "$PROFILE" ] && exit 0          # already calibrated -> nothing to do

# The live axis check needs the tracker streaming, so make sure the bridge is up
# (idempotent; glasses.sh starts it too).
bash "$HERE/scripts/bridge.sh" >/dev/null 2>&1 || true

if [ -t 1 ]; then
    "$HERE/mirage-cal"               # already in a terminal (manual run)
    exit 0
fi

# Launched from a shortcut: find a terminal emulator and run mirage-cal in it,
# waiting for the window to close.
term=""
for t in "${TERMINAL:-}" kitty alacritty foot ghostty xterm konsole; do
    [ -n "$t" ] && command -v "$t" >/dev/null 2>&1 && { term="$t"; break; }
done
if [ -z "$term" ]; then
    command -v notify-send >/dev/null && notify-send "3D Workspaces" \
        "No terminal found for first-run calibration. Run  ./mirage-cal  once, then start again."
    exit 0
fi

case "$term" in
    wezterm) "$term" start --  "$HERE/mirage-cal" ;;
    *)       "$term" -e "$HERE/mirage-cal" ;;     # kitty/alacritty/foot/ghostty/xterm/konsole
esac
