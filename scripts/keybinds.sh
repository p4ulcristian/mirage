#!/usr/bin/env bash
# keybinds.sh - register Hyprland keybinds to control mirage.
# Hyprland 0.55+ uses a Lua config; binds are registered via hyprctl eval.
# These are compositor-level binds so they fire even while mirage covers the glasses.
#
#   SUPER + SHIFT + Q   quit mirage + restore windows + remove virtual displays
#
# Recenter is handled inside mirage itself: double-tap Cmd (Super) while the
# glasses view is active. It reads libinput directly, so it survives restarts
# without needing a compositor bind re-registered here.
set -euo pipefail
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

eval_bind() {
    hyprctl eval "$1" >/dev/null
}

eval_bind "local d=\"$HERE\"; hl.bind(\"SUPER + SHIFT + Q\", hl.dsp.exec_cmd(\"bash \" .. d .. \"/scripts/stop.sh --restore\"))"
eval_bind "hl.bind(\"ALT + C\", hl.dsp.exec_cmd(\"pkill -USR1 -x mirage\"))"

echo "registered:"
echo "  SUPER+SHIFT+Q  -> quit mirage + restore windows + remove virtual displays"
echo "  ALT+C          -> recenter head pose (look straight ahead, then press)"
