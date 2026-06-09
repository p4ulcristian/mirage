#!/usr/bin/env bash
# keybinds.sh - register Hyprland keybinds to control mirage.
# Hyprland 0.55+ uses a Lua config; binds are registered via hyprctl eval.
# These are compositor-level binds so they fire even while mirage covers the glasses.
#
#   SUPER + SHIFT + Q   quit mirage + restore windows + remove virtual displays
#   ALT + C             recenter head pose
#
# NB: recenter is ALT+C, not SUPER+SHIFT+C. A mac-style keyd config remaps the cmd
# (Meta) layer's c/x/v/a/... to Ctrl equivalents, so SUPER+SHIFT+C arrives as
# Ctrl+Shift+C and never reaches Hyprland. keyd has no Alt layer, so ALT+C passes
# through untouched.
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
