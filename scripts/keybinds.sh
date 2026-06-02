#!/usr/bin/env bash
# keybinds.sh - register Hyprland keybinds to control mirage.
# These are compositor-level binds, so they fire even while the mirage overlay
# covers the glasses. Re-run after a Hyprland reload.
#
#   SUPER SHIFT Q   quit mirage + restore windows to a real monitor + remove virtual displays
#   SUPER SHIFT X   quit + CLOSE demo terminals + remove virtual displays
set -euo pipefail
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

hyprctl keyword bind "SUPER SHIFT, Q, exec, bash $HERE/scripts/stop.sh --restore" >/dev/null
hyprctl keyword bind "SUPER SHIFT, X, exec, bash $HERE/scripts/stop.sh --all" >/dev/null
hyprctl keyword bind "SUPER SHIFT, C, exec, pkill -USR1 -x mirage" >/dev/null
hyprctl keyword bind "SUPER, G, exec, pkill -USR2 -x mirage" >/dev/null

echo "registered:"
echo "  SUPER+SHIFT+Q  -> quit mirage + restore windows + remove virtual displays"
echo "  SUPER+SHIFT+X  -> quit mirage + close demo terminals + remove virtual displays"
echo "  SUPER+SHIFT+C  -> recenter head pose (look straight ahead, then press)"
echo "  SUPER+G        -> toggle mouse/keyboard capture onto the arc"
