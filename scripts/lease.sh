#!/usr/bin/env bash
# lease.sh - run the SCREENSHARE mirage on the LEASED glasses.
#
# Why leasing: the apple-dcp kernel patch flags the glasses (DP-1, TCL 0x03d4)
# as non-desktop, so Hyprland withdraws them from the desktop and offers them
# via wp_drm_lease. mirage leases DP-1 and scans out DIRECTLY (GBM + atomic
# page-flips) - Hyprland never touches that output, so NONE of the old
# direct-scanout dance is needed (no HyprPanel quit, no watchdog, no cursor
# parking, no windowrules, no restore trap). Quit with Super+Shift+Q or pkill.
#
# Content model (unlike host-lane): this still STREAMS. setup-displays.sh brings
# up one 32:9 ultrawide VIRT display; you arrange your apps on it with normal
# Hyprland, and mirage captures it and floats it as the wall. The glasses output
# (lease) and the content (capture) are fully independent.
set -uo pipefail
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"; cd "$HERE"
[ -x ./mirage ] || make || exit 1

# Sanity: are the glasses actually offered for lease? (kernel patch + plugged in)
if ! hyprctl monitors all -j 2>/dev/null | grep -q SmartGlasses; then
    echo "glasses (DP-1) not detected — plug them in"; exit 1
fi

# Trackpad/keyboard for the in-app libinput grab (cursor + click on the wall).
detect_input(){ awk -v re="$1" '/^N: Name=/{match($0,re);h=RSTART>0}
  h&&/^H: Handlers=/{n=split($0,f,/[ \t]/);for(i=1;i<=n;i++)if(f[i]~/^event[0-9]+$/){print "/dev/input/"f[i];exit}}' /proc/bus/input/devices; }
: "${MIRAGE_TRACKPAD:=$(detect_input 'multi-touch|[Tt]ouchpad|[Tt]rackpad')}"
: "${MIRAGE_KEYBOARD:=$(detect_input '[Kk]eyboard')}"
export MIRAGE_TRACKPAD MIRAGE_KEYBOARD
echo "[lease] trackpad=${MIRAGE_TRACKPAD:-?} keyboard=${MIRAGE_KEYBOARD:-?}"

# Hyprland keybinds (Super+Shift+Q quit, Super+Shift+C recenter) + head tracking.
bash "$HERE/scripts/keybinds.sh" >/dev/null 2>&1 || true
bash "$HERE/scripts/bridge.sh"   >/dev/null 2>&1 || true

# Content: one 32:9 ultrawide VIRT display at 60 Hz (setup-displays.sh defaults).
# This is what mirage captures and shows as the wall - arrange apps on it with
# normal Hyprland (it sits far to the right so it never steals your cursor).
bash "$HERE/scripts/setup-displays.sh" || true

# MIRAGE_FPS_CAP: steady render cadence for the lease pacer (lease_out.c).
# 60 = locked, tear-free on the 120Hz panel (every 2nd vblank). (Pose prediction
# / MIRAGE_PREDICT_MS is a host-lane feature, not on this streaming branch.)
export MIRAGE_FPS_CAP="${MIRAGE_FPS_CAP:-60}"
echo "[lease] pacing=${MIRAGE_FPS_CAP}Hz"
echo "[lease] launching screenshare mirage on the leased glasses (Super+Shift+Q to quit)"
exec ./mirage --lease "$@" 2>&1 | tee /tmp/mirage.log
