#!/usr/bin/env bash
# setup-displays.sh - create and configure the virtual displays that mirage
# renders as floating screens on the glasses.
#
# They are placed far off to the right of your physical layout so they never
# interfere with the real desktop or steal the cursor; mirage captures them by
# name regardless of where they sit in the Hyprland layout.
#
# The displays match mirage's screen order (VIRT1, VIRT2, ...) and config.c's
# per-screen geometry. The wall is a 3-column arrangement:
#   col 0 (left)  : VIRT3  1080x2160 portrait
#   col 1 (centre): VIRT1 (top) + VIRT2 (bottom), 1920x1080 stacked
#   col 2 (right) : VIRT4  1080x2160 portrait
# They're placed far off-screen right, stacked so they never overlap; mirage
# captures them by name, so the exact desktop layout only matters for keeping
# them out of the real desktop.
set -euo pipefail

# Per-display geometry, in mirage's capture order (VIRT1, VIRT2, VIRT3, VIRT4).
# Two Full-HD 16:9 for the centre stack + two 1080x2160 portraits for the sides.
NAMES=(VIRT1 VIRT2 VIRT3 VIRT4)
WS=(1920 1920 1080 1080)        # widths
HS=(1080 1080 2160 2160)        # heights
VR=60; VSCALE=1; VORIGIN=8000

command -v hyprctl >/dev/null || { echo "hyprctl not found (need Hyprland)"; exit 1; }

existing="$(hyprctl monitors all -j | tr ',' '\n' | grep -oE '"name": *"VIRT[0-9]+"' | grep -oE 'VIRT[0-9]+' || true)"

for name in "${NAMES[@]}"; do
    if ! grep -q "^$name$" <<<"$existing"; then
        echo "creating $name"
        hyprctl output create headless "$name" >/dev/null
    fi
done
sleep 0.3

# Arrange to MATCH the AR stack so window-drags + the cursor behave: every screen
# horizontally CENTERED on the widest one (the wall), stacked vertically with the
# wall at the bottom (index 0 = eye level) and later screens above it. The
# compositor's +y is DOWN, so the top screen gets the smallest y. Hyprland 0.55
# rejects `hyprctl keyword monitor`, so set monitors via the Lua API (eval).
maxw=0
for w in "${WS[@]}"; do if [ "$w" -gt "$maxw" ]; then maxw=$w; fi; done
cx=$(( VORIGIN + maxw / 2 ))                    # shared horizontal centre
ytop=0
for (( i=${#NAMES[@]}-1; i>=0; i-- )); do       # highest index = top row, place first
    name="${NAMES[$i]}"; w="${WS[$i]}"; h="${HS[$i]}"
    x=$(( cx - w / 2 )); y=$ytop; ytop=$(( ytop + h ))
    echo "configuring monitor $name ${w}x${h}@${VR} at ${x},${y} scale ${VSCALE} (centered)"
    hyprctl eval "hl.monitor({ output='$name', mode='${w}x${h}@${VR}', position='${x}x${y}', scale=${VSCALE} })" >/dev/null
done

echo
echo "virtual displays ready:"
hyprctl monitors all -j | python3 -c '
import json,sys
for m in json.load(sys.stdin):
    n = m["name"]
    if n.startswith("VIRT"):
        print("  %s: %dx%d@%.0f at %d,%d scale %s" % (
            n, m["width"], m["height"], m["refreshRate"], m["x"], m["y"], m["scale"]))
'
