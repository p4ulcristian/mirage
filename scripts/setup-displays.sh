#!/usr/bin/env bash
# setup-displays.sh - create and configure the virtual displays that mirage
# renders as floating screens on the glasses.
#
# They are placed far off to the right of your physical layout so they never
# interfere with the real desktop or steal the cursor; mirage captures them by
# name regardless of where they sit in the Hyprland layout.
#
# The displays match mirage's screen order (VIRT1, VIRT2, ...) and config.c's
# per-screen geometry: VIRT1 is the 32:9 DFHD wall, VIRT2 the 16:9 monitor mirage
# draws curved above it. They're placed far off-screen right, stacked so they
# never overlap; mirage captures them by name, so the exact layout only matters
# for keeping them out of the real desktop.
set -euo pipefail

# Per-display geometry (index 0 = VIRT1, index 1 = VIRT2). DFHD wall + Full-HD 16:9.
# 1080-tall maps ~1:1 onto the glasses' 1080p panel, so text stays crisp.
NAMES=(VIRT1 VIRT2)
WS=(3840 1920)        # widths
HS=(1080 1080)        # heights
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
