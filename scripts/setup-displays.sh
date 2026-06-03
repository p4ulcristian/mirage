#!/usr/bin/env bash
# setup-displays.sh - create and configure the virtual displays that mirage
# renders as floating screens on the glasses.
#
# They are placed far off to the right of your physical layout so they never
# interfere with the real desktop or steal the cursor; mirage captures them by
# name regardless of where they sit in the Hyprland layout.
#
# The displays are laid out in the SAME grid mirage draws them in - VCOLS wide,
# filling left->right then bottom->top (VIRT1..VCOLS on the bottom row). So
# Hyprland's spatial model matches the arc you see: dragging a window to the
# neighbour on your right/above lands where you'd expect. (VCOLS must match
# mirage's --screens column count, default 3.)
#
# Env overrides:
#   VW=1920 VH=1080 VR=60 VSCALE=1   resolution / refresh / scale
#   VCOUNT=6                         how many virtual displays
#   VCOLS=3                          columns in the grid (match mirage screen_cols)
#   VORIGIN=8000                     x of the leftmost virtual display
#   VGAP=200                         gap between adjacent virtual displays
set -euo pipefail

VW=${VW:-1920}; VH=${VH:-1080}; VR=${VR:-60}; VSCALE=${VSCALE:-1}
VCOUNT=${VCOUNT:-6}; VCOLS=${VCOLS:-3}; VORIGIN=${VORIGIN:-8000}; VGAP=${VGAP:-200}

command -v hyprctl >/dev/null || { echo "hyprctl not found (need Hyprland)"; exit 1; }

existing="$(hyprctl monitors all -j | tr ',' '\n' | grep -oE '"name": *"VIRT[0-9]+"' | grep -oE 'VIRT[0-9]+' || true)"

for i in $(seq 1 "$VCOUNT"); do
    name="VIRT$i"
    if ! grep -q "^$name$" <<<"$existing"; then
        echo "creating $name"
        hyprctl output create headless "$name" >/dev/null
    fi
done
sleep 0.3

# Configure each: lay them out in the SAME grid mirage draws (VCOLS wide), far
# to the right so the cursor never drifts in. Index i-1 -> column (left->right)
# and visual row (0 = bottom, at eye level, matching layout.c's lift). The
# compositor's +y is DOWN, so the top visual row gets the smallest y.
nrows=$(( (VCOUNT + VCOLS - 1) / VCOLS ))
for i in $(seq 1 "$VCOUNT"); do
    name="VIRT$i"
    idx=$(( i - 1 ))
    col=$(( idx % VCOLS ))
    vrow=$(( idx / VCOLS ))                  # visual row: 0 = bottom (eye level)
    crow=$(( nrows - 1 - vrow ))             # compositor row: 0 = top (smaller y)
    x=$(( VORIGIN + col * (VW + VGAP) ))
    y=$(( crow * (VH + VGAP) ))
    spec="$name,${VW}x${VH}@${VR},${x}x${y},${VSCALE}"
    echo "configuring monitor $spec"
    hyprctl keyword monitor "$spec" >/dev/null
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
