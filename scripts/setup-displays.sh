#!/usr/bin/env bash
# setup-displays.sh - create and configure the virtual displays that mirage
# renders as floating screens on the glasses.
#
# They are placed far off to the right of your physical layout so they never
# interfere with the real desktop or steal the cursor; mirage captures them by
# name regardless of where they sit in the Hyprland layout.
#
# The display is laid out in the SAME grid mirage draws it in - VCOLS wide,
# filling left->right then bottom->top. So Hyprland's spatial model matches the
# arc you see: dragging a window to the neighbour on your right/above lands
# where you'd expect. (VCOLS matches mirage's screen_cols.)
set -euo pipefail

# ONE 32:9 ultrawide wall (VIRT1) at 5120x1440@60, scale 1.
VW=5120; VH=1440; VR=60; VSCALE=1
# VGAP=0: the virtual displays must ABUT in the compositor's coordinate space.
# Any gap is dead no-man's-land between outputs, and the cursor can only leap a
# dead gap with momentum - a slow drag parks at the screen edge and never
# crosses. Abutting them makes the wall one continuous surface so the cursor
# flows screen-to-screen at any speed. (Visual spacing in the glasses is mirage's
# arc rendering, unrelated to this layout gap.)
VCOUNT=1; VCOLS=1; VORIGIN=8000; VGAP=0

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
    echo "configuring monitor $name ${VW}x${VH}@${VR} at ${x},${y} scale ${VSCALE}"
    # Hyprland 0.55 uses the Lua (non-legacy) config parser, where `hyprctl keyword
    # monitor` is rejected ("keyword can't work with non-legacy parsers"). Set the
    # monitor through the Lua API via eval instead, or VIRT1 silently keeps its
    # 1920x1080 headless default and the wall renders 16:9 instead of 32:9.
    hyprctl eval "hl.monitor({ output='$name', mode='${VW}x${VH}@${VR}', position='${x}x${y}', scale=${VSCALE} })" >/dev/null
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
