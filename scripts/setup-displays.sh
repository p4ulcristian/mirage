#!/usr/bin/env bash
# setup-displays.sh - create and configure the 3 virtual displays that mirage
# renders as floating screens on the glasses.
#
# They are placed far off to the right of your physical layout so they never
# interfere with the real desktop or steal the cursor; mirage captures them by
# name regardless of where they sit in the Hyprland layout.
#
# Env overrides:
#   VW=1920 VH=1080 VR=60 VSCALE=1   resolution / refresh / scale
#   VCOUNT=3                         how many virtual displays
#   VORIGIN=8000                     x of the first virtual display
#   VGAP=200                         gap between stacked virtual displays
set -euo pipefail

VW=${VW:-1920}; VH=${VH:-1080}; VR=${VR:-60}; VSCALE=${VSCALE:-1}
VCOUNT=${VCOUNT:-3}; VORIGIN=${VORIGIN:-8000}; VGAP=${VGAP:-200}

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

# Configure each: stack vertically far to the right so cursor never drifts in.
y=0
for i in $(seq 1 "$VCOUNT"); do
    name="VIRT$i"
    x=$VORIGIN
    spec="$name,${VW}x${VH}@${VR},${x}x${y},${VSCALE}"
    echo "configuring monitor $spec"
    hyprctl keyword monitor "$spec" >/dev/null
    y=$(( y + VH + VGAP ))
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
