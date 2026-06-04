#!/usr/bin/env bash
# mirror-laptop.sh - clone the glasses (DP-1) ONTO the laptop panel (eDP-1) so a
# bystander watching the laptop sees the same AR view as the glasses wearer.
#
#   mirror-laptop.sh on    eDP-1 mirrors the glasses
#   mirror-laptop.sh off   eDP-1 back to its normal standalone desktop
#
# Direction is deliberate: the LAPTOP mirrors the GLASSES, never the reverse. A
# mirrored output stops being published as its own wl_output, and mirage needs
# the glasses to stay a real output to render onto - so the glasses must never be
# the one set to mirror.
set -uo pipefail

# Laptop panel spec - must match the eDP-1 line in ~/.config/hypr/hyprland.conf.
LAPTOP="${LAPTOP:-eDP-1}"
LAPTOP_SPEC="${LAPTOP_SPEC:-2560x1600@60,0x0,1.6}"

command -v hyprctl >/dev/null || { echo "hyprctl not found (need Hyprland)"; exit 1; }

# Auto-detect the glasses output by description; fall back to the first DP-* then
# DP-1. Matches run.sh's detection so both agree on which output is the glasses.
glasses() {
    hyprctl monitors all -j | python3 -c '
import json,sys
mons=json.load(sys.stdin)
for m in mons:
    d=(m.get("description") or "")
    if "SmartGlasses" in d or "RayNeo" in d:
        print(m["name"]); break
else:
    print(next((m["name"] for m in mons if m["name"].startswith("DP-")), "DP-1"))
'
}

case "${1:-on}" in
    on)
        g="$(glasses)"
        echo "mirror: $LAPTOP -> $g"
        hyprctl keyword monitor "$LAPTOP,$LAPTOP_SPEC,mirror,$g" >/dev/null
        ;;
    off)
        echo "mirror: $LAPTOP restored to standalone"
        hyprctl keyword monitor "$LAPTOP,$LAPTOP_SPEC" >/dev/null
        ;;
    *)
        echo "usage: $0 on|off" >&2; exit 2;;
esac
