#!/usr/bin/env bash
# teardown-displays.sh - remove the virtual displays mirage created.
set -euo pipefail
VCOUNT=${VCOUNT:-6}
for i in $(seq 1 "$VCOUNT"); do
    name="VIRT$i"
    if hyprctl monitors all -j | grep -q "\"name\": *\"$name\""; then
        echo "removing $name"
        hyprctl output remove "$name" >/dev/null || true
    fi
done
echo "done."
