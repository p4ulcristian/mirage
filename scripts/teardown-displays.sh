#!/usr/bin/env bash
# teardown-displays.sh - remove EVERY virtual display mirage created.
#
# We discover them by name pattern (VIRT*) rather than counting up to VCOUNT, so
# a stray output left by a hard kill is still cleaned up, and a changed VCOUNT
# can't strand one. `hyprctl output remove` is historically flaky, so we re-query
# and retry until none remain (or we give up after a few passes).
set -uo pipefail

command -v hyprctl >/dev/null || { echo "teardown: hyprctl not found"; exit 0; }

list_virt() {
    hyprctl monitors all -j | python3 -c '
import json,sys
for m in json.load(sys.stdin):
    if str(m.get("name","")).startswith("VIRT"):
        print(m["name"])'
}

for pass in 1 2 3 4; do
    remaining="$(list_virt)"
    [ -z "$remaining" ] && break
    for name in $remaining; do
        echo "removing $name"
        hyprctl output remove "$name" >/dev/null 2>&1 || true
    done
    sleep 0.2
done

if [ -n "$(list_virt)" ]; then
    echo "teardown: warning - these VIRT outputs would not remove:" $(list_virt)
else
    echo "done."
fi
