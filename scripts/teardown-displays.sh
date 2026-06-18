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

# Drop the persistent workspaces setup_displays.py pinned to the VIRT outputs
# (VIRTn -> ws 9n). With the outputs gone those persistent workspaces migrate onto
# the real monitor and, being persistent, would linger there empty. Redeclare each
# rule non-persistent so Hyprland garbage-collects the now-empty workspace. Aimed
# at the first real (non-VIRT) monitor; all that matters is the rule clears.
REAL="$(hyprctl monitors -j | python3 -c '
import json,sys
ms=[m["name"] for m in json.load(sys.stdin) if not str(m.get("name","")).startswith("VIRT")]
print(ms[0] if ms else "")')"
if [ -n "$REAL" ]; then
    for n in $(seq 1 9); do
        hyprctl eval "hl.workspace_rule({ workspace='9$n', monitor='$REAL', default=false, persistent=false })" >/dev/null 2>&1 || true
    done
fi

if [ -n "$(list_virt)" ]; then
    echo "teardown: warning - these VIRT outputs would not remove:" $(list_virt)
else
    echo "done."
fi
