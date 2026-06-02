#!/usr/bin/env bash
# stop.sh - quit the mirage compositor (graceful SIGINT, then SIGKILL).
#   stop.sh           quit just the renderer
#   stop.sh --restore quit, move windows off the virtual displays back to a
#                     real monitor, then remove the virtual displays
#   stop.sh --all     quit, CLOSE the demo terminals, remove virtual displays
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
PIDFILE=/tmp/mirage.pid
RESTORE_MON="${RESTORE_MON:-}"   # target monitor; auto-detected if empty

stop_mirage() {
    local pid=""
    [ -f "$PIDFILE" ] && pid="$(cat "$PIDFILE" 2>/dev/null)"
    if [ -n "$pid" ] && kill -0 "$pid" 2>/dev/null; then
        kill -INT "$pid" 2>/dev/null
        for _ in 1 2 3 4 5 6 7 8 9 10; do
            kill -0 "$pid" 2>/dev/null || break
            sleep 0.1
        done
        kill -KILL "$pid" 2>/dev/null || true
    fi
    # belt and suspenders: catch any stray instance
    pkill -INT -x mirage 2>/dev/null || true
    rm -f "$PIDFILE"
    # stop the RayNeo bridge too (runs under sudo until a udev replug)
    sudo pkill -INT -f rayneo-bridge 2>/dev/null || pkill -INT -f rayneo-bridge 2>/dev/null || true
    echo "mirage + bridge stopped"
}

# Move every workspace that lives on a VIRT output back to a real monitor, so
# windows aren't stranded when the virtual displays go away.
restore_windows() {
    local target="$RESTORE_MON"
    if [ -z "$target" ]; then
        # first monitor whose name doesn't start with VIRT (prefer eDP-*)
        target="$(hyprctl monitors -j | python3 -c '
import json,sys
mons=[m["name"] for m in json.load(sys.stdin)]
real=[n for n in mons if not n.startswith("VIRT")]
edp=[n for n in real if n.startswith("eDP")]
print((edp or real or [""])[0])')"
    fi
    [ -z "$target" ] && { echo "restore: no real monitor found"; return; }

    # workspace ids currently on VIRT monitors
    local wss
    wss="$(hyprctl workspaces -j | python3 -c '
import json,sys
for w in json.load(sys.stdin):
    if str(w.get("monitor","")).startswith("VIRT") and w["id"] > 0:
        print(w["id"])')"
    for ws in $wss; do
        echo "restore: workspace $ws -> $target"
        hyprctl dispatch moveworkspacetomonitor "$ws" "$target" >/dev/null || true
    done
}

stop_mirage

if [ "${1:-}" = "--restore" ]; then
    restore_windows
    [ -x "$HERE/scripts/teardown-displays.sh" ] && bash "$HERE/scripts/teardown-displays.sh" || true
    echo "windows restored, virtual displays removed"
fi

if [ "${1:-}" = "--all" ]; then
    # close demo terminals titled V1/V2/V3
    pkill -f 'kitty --title V[123]' 2>/dev/null || true
    # remove the virtual displays
    [ -x "$HERE/scripts/teardown-displays.sh" ] && bash "$HERE/scripts/teardown-displays.sh" || true
    echo "full teardown complete"
fi
