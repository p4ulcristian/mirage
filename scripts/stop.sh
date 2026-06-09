#!/usr/bin/env bash
# stop.sh - quit the mirage compositor and clean up after it.
#   stop.sh           quit just the renderer (leave displays up)
#   stop.sh --restore quit, save the on-arc layout, move windows back to a real
#                     monitor, then remove the virtual displays
#   stop.sh --all     quit, CLOSE the demo terminals, remove virtual displays
#
# Cleanup is idempotent and serialised with a lock, so it's safe to call from
# both the Super+Shift+Q/X keybinds and glasses.sh's restore trap - whichever
# runs first does the work, the other is a no-op. That's what makes teardown
# reliable no matter how mirage exits (clean quit, crash, killactive, reload).
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
PIDFILE=/tmp/mirage.pid
LOCK=/tmp/mirage-cleanup.lock

stop_mirage() {  # harmless if mirage already exited
    local pid=""
    [ -f "$PIDFILE" ] && pid="$(cat "$PIDFILE" 2>/dev/null)"
    # Release input grab first so the cursor is free before the process exits
    pkill -USR2 -x mirage 2>/dev/null || true
    sleep 0.05
    if [ -n "$pid" ] && kill -0 "$pid" 2>/dev/null; then
        kill -INT "$pid" 2>/dev/null
        for _ in 1 2 3 4 5 6 7 8 9 10; do
            kill -0 "$pid" 2>/dev/null || break
            sleep 0.1
        done
        kill -KILL "$pid" 2>/dev/null || true
    fi
    pkill -INT -x mirage 2>/dev/null || true   # catch any stray instance
    rm -f "$PIDFILE"
    # stop the RayNeo bridge too (runs under sudo until a udev replug)
    sudo pkill -INT -f rayneo-bridge 2>/dev/null || pkill -INT -f rayneo-bridge 2>/dev/null || true
}

# Save the arc layout, then move every window that lives on a VIRT output back to
# a real monitor so nothing is stranded when the virtual displays go away.
restore_windows() {
    # Preferred: per-window restore from the sweep snapshot (records the current
    # arc layout for next launch, then puts each window back on its original
    # workspace). Falls back to the workspace-level sweep when there's no snapshot.
    if [ -f /tmp/mirage-sweep.json ]; then
        bash "$HERE/scripts/sweep.sh" restore || true
        return
    fi

    # target monitor to move stranded windows back onto: auto-detect (prefer eDP).
    local target
    target="$(hyprctl monitors -j | python3 -c '
import json,sys
mons=[m["name"] for m in json.load(sys.stdin)]
real=[n for n in mons if not n.startswith("VIRT")]
edp=[n for n in real if n.startswith("eDP")]
print((edp or real or [""])[0])')"
    [ -z "$target" ] && { echo "restore: no real monitor found"; return; }

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

# The full post-quit cleanup, run under a non-blocking lock so concurrent callers
# don't interleave. Every step is independently idempotent, so even if it does
# run twice the second pass is a harmless no-op.
cleanup() {  # $1 = restore mode: "restore" | "all" | ""
    case "${1:-}" in
        restore)
            restore_windows
            bash "$HERE/scripts/teardown-displays.sh" || true
            echo "windows restored, virtual displays removed"
            ;;
        all)
            pkill -f 'kitty --title V[123]' 2>/dev/null || true
            bash "$HERE/scripts/teardown-displays.sh" || true
            echo "full teardown complete"
            ;;
    esac
}

mode=""
case "${1:-}" in
    --restore) mode="restore" ;;
    --all)     mode="all" ;;
esac

stop_mirage
echo "mirage + bridge stopped"

# Serialise + dedupe cleanup across the keybind and the supervisor trap.
if command -v flock >/dev/null; then
    exec 9>"$LOCK"
    if flock -n 9; then
        cleanup "$mode"
    fi   # lock held by the other caller -> it's doing the cleanup; skip
else
    cleanup "$mode"
fi
