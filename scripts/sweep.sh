#!/usr/bin/env bash
# sweep.sh - move all real windows onto the virtual displays (the arc) and back.
#
#   sweep.sh            snapshot every real window's workspace, then distribute
#                       them round-robin across the VIRT outputs' workspaces.
#   sweep.sh restore    move each window back to the workspace it came from,
#                       using the snapshot, then drop the snapshot.
#
# The snapshot lives at $STATE so quit can restore even from a fresh shell.
# mirage's own overlay is a layer-surface, not a toplevel window, so it is never
# touched by these window moves - it stays on the glasses as the container.
set -euo pipefail
STATE="${MIRAGE_SWEEP_STATE:-/tmp/mirage-sweep.json}"

command -v hyprctl >/dev/null || { echo "sweep: hyprctl not found"; exit 1; }

do_sweep() {
    # targets = the active workspace id on each VIRT output, in name order
    # (VIRT4,VIRT5,VIRT6 -> the left/centre/right screens mirage captures).
    local plan
    plan="$(hyprctl -j monitors | python3 -c '
import json,sys
mons=json.load(sys.stdin)
targets=[m["activeWorkspace"]["id"] for m in sorted(mons,key=lambda m:m["name"])
         if m["name"].startswith("VIRT")]
print(json.dumps(targets))')"
    [ "$plan" = "[]" ] && { echo "sweep: no VIRT outputs - run setup-displays.sh first"; exit 1; }

    # snapshot every real, mapped, non-pinned toplevel not already on a VIRT
    # output, then emit the move dispatches (round-robin across targets).
    local batch
    batch="$(hyprctl -j clients | TARGETS="$plan" STATE="$STATE" python3 -c '
import json,sys,os
targets=json.loads(os.environ["TARGETS"])
clients=json.load(sys.stdin)
snap=[]; moves=[]; i=0
for c in clients:
    if not c.get("mapped") or c.get("pinned"): continue
    ws=c.get("workspace",{}).get("id",0)
    if ws<=0: continue                      # skip special/scratchpad
    mon=str(c.get("monitor",""))
    cls=c.get("class","")
    if cls=="mirage": continue              # never sweep the container itself
    addr=c["address"]
    snap.append({"address":addr,"ws":ws,"floating":bool(c.get("floating"))})
    moves.append("dispatch movetoworkspacesilent %d,address:%s" % (targets[i%len(targets)], addr))
    i+=1
open(os.environ["STATE"],"w").write(json.dumps(snap))
print(" ; ".join(moves))')"

    local n; n="$(STATE="$STATE" python3 -c 'import json,os;print(len(json.load(open(os.environ["STATE"]))))' )"
    if [ -z "$batch" ]; then echo "sweep: nothing to move"; return; fi
    hyprctl --batch "$batch" >/dev/null
    echo "sweep: moved $n window(s) onto the arc (snapshot: $STATE)"
}

do_restore() {
    if [ ! -f "$STATE" ]; then echo "sweep: no snapshot at $STATE, nothing to restore"; return; fi
    local batch
    batch="$(STATE="$STATE" python3 -c '
import json,os
snap=json.load(open(os.environ["STATE"]))
out=[]
for w in snap:
    out.append("dispatch movetoworkspacesilent %d,address:%s" % (w["ws"], w["address"]))
    if w.get("floating"):
        out.append("dispatch setfloating address:%s" % w["address"])
print(" ; ".join(out))')"
    [ -n "$batch" ] && hyprctl --batch "$batch" >/dev/null || true
    rm -f "$STATE"
    echo "sweep: restored $(echo "$batch" | grep -o movetoworkspacesilent | wc -l) window move(s)"
}

case "${1:-sweep}" in
    sweep)   do_sweep ;;
    restore) do_restore ;;
    *) echo "usage: sweep.sh [sweep|restore]"; exit 2 ;;
esac
