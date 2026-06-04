#!/usr/bin/env bash
# sweep.sh - move all real windows onto the virtual displays (the arc) and back,
# remembering the EXACT screen each window sat on across restarts.
#
#   sweep.sh            place every real window on the arc. Windows we've seen
#                       before go back to their remembered screen; newcomers
#                       fill the empty screens in a stable order. Snapshots each
#                       window's real workspace so restore can put it back.
#   sweep.sh restore    first record where each window currently sits on the arc
#                       (-> persistent layout map, so next launch is identical),
#                       then move every window back to the workspace it came from.
#
# Two state files:
#   $STATE  (ephemeral, /tmp)  - this session's "real workspace" snapshot, for
#                                putting windows back exactly where they were.
#   $LAYOUT (persistent)       - key(class+initialTitle) -> screen slot, so the
#                                arrangement you leave behind returns on next run.
#
# A window's identity is its initialClass + initialTitle (stable across launches,
# unlike the Hyprland address). Several windows sharing one identity (e.g. three
# bare `kitty` terminals) are only told apart by a stable ordering, not pinned
# individually - fine for distinct apps.
#
# mirage's own overlay is a layer-surface, not a toplevel window, so it is never
# touched by these moves - it stays on the glasses as the container.
set -euo pipefail
STATE="${MIRAGE_SWEEP_STATE:-/tmp/mirage-sweep.json}"
LAYOUT="${MIRAGE_LAYOUT_STATE:-${XDG_STATE_HOME:-$HOME/.local/state}/mirage/layout.json}"

command -v hyprctl >/dev/null || { echo "sweep: hyprctl not found"; exit 1; }
mkdir -p "$(dirname "$LAYOUT")"

do_sweep() {
    local batch n
    batch="$(STATE="$STATE" LAYOUT="$LAYOUT" python3 - <<'PY'
import json, os, subprocess

def hypr(*args):
    return json.loads(subprocess.check_output(["hyprctl", "-j", *args]))

# slots = VIRT outputs in name order; slot i -> its active workspace id.
# Keep their monitor IDs too: a client's "monitor" field is an integer ID, NOT
# the name, so VIRT membership must be tested by ID.
mons = sorted(hypr("monitors"), key=lambda m: m["name"])
virt = [m for m in mons if m["name"].startswith("VIRT")]
slots = [m["activeWorkspace"]["id"] for m in virt]
virt_ids = {m["id"] for m in virt}
if not slots:
    raise SystemExit("sweep: no VIRT outputs - run setup-displays.sh first")
nslots = len(slots)

# remembered address -> slot from the last quit (clamped to current slot count).
try:
    remembered = {k: v for k, v in json.load(open(os.environ["LAYOUT"])).items()
                  if isinstance(v, int) and 0 <= v < nslots}
except (OSError, ValueError):
    remembered = {}

# candidate windows: real, mapped, non-pinned, on a normal workspace, not mirage,
# not already parked on a VIRT output. The Hyprland address is stable across a
# mirage restart (the window isn't recreated, only moved), so it is our layout
# key; sort by it for a stable newcomer fill order.
cands = []
for c in sorted(hypr("clients"), key=lambda c: c["address"]):
    if not c.get("mapped") or c.get("pinned"):
        continue
    if c.get("workspace", {}).get("id", 0) <= 0:
        continue
    if c.get("class", "") == "mirage":
        continue
    if c.get("monitor") in virt_ids:
        continue
    cands.append(c)

# assign slots: remembered windows first (skip if their slot is taken), then
# newcomers into whatever slots remain, then any overflow round-robins.
taken = {}            # slot -> address, first claimant wins
assigned = {}         # address -> slot
def claim(addr, slot):
    if taken.get(slot) in (None, addr):
        taken[slot] = addr; assigned[addr] = slot; return True
    return False

for c in cands:
    a = c["address"]
    if a in remembered:
        claim(a, remembered[a])
free = [s for s in range(nslots) if s not in taken]
fi = 0
for c in cands:
    a = c["address"]
    if a in assigned:
        continue
    if fi < len(free):
        claim(a, free[fi]); fi += 1
    else:
        assigned[a] = len(assigned) % nslots   # overflow: stable round-robin

# snapshot real workspaces (for restore) and emit the moves onto the arc.
snap, moves = [], []
for c in cands:
    a = c["address"]
    snap.append({"address": a, "ws": c["workspace"]["id"],
                 "floating": bool(c.get("floating"))})
    moves.append("dispatch movetoworkspacesilent %d,address:%s" % (slots[assigned[a]], a))

json.dump(snap, open(os.environ["STATE"], "w"))
print(len(snap))
print(" ; ".join(moves))
PY
)" || { echo "sweep: planning failed"; return 1; }
    n="$(printf '%s\n' "$batch" | sed -n '1p')"
    batch="$(printf '%s\n' "$batch" | sed -n '2p')"
    [ "${n:-0}" -gt 0 ] || { echo "sweep: nothing to move"; return; }
    [ -n "$batch" ] && hyprctl --batch "$batch" >/dev/null || true
    echo "sweep: moved $n window(s) onto the arc (snapshot: $STATE)"
}

do_restore() {
    # First, learn from where things sit NOW: record address -> slot for every
    # window currently on a VIRT output, so next launch reproduces this layout.
    # A client's "monitor" is an integer ID, so map VIRT IDs -> slot index.
    STATE="$STATE" LAYOUT="$LAYOUT" python3 - <<'PY' || true
import json, os, subprocess

def hypr(*args):
    return json.loads(subprocess.check_output(["hyprctl", "-j", *args]))

mons = sorted(hypr("monitors"), key=lambda m: m["name"])
virt = [m for m in mons if m["name"].startswith("VIRT")]
slot_of_id = {m["id"]: i for i, m in enumerate(virt)}
if not virt:
    raise SystemExit  # nothing on the arc to learn from

layout = {}
for c in hypr("clients"):
    if not c.get("mapped") or c.get("class", "") == "mirage":
        continue
    slot = slot_of_id.get(c.get("monitor"))
    if slot is None:
        continue
    layout[c["address"]] = slot

if layout:
    json.dump(layout, open(os.environ["LAYOUT"], "w"))
PY

    # Then move every window back to the real workspace it came from.
    if [ ! -f "$STATE" ]; then echo "sweep: no snapshot at $STATE, nothing to restore"; return; fi
    local batch
    batch="$(STATE="$STATE" python3 - <<'PY'
import json, os
snap = json.load(open(os.environ["STATE"]))
out = []
for w in snap:
    out.append("dispatch movetoworkspacesilent %d,address:%s" % (w["ws"], w["address"]))
    if w.get("floating"):
        out.append("dispatch setfloating address:%s" % w["address"])
print(" ; ".join(out))
PY
)"
    [ -n "$batch" ] && hyprctl --batch "$batch" >/dev/null || true
    rm -f "$STATE"
    echo "sweep: restored $(printf '%s' "$batch" | grep -o movetoworkspacesilent | wc -l) window move(s)"
}

case "${1:-sweep}" in
    sweep)   do_sweep ;;
    restore) do_restore ;;
    *) echo "usage: sweep.sh [sweep|restore]"; exit 2 ;;
esac
