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
    out = subprocess.check_output(["hyprctl", "-j", *args])
    return json.loads(out)

# slots = VIRT outputs in name order; slot i -> its active workspace id.
mons = sorted(hypr("monitors"), key=lambda m: m["name"])
slots = [m["activeWorkspace"]["id"] for m in mons if m["name"].startswith("VIRT")]
if not slots:
    raise SystemExit("sweep: no VIRT outputs - run setup-displays.sh first")
nslots = len(slots)

# remembered key -> slot from the last quit (clamped to current slot count).
try:
    remembered = {k: v for k, v in json.load(open(os.environ["LAYOUT"])).items()
                  if isinstance(v, int) and 0 <= v < nslots}
except (OSError, ValueError):
    remembered = {}

# candidate windows: real, mapped, non-pinned, on a normal workspace, not mirage,
# not already parked on a VIRT output. Sort by address for a stable order so the
# per-identity occurrence index is reproducible.
cands = []
for c in sorted(hypr("clients"), key=lambda c: c["address"]):
    if not c.get("mapped") or c.get("pinned"):
        continue
    if c.get("workspace", {}).get("id", 0) <= 0:
        continue
    if c.get("class", "") == "mirage":
        continue
    if str(c.get("monitor", "")).startswith("VIRT"):
        continue
    cands.append(c)

# stable identity key: initialClass + initialTitle + per-identity occurrence.
seen = {}
def key_of(c):
    base = "%s\x1f%s" % (c.get("initialClass", c.get("class", "")),
                         c.get("initialTitle", ""))
    i = seen.get(base, 0); seen[base] = i + 1
    return "%s\x1f%d" % (base, i)

items = [(key_of(c), c) for c in cands]

# assign slots: remembered windows first (skip if their slot is taken), then
# newcomers into whatever slots remain, then any overflow round-robins.
taken = {}            # slot -> key, first claimant wins
assigned = {}         # key -> slot
def claim(key, slot):
    if taken.get(slot) in (None, key):
        taken[slot] = key; assigned[key] = slot; return True
    return False

for key, _ in items:
    if key in remembered:
        claim(key, remembered[key])
free = [s for s in range(nslots) if s not in taken]
fi = 0
for key, _ in items:
    if key in assigned:
        continue
    if fi < len(free):
        claim(key, free[fi]); fi += 1
    else:
        assigned[key] = (len(assigned)) % nslots   # overflow: stable round-robin

# snapshot real workspaces (for restore) and emit the moves onto the arc.
snap, moves = [], []
for key, c in items:
    snap.append({"address": c["address"], "key": key,
                 "ws": c["workspace"]["id"], "floating": bool(c.get("floating"))})
    moves.append("dispatch movetoworkspacesilent %d,address:%s"
                 % (slots[assigned[key]], c["address"]))

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
    # First, learn from where things sit NOW: record key -> slot for every window
    # currently on a VIRT output, so next launch reproduces this exact layout.
    STATE="$STATE" LAYOUT="$LAYOUT" python3 - <<'PY' || true
import json, os, subprocess

def hypr(*args):
    return json.loads(subprocess.check_output(["hyprctl", "-j", *args]))

mons = sorted(hypr("monitors"), key=lambda m: m["name"])
virt = [m["name"] for m in mons if m["name"].startswith("VIRT")]
slot_of = {name: i for i, name in enumerate(virt)}
if not virt:
    raise SystemExit  # nothing on the arc to learn from

seen, layout = {}, {}
for c in sorted(hypr("clients"), key=lambda c: c["address"]):
    if not c.get("mapped") or c.get("class", "") == "mirage":
        continue
    mon = str(c.get("monitor", ""))
    if mon not in slot_of:
        continue
    base = "%s\x1f%s" % (c.get("initialClass", c.get("class", "")),
                         c.get("initialTitle", ""))
    i = seen.get(base, 0); seen[base] = i + 1
    layout["%s\x1f%d" % (base, i)] = slot_of[mon]

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
