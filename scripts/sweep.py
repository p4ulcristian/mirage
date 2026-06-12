#!/usr/bin/env python3
"""sweep.py - move real windows onto the virtual displays (the arc) and back,
remembering the exact screen each window sat on across restarts.

    sweep.py            place every real window on the arc. Windows we've seen
                        before go back to their remembered screen; newcomers
                        fill empty screens in a stable order. Snapshots each
                        window's real workspace so restore can put it back.
    sweep.py restore    record where each window currently sits on the arc
                        (-> persistent layout map, so next launch is identical),
                        then move every window back to the workspace it came from.

Talks to Hyprland over `hyprctl` IPC: `hyprctl -j` for JSON state, `hyprctl eval`
for the Lua window-move API (Hyprland 0.55 broke the plain
`dispatch movetoworkspacesilent`, so the Lua `hl.dispatch(...)` form is the
working way to move a window silently).

Robustness: each window is moved on its own, then we re-query and verify it
actually landed on a VIRT output, retrying the stragglers once. A single window
that refuses to move can no longer silently drop every move after it - the bug
that left most apps behind when the moves were one big eval batch.

State files:
  STATE  (ephemeral, /tmp)  this session's real-workspace snapshot, for restore.
  LAYOUT (persistent)       key(window address) -> screen slot, so the layout
                            you leave behind returns next launch.
"""
import json
import os
import subprocess
import sys

STATE = "/tmp/mirage-sweep.json"
LAYOUT = os.path.join(
    os.environ.get("XDG_STATE_HOME", os.path.expanduser("~/.local/state")),
    "mirage", "layout.json")


def hypr(*args):
    """Query Hyprland; returns parsed JSON."""
    return json.loads(subprocess.check_output(["hyprctl", "-j", *args]))


def move_window(address, workspace):
    """Move one window to a workspace, silently (no active-workspace jump)."""
    lua = ("hl.dispatch(hl.dsp.window.move({ window='address:%s', "
           "workspace='%d', silent=true }))" % (address, workspace))
    subprocess.run(["hyprctl", "eval", lua],
                   stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)


def virt_outputs():
    """VIRT monitors in name order (VIRT1, VIRT2, ...)."""
    mons = sorted(hypr("monitors"), key=lambda m: m["name"])
    return [m for m in mons if m["name"].startswith("VIRT")]


def monitor_of(address):
    """Current monitor id of a window, or None if it's gone."""
    for c in hypr("clients"):
        if c["address"] == address:
            return c.get("monitor")
    return None


def apply_moves(targets):
    """targets: list of (address, workspace_id, virt_id). Move each, then verify
    on the arc and retry the stragglers once. Returns the count that landed."""
    for addr, ws, _vid in targets:
        move_window(addr, ws)
    # verify against ground truth, not the move command's (unreliable) exit code
    virt_ids = {m["id"] for m in virt_outputs()}
    landed, stragglers = [], []
    for addr, ws, vid in targets:
        (landed if monitor_of(addr) in virt_ids else stragglers).append((addr, ws, vid))
    for addr, ws, _vid in stragglers:          # one retry
        move_window(addr, ws)
    if stragglers:
        virt_ids = {m["id"] for m in virt_outputs()}
        for addr, ws, vid in stragglers:
            if monitor_of(addr) in virt_ids:
                landed.append((addr, ws, vid))
    return len(landed), [t for t in targets if t not in landed]


def do_sweep():
    virt = virt_outputs()
    if not virt:
        sys.exit("sweep: no VIRT outputs - run setup_displays.py first")
    slots = [m["activeWorkspace"]["id"] for m in virt]   # slot i -> workspace id
    virt_ids = {m["id"] for m in virt}
    nslots = len(slots)

    # remembered address -> slot from the last quit (clamped to slot count).
    try:
        remembered = {k: v for k, v in json.load(open(LAYOUT)).items()
                      if isinstance(v, int) and 0 <= v < nslots}
    except (OSError, ValueError):
        remembered = {}

    # candidate windows: real, mapped, non-pinned, on a normal workspace, not
    # mirage's own overlay, and not already parked on a VIRT output. Sorted by
    # address for a stable newcomer fill order.
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
    if not cands:
        print("sweep: nothing to move")
        return

    # assign slots: remembered windows first (skip if their slot is taken), then
    # newcomers into the remaining free slots, then any overflow round-robins.
    taken, assigned = {}, {}

    def claim(addr, slot):
        if taken.get(slot) in (None, addr):
            taken[slot] = addr
            assigned[addr] = slot
            return True
        return False

    for c in cands:
        if c["address"] in remembered:
            claim(c["address"], remembered[c["address"]])
    free = [s for s in range(nslots) if s not in taken]
    fi = 0
    for c in cands:
        a = c["address"]
        if a in assigned:
            continue
        if fi < len(free):
            claim(a, free[fi])
            fi += 1
        else:
            assigned[a] = len(assigned) % nslots   # overflow: stable round-robin

    # snapshot real workspaces (so restore can put each window back), then move.
    os.makedirs(os.path.dirname(LAYOUT), exist_ok=True)
    snap = [{"address": c["address"], "ws": c["workspace"]["id"],
             "floating": bool(c.get("floating"))} for c in cands]
    json.dump(snap, open(STATE, "w"))

    targets = [(c["address"], slots[assigned[c["address"]]],
                virt[assigned[c["address"]]]["id"]) for c in cands]
    moved, missed = apply_moves(targets)
    print("sweep: moved %d/%d window(s) onto the arc (snapshot: %s)"
          % (moved, len(cands), STATE))
    for addr, _ws, _vid in missed:
        print("  WARNING: %s would not move onto the arc" % addr, file=sys.stderr)


def do_restore():
    # learn from where things sit NOW: address -> slot for every window on a VIRT
    # output, so next launch reproduces this layout.
    virt = virt_outputs()
    slot_of_id = {m["id"]: i for i, m in enumerate(virt)}
    if virt:
        layout = {}
        for c in hypr("clients"):
            if not c.get("mapped") or c.get("class", "") == "mirage":
                continue
            slot = slot_of_id.get(c.get("monitor"))
            if slot is not None:
                layout[c["address"]] = slot
        if layout:
            os.makedirs(os.path.dirname(LAYOUT), exist_ok=True)
            json.dump(layout, open(LAYOUT, "w"))

    if not os.path.exists(STATE):
        print("sweep: no snapshot at %s, nothing to restore" % STATE)
        return
    snap = json.load(open(STATE))
    for w in snap:                       # move each back to its real workspace
        move_window(w["address"], w["ws"])
    os.remove(STATE)
    print("sweep: restored %d window move(s)" % len(snap))


if __name__ == "__main__":
    cmd = sys.argv[1] if len(sys.argv) > 1 else "sweep"
    if cmd == "sweep":
        do_sweep()
    elif cmd == "restore":
        do_restore()
    else:
        sys.exit("usage: sweep.py [sweep|restore]")
