#!/usr/bin/env python3
"""sweep.py - move real windows onto the virtual displays (the arc) and back,
remembering which screen each app sat on across restarts.

    sweep.py            place every real window on the arc. Windows whose app
                        we've seen before go back to their remembered screen;
                        newcomers fill empty screens in a stable order.
                        Snapshots each window's real workspace so restore can
                        put it back.
    sweep.py restore    learn the current arc layout (-> persistent layout map,
                        so next launch reproduces it), then move every window
                        back to the workspace it came from.

Talks to Hyprland over `hyprctl` IPC: `hyprctl -j` for JSON state, `hyprctl eval`
for the Lua window-move API (Hyprland 0.55 broke the plain
`dispatch movetoworkspacesilent`, so the Lua `hl.dispatch(...)` form is the
working way to move a window silently).

Identity / layout memory: a window's `address` is a runtime pointer that
changes every launch, so it is useless as a persistent key. We key the layout
map on the app's class instead, storing the ordered list of screen slots that
class's windows occupied: `{class: [slot, slot, ...]}`. On the next sweep the
Nth window of a class (in a stable address order) returns to that class's Nth
remembered slot. Two windows of the same app are therefore interchangeable -
unavoidable, since their titles are not stable either.

Robustness: moves are async, so we fire them, let the compositor settle, then
verify against ground truth (one client query per pass, not one per window) and
retry the stragglers a few times with backoff. A window that refuses to move
can no longer silently drop every move after it.

State files:
  STATE  (ephemeral, /tmp)  this session's real-workspace snapshot, for restore.
  LAYOUT (persistent)       class -> [slot, ...], so the layout you leave behind
                            returns next launch.
"""
import json
import os
import subprocess
import sys
import time

STATE = "/tmp/mirage-sweep.json"
LAYOUT = os.path.join(
    os.environ.get("XDG_STATE_HOME", os.path.expanduser("~/.local/state")),
    "mirage", "layout.json")

SETTLE = 0.12   # seconds to let Hyprland process a batch of moves
RETRIES = 3     # extra verify+retry rounds after the first move
WS_BASE = 0     # must match setup_displays.py: VIRTn -> reserved workspace WS_BASE+n (wall on ws 1..N)


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


def virt_ws(m):
    """Reserved workspace id for a VIRT monitor (VIRTn -> WS_BASE+n). We target
    this fixed, monitor-bound workspace instead of the monitor's volatile
    activeWorkspace, so a move always lands on the intended screen."""
    return WS_BASE + int(m["name"][len("VIRT"):])


def wkey(c):
    """Stable per-app key for layout memory. initialClass survives runtime
    class changes; class is the fallback."""
    return c.get("initialClass") or c.get("class") or ""


def apply_moves(targets):
    """targets: list of (address, workspace_id, virt_id). Move each, let the
    compositor settle, then verify on the arc and retry stragglers a few times
    with backoff. Returns (landed_count, missed_targets). Windows that vanished
    mid-sweep are dropped silently (neither landed nor missed)."""
    pending, landed = list(targets), []
    for attempt in range(RETRIES + 1):
        for addr, ws, _vid in pending:
            move_window(addr, ws)
        time.sleep(SETTLE * (attempt + 1))          # back off a little each round
        # verify against ground truth with ONE client query, not one per window.
        # A window counts as landed only if it's on ITS target monitor - not just
        # "any VIRT" - so a botched mapping can't masquerade as success.
        cmap = {c["address"]: c for c in hypr("clients")}
        still = []
        for t in pending:
            addr, _ws, vid = t
            c = cmap.get(addr)
            if c is None:
                continue                            # window closed; nothing to do
            (landed if c.get("monitor") == vid else still).append(t)
        pending = still
        if not pending:
            break
    return len(landed), pending


def do_sweep():
    virt = virt_outputs()
    if not virt:
        sys.exit("sweep: no VIRT outputs - run setup_displays.py first")
    slots = [virt_ws(m) for m in virt]                   # slot i -> reserved workspace id
    virt_ids = {m["id"] for m in virt}
    nslots = len(slots)

    # remembered layout from the last quit: {class: [slot, ...]}, clamped to the
    # current slot count. Legacy address-keyed maps (values are ints) are ignored
    # -> every window is treated as a newcomer, which is the correct migration.
    remembered = {}
    try:
        raw = json.load(open(LAYOUT))
    except (OSError, ValueError):
        raw = {}
    if isinstance(raw, dict):
        for k, v in raw.items():
            if isinstance(v, list):
                remembered[k] = [s for s in v
                                 if isinstance(s, int) and 0 <= s < nslots]

    # candidate windows: real, mapped, non-pinned, on a normal workspace, not
    # mirage's own overlay, and not already parked on a VIRT output. Sorted by
    # address for a stable per-class instance order.
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

    # assign slots. taken: slot -> addr. assigned: addr -> slot.
    taken, assigned = {}, {}

    def claim(addr, slot):
        if slot is None or not (0 <= slot < nslots):
            return False
        if taken.get(slot) in (None, addr):
            taken[slot], assigned[addr] = addr, slot
            return True
        return False

    # pass 1: remembered slots, per class, in instance order (skip if taken).
    by_class = {}
    for c in cands:
        by_class.setdefault(wkey(c), []).append(c)
    for cls, group in by_class.items():
        cls_slots = remembered.get(cls, [])
        for i, c in enumerate(group):
            if i < len(cls_slots):
                claim(c["address"], cls_slots[i])

    # pass 2: newcomers / bumped windows fill the remaining free slots in order;
    # genuine overflow (more windows than screens) round-robins across slots.
    free = [s for s in range(nslots) if s not in taken]
    fi = ov = 0
    for c in cands:
        a = c["address"]
        if a in assigned:
            continue
        if fi < len(free):
            claim(a, free[fi])
            fi += 1
        else:
            assigned[a] = ov % nslots
            ov += 1

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
    # learn from where things sit NOW: for each class, the slots its windows
    # occupy (ordered by address), so next launch reproduces this layout.
    virt = virt_outputs()
    slot_of_id = {m["id"]: i for i, m in enumerate(virt)}
    if virt:
        by_cls = {}
        for c in hypr("clients"):
            if not c.get("mapped") or c.get("class", "") == "mirage":
                continue
            slot = slot_of_id.get(c.get("monitor"))
            if slot is not None:
                by_cls.setdefault(wkey(c), []).append((c["address"], slot))
        layout = {cls: [slot for _addr, slot in sorted(pairs)]
                  for cls, pairs in by_cls.items()}
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
