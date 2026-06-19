#!/usr/bin/env python3
"""setup_displays.py - create + position mirage's virtual displays.

Reads layouts.conf (the single source of truth) and creates the Hyprland VIRT
outputs for one layout from its screens' masonry cells: w,h (resolution) at x,y
(2D desktop position). mirage reads the SAME cells and projects the 3D arc, so the
desktop tiling and the on-glasses wall derive from one set of numbers.

    setup_displays.py            create the active layout's displays
    setup_displays.py desk720    create a named layout's displays

The 2D desktop for the default 'desk' layout (what governs window dragging) is one
row: four 16:9 wings, a 21:9 ultrawide dead-ahead in the centre, four more wings:

  +-----+-----+-----+-----+-----------+-----+-----+-----+-----+
  |VIRT1|VIRT2|VIRT3|VIRT4| VIRT5 21:9|VIRT6|VIRT7|VIRT8|VIRT9|
  +-----+-----+-----+-----+-----------+-----+-----+-----+-----+

Talks to Hyprland over `hyprctl`: outputs are created at runtime (they don't
exist at config-load), so this is necessarily imperative. Layout path overridable
via $MIRAGE_LAYOUTS; otherwise <repo>/layouts.conf.
"""
import json
import os
import shutil
import subprocess
import sys
import time
import tomllib

ORIGIN_X, ORIGIN_Y, REFRESH = 8000, 0, 60

# Each VIRTn gets a dedicated, persistent workspace WS_BASE+n. We deliberately put
# the wall on the LOW workspaces (WS_BASE=0 -> VIRT1..10 = ws 1..10) so Super+1..0
# drive the mirage screens directly. The physical display(s) are parked on PHYS_WS_BASE
# (90+) so they can't squat on ws 1 and collide with VIRT1. A fresh headless output
# still adopts its reserved ws via the pin-before-create rule below, so it never steals
# someone else's number. sweep.py mirrors WS_BASE; change both together.
WS_BASE = 0
PHYS_WS_BASE = 90       # physical monitors (DP-1, ...) get ws 90, 91, ... (off the wall's 1..10)


def layouts_path():
    env = os.environ.get("MIRAGE_LAYOUTS")
    if env:
        return env
    here = os.path.dirname(os.path.abspath(__file__))
    return os.path.join(os.path.dirname(here), "layouts.conf")


def load_layout(name=None):
    """Return (layout_name, [panels]) where each panel is dict(name,w,h,x,y,role).
    name=None picks the file's `active` layout."""
    path = layouts_path()
    try:
        with open(path, "rb") as f:
            conf = tomllib.load(f)
    except FileNotFoundError:
        sys.exit(f"setup_displays: layouts file not found: {path}")
    except tomllib.TOMLDecodeError as e:
        sys.exit(f"setup_displays: parse error in {path}: {e}")

    layouts = conf.get("layout", [])
    if not layouts:
        sys.exit(f"setup_displays: no [[layout]] tables in {path}")
    want = name or conf.get("active") or layouts[0].get("name")
    lay = next((l for l in layouts if l.get("name") == want), None)
    if lay is None:
        names = ", ".join(l.get("name", "?") for l in layouts)
        sys.exit(f"setup_displays: no layout named '{want}' (have: {names})")

    panels = []
    for s in lay.get("screens", []):
        if not all(k in s for k in ("n", "w", "h", "x", "y")):
            continue   # column/yaw-placed screen without a cell: not ours to create
        panels.append(dict(n=int(s["n"]), name=f"VIRT{int(s['n'])}",
                           w=int(s["w"]), h=int(s["h"]),
                           x=int(s["x"]), y=int(s["y"]), role=s.get("role", "")))
    if not panels:
        sys.exit(f"setup_displays: layout '{want}' has no cell-defined screens (w,h,x,y)")
    return want, panels


def hypr(*args):
    return json.loads(subprocess.check_output(["hyprctl", "-j", *args]))


def ev(lua):
    subprocess.run(["hyprctl", "eval", lua],
                   stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)


def create(panels):
    if not shutil.which("hyprctl"):
        sys.exit("setup_displays: hyprctl not found (need Hyprland)")
    # Park the physical display(s) on PHYS_WS_BASE+ so the wall can own ws 1..N. Without
    # this DP-1 sits on ws 1 and collides with VIRT1's reserved workspace.
    phys = sorted(m["name"] for m in hypr("monitors", "all")
                  if not m["name"].startswith("VIRT"))
    for i, name in enumerate(phys):
        ws = PHYS_WS_BASE + i
        ev("hl.workspace_rule({ workspace='%d', monitor='%s', default=true, persistent=true })"
           % (ws, name))
        ev("hl.dispatch(hl.dsp.workspace.move({ workspace='%d', monitor='%s' }))"
           % (ws, name))
        print("  %-6s -> ws%d (physical)" % (name, ws))
    # Pin each VIRT's reserved workspace BEFORE the output is created, so a fresh
    # headless monitor adopts WS_BASE+n on connect instead of stealing a low
    # workspace. persistent=true keeps it alive even when empty (so the screen
    # always accepts new windows); default=true sends new windows there.
    for p in panels:
        ev("hl.workspace_rule({ workspace='%d', monitor='%s', default=true, persistent=true })"
           % (WS_BASE + p["n"], p["name"]))

    want = {p["name"] for p in panels}
    have = {m["name"] for m in hypr("monitors", "all") if m["name"].startswith("VIRT")}
    for name in sorted(have - want):                  # drop stale VIRT outputs
        print("removing", name)
        subprocess.run(["hyprctl", "output", "remove", name], stdout=subprocess.DEVNULL)
    for p in panels:
        if p["name"] not in have:
            print("creating", p["name"])
            subprocess.run(["hyprctl", "output", "create", "headless", p["name"]],
                           stdout=subprocess.DEVNULL)
    time.sleep(0.3)
    for p in panels:                                  # position + mode via Lua (0.55 IPC)
        x, y = ORIGIN_X + p["x"], ORIGIN_Y + p["y"]
        ev("hl.monitor({ output='%s', mode='%dx%d@%d', position='%dx%d', scale=1 })"
           % (p["name"], p["w"], p["h"], REFRESH, x, y))
        # Idempotent force-bind for the warm-restart case: if the output already
        # existed it never adopted the rule on connect, so pull its reserved
        # workspace onto it now. A no-op on a cold start (already adopted).
        ev("hl.dispatch(hl.dsp.workspace.move({ workspace='%d', monitor='%s' }))"
           % (WS_BASE + p["n"], p["name"]))
        print("  %-6s %dx%d at %d,%d  ws%d  (%s)"
              % (p["name"], p["w"], p["h"], x, y, WS_BASE + p["n"], p["role"]))


if __name__ == "__main__":
    arg = next((a for a in sys.argv[1:] if not a.startswith("-")), None)
    name, panels = load_layout(arg)
    print(f"setup_displays: layout '{name}' ({len(panels)} screens)")
    create(panels)
