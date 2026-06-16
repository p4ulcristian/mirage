#!/usr/bin/env python3
"""setup_displays.py - create + position mirage's virtual displays.

Reads layouts.conf (the single source of truth) and creates the Hyprland VIRT
outputs for one layout from its screens' masonry cells: w,h (resolution) at x,y
(2D desktop position). mirage reads the SAME cells and projects the 3D arc, so the
desktop tiling and the on-glasses wall derive from one set of numbers.

    setup_displays.py            create the active layout's displays
    setup_displays.py desk720    create a named layout's displays

The 2D desktop for the default 'desk' layout (what governs window dragging):

      +------+------------------------+------+
      |      |   VIRT1  21:9          |      |   <- centred top, side gaps
      +------+------------------------+------+
      | VIRT2 16:9 | VIRT3 16:9 | VIRT4 16:9 |
      +------------+------------+------------+

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
        panels.append(dict(name=f"VIRT{int(s['n'])}", w=int(s["w"]), h=int(s["h"]),
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
        print("  %-6s %dx%d at %d,%d  (%s)" % (p["name"], p["w"], p["h"], x, y, p["role"]))


if __name__ == "__main__":
    arg = next((a for a in sys.argv[1:] if not a.startswith("-")), None)
    name, panels = load_layout(arg)
    print(f"setup_displays: layout '{name}' ({len(panels)} screens)")
    create(panels)
