#!/usr/bin/env python3
"""setup_displays.py - create + position mirage's virtual displays (replaces
setup-displays.sh).

Theater masonry layout: a 21:9 banner on top, two 16:9 stacked in the middle,
flanked by two portraits. The five panels tile a CLEAN RECTANGLE, so dragging a
window between any of them - portraits included - is seamless, with no dead zones
(the bug the old centred 1920/1080 stack had at the landscape<->portrait seam).

    2D desktop (what Hyprland sees, what governs dragging):
      +------------------------------------------------+
      |              VIRT1  21:9  4080x1748            |
      +-----------+----------------------+-------------+
      | VIRT2     |   VIRT3  1920x1080   |   VIRT5     |
      | portrait  +----------------------+   portrait  |
      | 1080x2160 |   VIRT4  1920x1080   |  1080x2160  |
      +-----------+----------------------+-------------+
        1080      +        1920          +    1080      = 4080 wide
        every column in the lower row is 2160 tall -> edges line up

This file is the single source of truth for the arrangement: `create()` builds
the 2D monitors here, and `--layout` emits the matching mirage 3D arc layout
(TOML), so the desktop tiling and the on-glasses wall are derived from the same
PANELS list and can't drift apart.

    setup_displays.py            create + position the displays
    setup_displays.py --layout   print the matching mirage arc layout (TOML)

Talks to Hyprland over `hyprctl`: outputs are created at runtime (they don't
exist at config-load), so this is necessarily imperative, not a declarative
`monitor=` block. Window management / monitor control belongs to the compositor,
not the mirage binary - mirage stays a generic Wayland client.
"""
import json
import shutil
import subprocess
import sys
import time

ORIGIN_X, ORIGIN_Y, REFRESH = 8000, 0, 60

# Each panel: its 2D desktop rect (w, h, x, y - x/y relative to the origin) plus
# its spot on the 3D arc (d3). 2D rects below form a gap-free rectangle.
PANELS = [
    dict(name="VIRT1", w=4080, h=1748, x=0,    y=0,    role="21:9 banner",
         d3=dict(yaw=0.0, lift=1.85, arc=88)),                 # free-placed, spans the wall
    dict(name="VIRT2", w=1080, h=2160, x=0,    y=1748, role="left portrait",
         d3=dict(col=0, arc=23, lift=0.41)),
    dict(name="VIRT3", w=1920, h=1080, x=1080, y=1748, role="centre top 16:9",
         d3=dict(col=1, arc=40, lift=0.41)),
    dict(name="VIRT4", w=1920, h=1080, x=1080, y=2828, role="centre bottom 16:9",
         d3=dict(col=1, arc=40, lift=0.41)),
    dict(name="VIRT5", w=1080, h=2160, x=3000, y=1748, role="right portrait",
         d3=dict(col=2, arc=23, lift=0.41)),
]
ACTIVE, DISTANCE, CENTER_COL, SPACING, SLAB = "theater", 2.0, 1, 0.0, 0.05


def hypr(*args):
    return json.loads(subprocess.check_output(["hyprctl", "-j", *args]))


def ev(lua):
    subprocess.run(["hyprctl", "eval", lua],
                   stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)


def create():
    if not shutil.which("hyprctl"):
        sys.exit("setup_displays: hyprctl not found (need Hyprland)")
    want = {p["name"] for p in PANELS}
    have = {m["name"] for m in hypr("monitors", "all") if m["name"].startswith("VIRT")}
    for name in sorted(have - want):                  # drop stale VIRT outputs
        print("removing", name)
        subprocess.run(["hyprctl", "output", "remove", name], stdout=subprocess.DEVNULL)
    for p in PANELS:
        if p["name"] not in have:
            print("creating", p["name"])
            subprocess.run(["hyprctl", "output", "create", "headless", p["name"]],
                           stdout=subprocess.DEVNULL)
    time.sleep(0.3)
    for p in PANELS:                                  # position via Lua (0.55 IPC)
        x, y = ORIGIN_X + p["x"], ORIGIN_Y + p["y"]
        ev("hl.monitor({ output='%s', mode='%dx%d@%d', position='%dx%d', scale=1 })"
           % (p["name"], p["w"], p["h"], REFRESH, x, y))
        print("  %-6s %dx%d at %d,%d  (%s)" % (p["name"], p["w"], p["h"], x, y, p["role"]))


def emit_layout():
    """Print the mirage arc layout (TOML) derived from the same PANELS."""
    out = [f'active = "{ACTIVE}"', '', '[[layout]]', f'name       = "{ACTIVE}"',
           f'distance   = {DISTANCE}', 'geometry   = "cylinder"',
           f'slab_depth = {SLAB}', f'spacing    = {SPACING}',
           f'center_col = {CENTER_COL}', 'screens = [']
    for i, p in enumerate(PANELS, 1):
        d = p["d3"]
        parts = [f"n = {i}"]
        if "col" in d:
            parts.append(f"col = {d['col']}")
        if "yaw" in d:
            parts.append(f"yaw = {d['yaw']}")
        parts += [f"arc = {d['arc']}", f"lift = {d['lift']}"]
        out.append("  { %s },  # %s" % (", ".join(parts), p["role"]))
    out.append("]")
    print("\n".join(out))


if __name__ == "__main__":
    if "--layout" in sys.argv:
        emit_layout()
    else:
        create()
