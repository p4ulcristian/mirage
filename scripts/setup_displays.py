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
import math
import shutil
import subprocess
import sys
import time

ORIGIN_X, ORIGIN_Y, REFRESH = 8000, 0, 60

# Each panel is ONLY its 2D grid cell (w, h, x, y; x/y relative to the origin).
# The cells form a gap-free rectangle - that's the entire source of truth. The 3D
# arc placement is projected from the cell below, so there are no separate arc /
# lift / col numbers to keep in sync.
PANELS = [
    dict(name="VIRT1", w=4080, h=1748, x=0,    y=0,    role="21:9 banner"),
    dict(name="VIRT2", w=1080, h=2160, x=0,    y=1748, role="left portrait"),
    dict(name="VIRT3", w=1920, h=1080, x=1080, y=1748, role="centre top 16:9"),
    dict(name="VIRT4", w=1920, h=1080, x=1080, y=2828, role="centre bottom 16:9"),
    dict(name="VIRT5", w=1080, h=2160, x=3000, y=1748, role="right portrait"),
]
ACTIVE, DISTANCE, SLAB = "theater", 2.0, 0.05

# Projection of the 2D masonry onto the cylinder. WALL_ARC_DEG is the total
# angular width of the grid; EYE_Y is the masonry y (px) that sits at eye level
# (lift 0). The grid is gapless and the projection is uniform (square pixels, and
# each VIRT output's resolution == its cell, so capture aspect == cell aspect), so
# the arc tiling is gapless too - by construction, not by hand-tuning.
WALL_ARC_DEG = 86.0
EYE_Y = 3368            # centre of the bottom 16:9 row -> the mains sit at eye level


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


def project(p, x0, y0, Wt, k):
    """Project one cell onto the cylinder -> (yaw_deg, arc_deg, lift_m), all
    free-placed. k = metres per masonry pixel on the wall."""
    cx = (p["x"] - x0) + p["w"] / 2.0
    yaw = math.degrees((Wt / 2.0 - cx) * k / DISTANCE)       # +yaw = viewer's left
    arc = math.degrees(p["w"] * k / DISTANCE)
    lift = (EYE_Y - (p["y"] - y0) - p["h"] / 2.0) * k        # masonry y down -> lift up
    return yaw, arc, lift


def emit_layout():
    """Project the 2D masonry onto the cylinder and print the mirage layout (TOML).
    Gapless in 2D -> gapless on the arc, derived entirely from PANELS."""
    x0 = min(p["x"] for p in PANELS)
    y0 = min(p["y"] for p in PANELS)
    Wt = max(p["x"] + p["w"] for p in PANELS) - x0
    k = math.radians(WALL_ARC_DEG) * DISTANCE / Wt           # metres per masonry px
    out = [f'active = "{ACTIVE}"', '', '[[layout]]', f'name       = "{ACTIVE}"',
           f'distance   = {DISTANCE}', 'geometry   = "cylinder"',
           f'slab_depth = {SLAB}', 'spacing    = 0.0', 'screens = [']
    for i, p in enumerate(PANELS, 1):
        yaw, arc, lift = project(p, x0, y0, Wt, k)
        out.append("  { n = %d, yaw = %.2f, arc = %.1f, lift = %.3f },  # %s"
                   % (i, yaw, arc, lift, p["role"]))
    out.append("]")
    print("\n".join(out))


if __name__ == "__main__":
    if "--layout" in sys.argv:
        emit_layout()
    else:
        create()
