#!/usr/bin/env python3
"""setup_displays.py - create + position mirage's virtual displays (replaces
setup-displays.sh).

Desk layout: a small 21:9 ultrawide centred on top, three 16:9 monitors in a row
below it. The bottom row tiles gaplessly; the top ultrawide is narrower than the
row, so the two upper corners are empty desktop (a dead zone for window-dragging
there only - the price of a small, non-full-width top monitor).

    2D desktop (what Hyprland sees, what governs dragging):
      +------+------------------------+------+
      |      |   VIRT1  21:9          |      |
      |      |   2560x1080            |      |   <- centred top, side gaps
      +------+------------------------+------+
      | VIRT2 16:9 | VIRT3 16:9 | VIRT4 16:9 |
      | 1920x1080  | 1920x1080  | 1920x1080  |
      +------------+------------+------------+
        1920       +    1920    +    1920     = 5760 wide

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
    dict(name="VIRT1", w=2560, h=1080, x=1600, y=0,    role="21:9 ultrawide (top, centred)"),
    dict(name="VIRT2", w=1920, h=1080, x=0,    y=1080, role="bottom-left 16:9"),
    dict(name="VIRT3", w=1920, h=1080, x=1920, y=1080, role="bottom-centre 16:9"),
    dict(name="VIRT4", w=1920, h=1080, x=3840, y=1080, role="bottom-right 16:9"),
]
ACTIVE, DISTANCE, SLAB = "desk", 2.0, 0.05

# Projection of the 2D masonry onto the cylinder. WALL_ARC_DEG is the total
# angular width of the grid; EYE_Y is the masonry y (px) that sits at eye level
# (lift 0). The grid is gapless and the projection is uniform (square pixels, and
# each VIRT output's resolution == its cell, so capture aspect == cell aspect), so
# the arc tiling is gapless too - by construction, not by hand-tuning.
WALL_ARC_DEG = 121.4    # spans the 5760-wide bottom row; each 16:9 ~40 deg, the top 21:9 ~54 deg
EYE_Y = 1620            # centre of the bottom 16:9 row (1080 + 540) -> the mains sit at eye level

# Bezel gap: shrink each panel's arc uniformly about its centre, so the screens
# sit apart with a visible gap between them on the arc WITHOUT moving (yaw/lift
# unchanged) and WITHOUT touching the 2D desktop tiling - the cells stay a gapless
# rectangle, so window-dragging is still seamless; only the on-glasses panels are
# inset. Height follows arc (render derives it), so the inset is uniform on all
# sides and aspect is preserved. 0 = gapless; 0.08 ~= an 8%-of-a-screen gap.
GAP_FRAC = 0.08


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
    arc = math.degrees(p["w"] * k / DISTANCE) * (1.0 - GAP_FRAC)  # inset for the gap
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
