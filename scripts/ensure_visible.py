#!/usr/bin/env python3
"""ensure_visible.py - guarantee mirage is actually shown on the laptop (windowed mode).

mirage set_fullscreens on the laptop's own output, which SHOULD be visible. But if the
compositor ever maps it on a headless VIRT, or the laptop is left showing a different
workspace, the user sees nothing ("I start it but it's on the wrong workspace"). This
runs in the background right after launch and, idempotently:

  - mirage on the laptop but its workspace isn't the one being shown -> switch the view to it
  - mirage stranded on a headless VIRT                               -> pull it to the laptop

Every observation is appended to /tmp/mirage-placement.log with timestamps, so a launch
that still comes up wrong is diagnosable from the log instead of guesswork.

    ensure_visible.py <laptop-output-name>     # e.g. eDP-1
"""
import json
import subprocess
import sys
import time

LAPTOP = sys.argv[1] if len(sys.argv) > 1 else "eDP-1"
LOG = "/tmp/mirage-placement.log"


def hctl(*a):
    return json.loads(subprocess.check_output(["hyprctl", "-j", *a]))


def dispatch(*a):
    subprocess.run(["hyprctl", "dispatch", *a],
                   stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)


def log(msg):
    t = time.strftime("%H:%M:%S")
    with open(LOG, "a") as f:
        f.write(f"{t} {msg}\n")


def mirage_client():
    for c in hctl("clients"):
        if c.get("class") == "mirage":
            return c
    return None


def main():
    log(f"--- ensure_visible start (laptop={LAPTOP}) ---")
    deadline = time.monotonic() + 8.0
    while time.monotonic() < deadline:
        try:
            c = mirage_client()
            mons_l = hctl("monitors")
        except Exception as e:                       # hyprctl hiccup; keep trying
            log(f"hyprctl error: {e}")
            time.sleep(0.3)
            continue
        if not c:
            time.sleep(0.2)
            continue

        by_id = {m["id"]: m for m in mons_l}
        by_name = {m["name"]: m for m in mons_l}
        lap = by_name.get(LAPTOP)
        if not lap:
            log(f"  laptop output {LAPTOP} not present?! monitors={list(by_name)}")
            time.sleep(0.3)
            continue
        target_ws = lap["activeWorkspace"]["id"]     # the workspace the laptop is showing
        mon = by_id.get(c["monitor"])
        monname = mon["name"] if mon else "?"
        ws = c["workspace"]["id"]
        addr = c.get("address", "")
        log(f"mirage on monitor={monname} ws={ws}; {LAPTOP} shows ws={target_ws}")

        if monname == LAPTOP and ws == target_ws:
            log("  mirage visible on the laptop - OK")
            return

        # Not visible: move mirage onto whatever workspace the laptop is currently showing.
        # Move BY ADDRESS (works on fullscreen windows where `movewindow mon:` did not), and
        # drop fullscreen first if it's blocking the move, then restore it.
        log(f"  relocating mirage ({addr}) -> {LAPTOP} ws{target_ws}")
        dispatch("focuswindow", f"address:{addr}")
        dispatch("setfloating", f"address:{addr}", "0")
        dispatch("movetoworkspacesilent", f"{target_ws},address:{addr}")
        dispatch("focusmonitor", LAPTOP)
        dispatch("workspace", str(target_ws))
        dispatch("fullscreen", "0")                  # ensure it's fullscreen on the laptop
        time.sleep(0.4)
    log("--- ensure_visible end: gave up (mirage never confirmed visible) ---")


if __name__ == "__main__":
    main()
