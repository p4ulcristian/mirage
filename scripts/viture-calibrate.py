#!/usr/bin/env python3
"""viture-calibrate.py - derive the Beast IMU axis map from gravity, deterministically.

No guessing. Gravity always points along world-up at rest, so holding your head in
two known poses tells us exactly which raw sensor axis is "up" and which is
"forward", and their signs. The sideways axis + its sign are then forced by
right-handedness (a proper rotation, det=+1 - reflections are what made earlier
maps wobble). The result is written to the running bridge live (/tmp/viture-imap
+ SIGHUP), so you can test immediately, then we bake the winner into the source.

Run it while the bridge is streaming (scripts/glasses.sh already started it):

    python3 scripts/viture-calibrate.py

It leads you step by step. Just follow the prompts.
"""
import os, sys, time, subprocess, math

LOG = "/tmp/viture-bridge.log"
IMAP = "/tmp/viture-imap"
AXES = "xyz"

def tail(path, n):
    try:
        out = subprocess.run(["tail", "-n", str(n), path],
                             capture_output=True, text=True, timeout=5).stdout
        return out.splitlines()
    except Exception:
        return []

def parse_acc(line):
    """pull the 3 floats after the RAWacc token; None if not an IMU line."""
    t = line.split()
    if "RAWacc" not in t:
        return None
    i = t.index("RAWacc")
    try:
        return [float(t[i+1]), float(t[i+2]), float(t[i+3])]
    except (IndexError, ValueError):
        return None

def bridge_alive():
    return subprocess.run(["pgrep", "-x", "viture-bridge"],
                          capture_output=True).returncode == 0

def streaming():
    """True if new IMU samples are arriving (n counter advancing)."""
    def last_n():
        for ln in reversed(tail(LOG, 6)):
            t = ln.split()
            if t and t[-2] == "n":
                try: return int(t[-1])
                except ValueError: pass
        return None
    a = last_n(); time.sleep(1.2); b = last_n()
    return a is not None and b is not None and b != a

def sample(seconds=2.5):
    """average raw accel over a hold window (the last `seconds` of log)."""
    time.sleep(0.6)              # let the pose settle
    time.sleep(seconds)
    rows = [parse_acc(l) for l in tail(LOG, int(seconds*5)+4)]
    rows = [r for r in rows if r]
    rows = rows[-int(seconds*4):] or rows
    if not rows:
        return None
    n = len(rows)
    v = [sum(r[k] for r in rows)/n for k in range(3)]
    return v

def normalize(v):
    m = math.sqrt(sum(c*c for c in v)) or 1.0
    return [c/m for c in v]

def dom(v):
    i = max(range(3), key=lambda k: abs(v[k]))
    return i, (1 if v[i] >= 0 else -1)

def countdown(label):
    input(f"\n  >>> {label}\n      ...then press ENTER and HOLD STILL.")
    for s in ("3", "2", "1", "reading"):
        print(f"      {s}", end="\r", flush=True); time.sleep(0.0)
    print("      sampling... hold still   ", end="\r", flush=True)

def perm_parity(a):
    # parity of a permutation of (0,1,2)
    inv = 0
    for i in range(3):
        for j in range(i+1, 3):
            if a[i] > a[j]: inv += 1
    return -1 if inv % 2 else 1

def main():
    print("="*60)
    print(" VITURE Beast - head-tracking axis calibration")
    print("="*60)
    if not bridge_alive():
        print("\n  ✗ bridge isn't running. Start it first:")
        print("      bash scripts/viture-bridge.sh\n")
        return 1
    print("\n  checking the sensor stream...")
    if not streaming():
        print("\n  ✗ bridge is up but NOT streaming (IMU frozen).")
        print("    Restart it:  sudo pkill -x viture-bridge; bash scripts/viture-bridge.sh\n")
        return 1
    print("  ✓ streaming.\n")
    print("  Two poses. Keep your head DEAD STILL during each 3s reading.")

    # --- Pose 1: level -> world-up in sensor frame ---
    countdown("POSE 1 of 2:  sit upright, look STRAIGHT AHEAD at the horizon (head level)")
    g_level = sample()
    if not g_level:
        print("\n  ✗ no data. Is the bridge logging? Check /tmp/viture-bridge.log\n"); return 1
    print(f"      level gravity:  x={g_level[0]:+.2f} y={g_level[1]:+.2f} z={g_level[2]:+.2f}   ✓")

    # --- Pose 2: face down -> forward axis ---
    countdown("POSE 2 of 2:  tip your head DOWN, chin to chest, face the floor")
    g_down = sample()
    if not g_down:
        print("\n  ✗ no data.\n"); return 1
    print(f"      down  gravity:  x={g_down[0]:+.2f} y={g_down[1]:+.2f} z={g_down[2]:+.2f}   ✓")

    # --- derive ---
    # Beast convention (verified on hardware): the raw accel vector aligns with the
    # axis that is physically UP at rest, and at face-down it lands on +forward - so
    # up = level reading, forward = down reading (NO negation; negating flips forward
    # AND, via det=+1, left-right -> a 180-deg-wrong map).
    up = normalize(g_level)
    fwd = normalize(g_down)
    d = sum(fwd[k]*up[k] for k in range(3))          # strip the mounting-tilt
    fwd = normalize([fwd[k] - d*up[k] for k in range(3)])

    up_ax, up_sg = dom(up)
    fwd_ax, fwd_sg = dom(fwd)
    if fwd_ax == up_ax:
        print("\n  ✗ couldn't separate forward from up (pose too shallow?). Re-run and tip"
              "\n    your head further down in pose 2.\n"); return 1
    lat_ax = ({0,1,2} - {up_ax, fwd_ax}).pop()

    # sideways sign forced by right-handedness: det(signed perm) must be +1.
    a = [fwd_ax, lat_ax, up_ax]
    base_det = perm_parity(a) * fwd_sg * up_sg       # with lat_sg = +1
    lat_sg = 1 if base_det > 0 else -1

    def tok(ax, sg): return ("-" if sg < 0 else "") + AXES[ax]
    imap = f"{tok(fwd_ax,fwd_sg)},{tok(lat_ax,lat_sg)},{tok(up_ax,up_sg)}"

    print("\n" + "="*60)
    print(f"  DERIVED MAP:  --imu-map {imap}")
    print(f"    forward = {tok(fwd_ax,fwd_sg)}   side = {tok(lat_ax,lat_sg)}   up = {tok(up_ax,up_sg)}")
    print("="*60)

    # --- apply live ---
    with open(IMAP, "w") as f:
        f.write(imap + "\n")
    r = subprocess.run(["sudo", "-n", "pkill", "-HUP", "-x", "viture-bridge"],
                       capture_output=True)
    if r.returncode == 0:
        print("\n  ✓ applied to the running bridge.")
    else:
        print("\n  (couldn't SIGHUP the bridge automatically; run:")
        print("     sudo pkill -HUP -x viture-bridge )")

    print("\n  NOW TEST:")
    print("   1. Look straight ahead, double-tap Cmd to RECENTER.")
    print("   2. Look left/right, up/down, tilt your head.")
    print("\n  Everything should track 1:1. If ONE axis is reversed, tell Claude")
    print("  which one - that's a single sign flip, not a redo.")
    print(f"\n  (map saved live to {IMAP}; bake into src/viture_bridge.c once confirmed)\n")
    return 0

if __name__ == "__main__":
    try:
        sys.exit(main())
    except KeyboardInterrupt:
        print("\n  cancelled.\n"); sys.exit(130)
