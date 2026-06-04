#!/usr/bin/env bash
# run.sh - launch the mirage AR compositor in the background.
# Passes through any extra args (e.g. --fov 50 --spacing 18).
set -euo pipefail
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
BIN="$HERE/mirage"
PIDFILE=/tmp/mirage.pid
LOG=/tmp/mirage.log

[ -x "$BIN" ] || { echo "build first: make"; exit 1; }

# Auto-detect the trackpad + keyboard event nodes for Super+G capture. The grab
# code's compiled-in defaults (event0/event1) are wrong on this machine - event
# numbers are not stable and depend on probe order. We parse /proc/bus/input/
# devices, matching by device Name, and resolve to the event* handler. Anything
# the caller already exported in MIRAGE_TRACKPAD/MIRAGE_KEYBOARD wins.
detect_input() {  # $1 = Name regex -> prints /dev/input/eventN
    awk -v re="$1" '
        /^N: Name=/ { match($0, re); hit = RSTART > 0 }
        hit && /^H: Handlers=/ {
            n = split($0, f, /[ \t]/)
            for (i = 1; i <= n; i++) if (f[i] ~ /^event[0-9]+$/) { print "/dev/input/" f[i]; exit }
        }
    ' /proc/bus/input/devices
}
# Like detect_input, but prints EVERY matching device's event node, ':'-joined.
# This machine exposes duplicate input devices (a real node + a virtual one)
# with identical names, and the live one isn't stable across boots - so for the
# keyboard we observe them all and let grab.c sort out which carries the keys.
detect_input_all() {  # $1 = Name regex -> prints /dev/input/eventN[:eventM...]
    awk -v re="$1" '
        /^N: Name=/ { match($0, re); hit = RSTART > 0 }
        hit && /^H: Handlers=/ {
            n = split($0, f, /[ \t]/)
            for (i = 1; i <= n; i++) if (f[i] ~ /^event[0-9]+$/) {
                out = out (out ? ":" : "") "/dev/input/" f[i]; break
            }
        }
        END { print out }
    ' /proc/bus/input/devices
}
: "${MIRAGE_TRACKPAD:=$(detect_input "multi-touch|[Tt]ouchpad|[Tt]rackpad")}"
: "${MIRAGE_KEYBOARD:=$(detect_input_all "[Kk]eyboard")}"
export MIRAGE_TRACKPAD MIRAGE_KEYBOARD
echo "input: trackpad=${MIRAGE_TRACKPAD:-?} keyboard=${MIRAGE_KEYBOARD:-?}"

# --3d needs head tracking: make sure the RayNeo bridge is running first.
# (bridge.sh is a no-op if it's already up.)
if [[ " $* " == *" --3d "* ]] || [[ " $* " == *" --3d" ]]; then
    bash "$HERE/scripts/bridge.sh" 2>&1 | sed 's/^/  [bridge] /' || true
fi

# In windowed scene-setup mode, Hyprland would otherwise tile the window to a
# sliver. We render via a normal window because this compositor/Asahi DCP setup
# does NOT publish the glasses as a bindable wl_output (mirage's grab-the-output
# mode can't see it). A window doesn't need a wl_output - Hyprland places it on
# the glasses by name - so we send it there fullscreen at native 120Hz.
if [[ " $* " == *" --windowed "* ]] || [[ " $* " == *" --windowed" ]]; then
    # auto-detect the glasses output (by description), fall back to DP-1
    GLASSES="$(hyprctl monitors all -j | python3 -c '
import json,sys
mons=json.load(sys.stdin)
for m in mons:
    if "SmartGlasses" in (m.get("description") or "") or "RayNeo" in (m.get("description") or ""):
        print(m["name"]); break
else:
    print(next((m["name"] for m in mons if m["name"].startswith("DP-")), "DP-1"))
')"
    hyprctl keyword windowrulev2 "monitor $GLASSES,class:^(mirage)$" >/dev/null
    hyprctl keyword windowrulev2 "fullscreen,class:^(mirage)$"       >/dev/null
fi

# don't stack instances
if [ -f "$PIDFILE" ] && kill -0 "$(cat "$PIDFILE")" 2>/dev/null; then
    echo "mirage already running (pid $(cat "$PIDFILE")). Use scripts/stop.sh first."
    exit 1
fi

# Self-heal: if mirage isn't running but stray VIRT outputs are still around (a
# previous session was hard-killed past its cleanup), remove them so we always
# start from a clean slate. Normal quits already tear them down, so this is a
# no-op in the common case.
if ! pgrep -x mirage >/dev/null 2>&1 \
   && hyprctl monitors all -j 2>/dev/null | grep -q '"name": *"VIRT'; then
    echo "self-heal: clearing stray VIRT outputs from a previous session"
    bash "$HERE/scripts/teardown-displays.sh" 2>&1 | sed 's/^/  [displays] /' || true
fi

# Ensure the virtual displays exist (idempotent: skips any already present).
# stop.sh tears them down on quit, so we recreate them on every launch -
# otherwise the first start after a quit would have nothing to capture.
# VR=60 caps each virtual screen's content refresh at 60 Hz: the glasses still
# render the head-tracked view at native rate, only the captured source is 60.
# With damage-driven capture, idle screens cost ~0 regardless. Override VR=… .
VR="${VR:-60}" bash "$HERE/scripts/setup-displays.sh" 2>&1 | sed 's/^/  [displays] /' || true

# Sweep every real window onto the arc (snapshotted; stop.sh --restore puts them
# back). Non-fatal: if the sweep fails we still launch the renderer.
bash "$HERE/scripts/sweep.sh" sweep 2>&1 | sed 's/^/  [sweep] /' || true

"$BIN" "$@" >"$LOG" 2>&1 &
MPID=$!
echo "$MPID" > "$PIDFILE"
echo "mirage started (pid $MPID), log: $LOG"

# Supervisor watchdog: own the display lifecycle for exactly mirage's lifetime.
# It waits for the mirage pid to vanish - by ANY route: Super+Shift+Q, a crash,
# killactive, a Hyprland reload - then runs the idempotent cleanup (save layout,
# restore windows, remove VIRT, drop the mirror). This is what stops virtual
# displays leaking when mirage exits some way other than the quit keybind.
# Backgrounded + disowned so it outlives this launcher script.
(
    while kill -0 "$MPID" 2>/dev/null; do sleep 0.5; done
    bash "$HERE/scripts/stop.sh" --restore
) >>"$LOG" 2>&1 &
disown
echo "  (supervisor armed; displays will be torn down whenever mirage exits)"

# Mirror the glasses onto the laptop panel so a bystander sees the same AR view.
# stop.sh turns this back off. Override with MIRROR=0 to keep the laptop as its
# own standalone desktop while mirage runs.
if [ "${MIRROR:-1}" = "1" ]; then
    bash "$HERE/scripts/mirror-laptop.sh" on 2>&1 | sed 's/^/  [mirror] /' || true
fi

# Auto-grab the mouse/keyboard onto the arc once the renderer is up. This sends
# the same SIGUSR2 toggle that Super+G uses, after waiting for the first frame
# so the grab subsystem is ready. Override with AUTOGRAB=0 to start ungrabbed.
if [ "${AUTOGRAB:-1}" = "1" ]; then
    ( for _ in $(seq 1 50); do
          grep -q -e 'fps' -e 'capture\[' "$LOG" 2>/dev/null && break
          sleep 0.1
      done
      pkill -USR2 -x mirage ) >/dev/null 2>&1 &
    echo "  (auto-grab armed; press Super+G to release input back to the laptop)"
fi
