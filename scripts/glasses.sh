#!/usr/bin/env bash
# glasses.sh - run mirage on the AR glasses with DIRECT SCANOUT (~120Hz), while
# mirage stays an ordinary app (no compositor/lease/kernel changes).
#
# Direct scanout = Hyprland page-flips mirage's buffer straight to the panel with
# ZERO compositing — but only if the glasses output (DP-1) has NOTHING on it: no
# bar, no notifications, no software cursor, and mirage hands over an opaque XR24
# fullscreen buffer. HyprPanel forces a bar+notification layer onto every monitor
# and can't be told to skip one (and it dbus-reactivates when killed), so for the
# session we run a watchdog that keeps it off. Everything is restored on exit.
#
#   bash scripts/glasses.sh                                       # real scene + tracking
#   bash scripts/glasses.sh --no-terrain --no-sky --sharpen 0     # lightest (locks 120)
#
# Quit mirage (Super+Shift+Q, or `pkill -x mirage`) and the bar/cursor/scanout all
# come back automatically. The laptop bar is gone ONLY during the session.
set -uo pipefail
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$HERE"
GLASSES="${GLASSES:-DP-1}"

# Input-device detection for Super+G mouse/keyboard grab (same as run.sh).
detect_input() {  # $1 = Name regex -> prints /dev/input/eventN
    awk -v re="$1" '
        /^N: Name=/ { match($0, re); hit = RSTART > 0 }
        hit && /^H: Handlers=/ {
            n = split($0, f, /[ \t]/)
            for (i = 1; i <= n; i++) if (f[i] ~ /^event[0-9]+$/) { print "/dev/input/" f[i]; exit }
        }
    ' /proc/bus/input/devices
}
: "${MIRAGE_TRACKPAD:=$(detect_input "multi-touch|[Tt]ouchpad|[Tt]rackpad")}"
: "${MIRAGE_KEYBOARD:=$(detect_input "[Kk]eyboard")}"
export MIRAGE_TRACKPAD MIRAGE_KEYBOARD
echo "[glasses] input: trackpad=${MIRAGE_TRACKPAD:-?} keyboard=${MIRAGE_KEYBOARD:-?}"

# gjs hyprpanel pid(s) — filter on comm so the pattern can never match this script
panel_pids() { for p in $(pgrep -f hyprpanel 2>/dev/null); do
    [ "$(cat /proc/$p/comm 2>/dev/null)" = gjs ] && echo "$p"; done; }
HAD_PANEL=0; [ -n "$(panel_pids)" ] && HAD_PANEL=1
SUPPRESS_PID=""

restore() {
    echo "[glasses] restoring desktop..."
    [ -n "$SUPPRESS_PID" ] && kill "$SUPPRESS_PID" 2>/dev/null   # stop the watchdog FIRST
    bash "$HERE/scripts/sweep.sh" restore >/dev/null 2>&1 || true   # windows back to real monitors
    hyprctl keyword render:direct_scanout 0 >/dev/null 2>&1 || true
    bash "$HERE/scripts/teardown-displays.sh" >/dev/null 2>&1 || true
    if [ "$HAD_PANEL" = 1 ] && [ -z "$(panel_pids)" ]; then
        setsid hyprpanel >/dev/null 2>&1 </dev/null &
    fi
}
trap restore EXIT INT TERM

echo "[glasses] quitting HyprPanel so its bar/notifications leave $GLASSES (scanout needs a clean output)..."
# Use HyprPanel's OWN clean quit, not SIGKILL gjs. A hard kill leaves it flickering
# back (top-layer bar/notifications re-land on DP-1 and disqualify direct scanout for
# ~1.5s at a time - measured as fps oscillating 58<->77). `hyprpanel -q` makes it stay
# down and clears DP-1 to scanout-eligible. Backup watchdog re-quits if it ever returns.
hyprpanel -q 2>/dev/null || true
( while :; do pgrep -x gjs >/dev/null 2>&1 && hyprpanel -q 2>/dev/null; sleep 2; done ) &
SUPPRESS_PID=$!
sleep 1

echo "[glasses] enabling direct scanout + fullscreen/opaque rule for mirage..."
hyprctl keyword render:direct_scanout 1 >/dev/null
# Lock the glasses to a STABLE 60Hz. The panel can do 120, but profiling showed the
# loop free-running at ~90-120 with periodic present hitches (eglSwapInterval(2) is
# NOT honored on Hyprland's direct-scanout path). GPU draw is <1ms, so the limiter
# is purely present pacing - running the panel at 60 makes vsync HARDWARE-lock to a
# rock-solid 60 with ~15ms of slack per frame. Set GLASSES_HZ=120 to opt back out.
GLASSES_HZ="${GLASSES_HZ:-60}"
echo "[glasses] locking $GLASSES to ${GLASSES_HZ}Hz for a stable vsync..."
hyprctl keyword monitor "$GLASSES,1920x1080@${GLASSES_HZ},auto,1" >/dev/null || true
# fullscreen is requested by mirage itself (--fullscreen, below); these rules just
# keep the surface scanout-eligible (opaque, no blur/rounding) and pin it to $GLASSES.
for r in "monitor $GLASSES" "opaque" "noblur" "norounding"; do
    hyprctl keyword windowrulev2 "$r,class:^(mirage)\$" >/dev/null
done
# Laptop mirror (opt-in: MIRROR=1): mirage opens a second --preview toplevel showing
# the flat view of the same screens on the laptop. OFF by default - the extra surface
# interfered with the glasses coming up cleanly; needs revisiting before re-enabling.
LAPTOP="${LAPTOP:-eDP-1}"
if [ "${MIRROR:-0}" = 1 ]; then
    for r in "monitor $LAPTOP" "float" "noblur"; do
        hyprctl keyword windowrulev2 "$r,class:^(mirage-preview)\$" >/dev/null
    done
fi

echo "[glasses] parking cursor on the laptop (Asahi has no HW cursor plane)..."
hyprctl dispatch movecursor 300 300 >/dev/null

echo "[glasses] virtual screens + head tracking..."
VR="${VR:-60}" bash "$HERE/scripts/setup-displays.sh" >/dev/null 2>&1 || true
bash "$HERE/scripts/bridge.sh" >/dev/null 2>&1 || true

echo "[glasses] sweeping your windows onto the virtual screens..."
bash "$HERE/scripts/sweep.sh" sweep 2>&1 | sed 's/^/  [sweep] /' || true

echo "[glasses] launching mirage fullscreen on $GLASSES (AUTOGRAB=${AUTOGRAB:-0}; Super+G toggles grab)"

# Auto-grab only: once the first frame lands, if AUTOGRAB=1 (real session; 0 =
# hands-off testing) send the SIGUSR2 that Super+G uses to capture mouse/keyboard
# onto the arc. Fullscreen is handled by mirage itself (--fullscreen below).
if [ "${AUTOGRAB:-0}" = 1 ]; then
  ( for _ in $(seq 1 80); do
        grep -q -e 'fps' -e 'capture\[' /tmp/mirage.log 2>/dev/null && break
        sleep 0.1
    done
    pkill -USR2 -x mirage ) &
fi

PREVIEW=; [ "${MIRROR:-0}" = 1 ] && PREVIEW="--preview"
./mirage --windowed 1920x1080 --fullscreen --3d $PREVIEW "$@" >/tmp/mirage.log 2>&1
# mirage exited -> trap restore runs
