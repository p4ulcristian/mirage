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

echo "[glasses] keeping the desktop shell off $GLASSES (watchdog)..."
( while :; do for p in $(panel_pids); do kill "$p" 2>/dev/null; done; sleep 1.5; done ) &
SUPPRESS_PID=$!
sleep 1

echo "[glasses] enabling direct scanout + fullscreen/opaque rule for mirage..."
hyprctl keyword render:direct_scanout 1 >/dev/null
# fullscreen is requested by mirage itself (--fullscreen, below); these rules just
# keep the surface scanout-eligible (opaque, no blur/rounding) and pin it to $GLASSES.
for r in "monitor $GLASSES" "opaque" "noblur" "norounding"; do
    hyprctl keyword windowrulev2 "$r,class:^(mirage)\$" >/dev/null
done
# Laptop mirror (MIRROR=1, default): mirage opens a second --preview toplevel showing
# the flat view of the same screens. It's a SEPARATE surface on the laptop, so it
# never touches the glasses output - DP-1 stays scanned out at full rate. Pin it to
# the laptop, floating, so it doesn't tile over your desktop. MIRROR=0 to skip it.
LAPTOP="${LAPTOP:-eDP-1}"
if [ "${MIRROR:-1}" = 1 ]; then
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

PREVIEW=; [ "${MIRROR:-1}" = 1 ] && PREVIEW="--preview"
./mirage --windowed 1920x1080 --fullscreen --3d $PREVIEW "$@" >/tmp/mirage.log 2>&1
# mirage exited -> trap restore runs
