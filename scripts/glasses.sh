#!/usr/bin/env bash
# glasses.sh - run mirage on the AR glasses with DIRECT SCANOUT (~120Hz), while
# mirage stays an ordinary app (no compositor/lease/kernel changes).
#
# Direct scanout = Hyprland page-flips mirage's buffer straight to the panel with
# ZERO compositing — but only if the glasses output (DP-1) has NOTHING on it: no
# bar, no notifications, no software cursor, and mirage hands over an opaque XR24
# fullscreen buffer.
#
#   bash scripts/glasses.sh        # real scene + head tracking on the glasses
#
# Quit mirage (Super+Shift+Q, or `pkill -x mirage`) and cursor/scanout come back.
set -uo pipefail
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$HERE"
GLASSES=DP-1

# Kill any mirage still running so the shortcut always launches THIS build (an
# old instance left up would otherwise keep rendering with stale behaviour, and
# its trackpad grab + scanout would fight this one). Wait for it to actually exit.
if pgrep -x mirage >/dev/null; then
    echo "[glasses] stopping the running mirage first..."
    pkill -x mirage
    for _ in $(seq 1 20); do pgrep -x mirage >/dev/null || break; sleep 0.2; done
fi

# mirage auto-detects the trackpad + keyboard event nodes itself (by device name,
# in grab.c) - including preferring the keyd virtual keyboard - so there's nothing
# to pass in here.

restore() {
    echo "[glasses] restoring desktop..."
    bash "$HERE/scripts/sweep.sh" restore >/dev/null 2>&1 || true
    hyprctl eval "hl.config({ render = { direct_scanout = 0 } })" >/dev/null 2>&1 || true
    bash "$HERE/scripts/teardown-displays.sh" >/dev/null 2>&1 || true
}
trap restore EXIT INT TERM

# Hyprland 0.55 uses the Lua (non-legacy) config parser - `hyprctl keyword ...` is
# rejected there, so every setting goes through the Lua API via `hyprctl eval`.
echo "[glasses] enabling direct scanout + fullscreen/opaque rule for mirage..."
hyprctl eval "hl.config({ render = { direct_scanout = 1 } })" >/dev/null
# Run the glasses at their native 120Hz. On Hyprland 0.55 (aquamarine 0.12) present
# pacing is rock-solid - measured 120.0fps, 0.00 hitches/sec, worst-frame ~11ms - so
# the old ~95Hz present wall is gone and GPU draw is <1ms.
echo "[glasses] locking $GLASSES to 120Hz for a stable vsync..."
hyprctl eval "hl.monitor({ output='$GLASSES', mode='1920x1080@120', position='auto', scale=1 })" >/dev/null || true
# fullscreen is requested by mirage itself; these rules just keep the surface
# scanout-eligible (opaque, no blur/rounding) and pin it to $GLASSES.
hyprctl eval "
hl.window_rule({ match = { class = 'mirage' }, monitor = '$GLASSES' })
hl.window_rule({ match = { class = 'mirage' }, opacity = '1.0' })
hl.window_rule({ match = { class = 'mirage' }, no_blur = true })
hl.window_rule({ match = { class = 'mirage' }, rounding = 0 })
" >/dev/null

echo "[glasses] parking cursor on the laptop (Asahi has no HW cursor plane)..."
hyprctl dispatch movecursor 300 300 >/dev/null

echo "[glasses] virtual screens + head tracking..."
bash "$HERE/scripts/setup-displays.sh" >/dev/null 2>&1 || true
bash "$HERE/scripts/bridge.sh" >/dev/null 2>&1 || true

echo "[glasses] sweeping your windows onto the virtual screens..."
bash "$HERE/scripts/sweep.sh" sweep 2>&1 | sed 's/^/  [sweep] /' || true

echo "[glasses] launching mirage fullscreen on $GLASSES (trackpad capture is always on; Super+Shift+Q quits)"
./mirage >/tmp/mirage.log 2>&1
# mirage exited -> trap restore runs
