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

# Creating/removing the VIRT output and re-moding DP-1 orphans waybar's surface
# (the process survives but stops drawing). Note whether it was up so we can give
# it a clean restart once the desktop layout is back.
WAYBAR_UP=0; pgrep -x waybar >/dev/null && WAYBAR_UP=1

restore() {
    echo "[glasses] restoring desktop..."
    pkill -f facecam-bridge >/dev/null 2>&1 || true   # stop the webcam tracker
    python3 "$HERE/scripts/sweep.py" restore >/dev/null 2>&1 || true
    hyprctl eval "hl.config({ render = { direct_scanout = 0 } })" >/dev/null 2>&1 || true
    bash "$HERE/scripts/teardown-displays.sh" >/dev/null 2>&1 || true
    if [ "$WAYBAR_UP" = 1 ]; then           # re-attach waybar to the restored layout
        # waybar is a systemd user service (Restart=always); restart it through
        # systemd so we don't fight its supervisor or spawn a second instance.
        systemctl --user restart waybar.service 2>/dev/null || true
    fi
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
hyprctl eval "hl.dispatch(hl.dsp.cursor.move({ x=300, y=300 }))" >/dev/null

echo "[glasses] virtual screens + head tracking..."
python3 "$HERE/scripts/setup_displays.py" >/dev/null 2>&1 || true
bash "$HERE/scripts/bridge.sh" >/dev/null 2>&1 || true
# webcam head-position bridge for 6DoF lean/slide parallax (optional; mirage runs
# fine in 3DoF if the camera is busy or this fails). Log: /tmp/facecam.log
bash "$HERE/scripts/facecam.sh" >/dev/null 2>&1 || true

# Now that VIRT1 exists, restart waybar so it attaches a bar there too. mirage
# captures the whole VIRT1 output (waybar is a layer surface on it), so the bar
# rides along onto the wall - that's how you get your status bar inside mirage.
if [ "$WAYBAR_UP" = 1 ]; then
    systemctl --user restart waybar.service 2>/dev/null || true
    sleep 0.5
fi

echo "[glasses] sweeping your windows onto the virtual screens..."
python3 "$HERE/scripts/sweep.py" sweep 2>&1 | sed 's/^/  [sweep] /' || true

echo "[glasses] launching mirage fullscreen on $GLASSES (trackpad capture is always on; Super+Shift+Q quits)"
./mirage >/tmp/mirage.log 2>&1
# mirage exited -> trap restore runs
