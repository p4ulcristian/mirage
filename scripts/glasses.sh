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

# Detect the glasses output (RayNeo "SmartGlasses" / "VITURE" Beast) by description,
# and the laptop's own physical output (first non-VIRT). HAS_GLASSES picks the path:
#   1 -> direct-scanout + head tracking onto the glasses (GLASSES output)
#   0 -> same scene windowed on the laptop (LAPTOP output), trackpad swipes to look
# This is the ONE difference between "glasses on" and "glasses off"; everything below
# (VIRT displays, sweep, mirage, teardown) is identical for both.
mon_json() { hyprctl monitors all -j; }
pick_mon() { python3 -c 'import sys,json
ms=json.load(sys.stdin)
import os
mode=os.environ["PICK"]
def desc(m): return (m.get("description") or "")+" "+m.get("name","")
if mode=="glasses":
    print(next((m["name"] for m in ms if any(k in desc(m) for k in ("SmartGlasses","VITURE","RayNeo"))),""))
else:
    print(next((m["name"] for m in ms if not m["name"].startswith("VIRT")),""))'; }
GLASSES="$(mon_json | PICK=glasses pick_mon 2>/dev/null)"
LAPTOP="$(mon_json | PICK=laptop  pick_mon 2>/dev/null)"
if [ -n "$GLASSES" ]; then HAS_GLASSES=1; TARGET="$GLASSES"; else HAS_GLASSES=0; TARGET="$LAPTOP"; fi
echo "[glasses] mode: $([ "$HAS_GLASSES" = 1 ] && echo "glasses ($GLASSES)" || echo "windowed ($LAPTOP)")"

# Boot ownership handshake. On a relaunch the OLD instance tears down its VIRT outputs
# when it exits, which races the NEW instance creating them -> mirage boots with 0
# screens and dies ("booting up often doesn't work"). Fix: serialise create-vs-teardown
# on one lock and stamp ownership - a launch claims the displays, and only the current
# owner tears them down, so a newer launch supersedes the old one's teardown.
DISP_LOCK=/tmp/mirage-displays.lock
DISP_OWNER=/tmp/mirage-displays.owner
MY_SESS=$$

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
    # Take the displays lock and only tear down if we're STILL the owner: if a newer
    # launch has superseded us it owns the VIRT outputs now, and ripping them out would
    # kill its boot (the flaky-relaunch bug). Serialised against setup so they can't interleave.
    (
        flock 9
        if [ "$(cat "$DISP_OWNER" 2>/dev/null)" != "$MY_SESS" ]; then
            echo "[glasses] superseded by a newer launch - leaving its displays up"
            exit 0
        fi
        python3 "$HERE/scripts/sweep.py" restore >/dev/null 2>&1 || true
        hyprctl eval "hl.config({ render = { direct_scanout = 0 } })" >/dev/null 2>&1 || true
        bash "$HERE/scripts/teardown-displays.sh" >/dev/null 2>&1 || true
        rm -f "$DISP_OWNER"
    ) 9>"$DISP_LOCK"
    if [ "$WAYBAR_UP" = 1 ]; then           # re-attach waybar to the restored layout
        # waybar is a systemd user service (Restart=always); restart it through
        # systemd so we don't fight its supervisor or spawn a second instance.
        systemctl --user restart waybar.service 2>/dev/null || true
    fi
}
trap restore EXIT INT TERM

# Hyprland 0.55 uses the Lua (non-legacy) config parser - `hyprctl keyword ...` is
# rejected there, so every setting goes through the Lua API via `hyprctl eval`.
if [ "$HAS_GLASSES" = 1 ]; then
    echo "[glasses] enabling direct scanout + fullscreen/opaque rule for mirage..."
    hyprctl eval "hl.config({ render = { direct_scanout = 1 } })" >/dev/null
    # Run the glasses at their native 120Hz. On Hyprland 0.55 (aquamarine 0.12) present
    # pacing is rock-solid - measured 120.0fps, 0.00 hitches/sec, worst-frame ~11ms - so
    # the old ~95Hz present wall is gone and GPU draw is <1ms.
    echo "[glasses] locking $GLASSES to 120Hz for a stable vsync..."
    # The Beast only exposes 1920x1080@60 over DP on Asahi (its 120Hz mode FREEZES the
    # apple-drm/DCP driver - never set it). Keep DP-1 at the safe 60Hz mode.
    hyprctl eval "hl.monitor({ output='$GLASSES', mode='1920x1080@60', position='auto', scale=1 })" >/dev/null || true
    echo "[glasses] parking cursor on the laptop (Asahi has no HW cursor plane)..."
    hyprctl eval "hl.dispatch(hl.dsp.cursor.move({ x=300, y=300 }))" >/dev/null
    PLACE="monitor = '$GLASSES'"            # the glasses output is visible; pin mirage there
else
    # No glasses: direct-scanout needs a dedicated panel, so leave it OFF and let mirage
    # composite as a normal fullscreen window on the laptop. Don't re-mode anything.
    echo "[glasses] windowed mode on $LAPTOP — no scanout, no re-mode"
    hyprctl eval "hl.config({ render = { direct_scanout = 0 } })" >/dev/null
    # mirage itself set_fullscreens on $LAPTOP's wl_output (classify_outputs picks the real
    # non-VIRT panel in windowed mode), which is the deterministic placement. This rule just
    # reinforces it - no workspace juggling, no post-launch dispatch racing the first map.
    PLACE="monitor = '$LAPTOP'"
fi
# fullscreen is requested by mirage itself; these rules keep the surface opaque/sharp and
# place it where it's visible ($PLACE) so it never lands on a headless VIRT.
hyprctl eval "
hl.window_rule({ match = { class = 'mirage' }, $PLACE })
hl.window_rule({ match = { class = 'mirage' }, opacity = '1.0' })
hl.window_rule({ match = { class = 'mirage' }, no_blur = true })
hl.window_rule({ match = { class = 'mirage' }, rounding = 0 })
" >/dev/null

echo "[glasses] virtual screens$([ "$HAS_GLASSES" = 1 ] && echo ' + head tracking')..."
# Claim the displays and create the VIRTs under the lock so a dying instance's teardown
# can't race us to zero. setup_displays self-retries flaky `hyprctl output create` and
# exits non-zero if any VIRT is still missing; retry the whole call before launching.
(
    flock 9
    echo "$MY_SESS" > "$DISP_OWNER"          # we own the displays now (supersede older instances)
    for attempt in 1 2 3 4; do
        python3 "$HERE/scripts/setup_displays.py" 2>&1 | sed 's/^/  [displays] /'
        [ "${PIPESTATUS[0]}" = 0 ] && break
        echo "[glasses] virtual screens incomplete (attempt $attempt) - retrying..."
        sleep 0.4
    done
) 9>"$DISP_LOCK"
# Head-tracking bridge only makes sense with glasses on; windowed mode looks around
# with 3/4-finger trackpad swipes (mirage falls back to identity pose without it).
[ "$HAS_GLASSES" = 1 ] && bash "$HERE/scripts/viture-bridge.sh" >/dev/null 2>&1 || true

# Now that VIRT1 exists, restart waybar so it attaches a bar there too. mirage
# captures the whole VIRT1 output (waybar is a layer surface on it), so the bar
# rides along onto the wall - that's how you get your status bar inside mirage.
if [ "$WAYBAR_UP" = 1 ]; then
    systemctl --user restart waybar.service 2>/dev/null || true
    sleep 0.5
fi

echo "[glasses] sweeping your windows onto the virtual screens..."
python3 "$HERE/scripts/sweep.py" sweep 2>&1 | sed 's/^/  [sweep] /' || true

echo "[glasses] launching mirage fullscreen on $TARGET (trackpad capture is always on; Super+Shift+Q quits)"
# Optional test knobs (empty = built-in defaults). Set MIRAGE_PREDICT_MS=0 to disable
# forward-prediction, or MIRAGE_WORLDVIO_GAIN=0 to disable world-cam parallax, when isolating.
PREDICT_MS="${MIRAGE_PREDICT_MS:-}"
WORLDVIO_GAIN="${MIRAGE_WORLDVIO_GAIN:-}"
# 6DoF-lite optical-flow backend: OpenCV LK path (worker thread); override with MIRAGE_WORLDVIO=proj.
export MIRAGE_WORLDVIO="${MIRAGE_WORLDVIO:-cv}"
[ -n "$PREDICT_MS" ]    && export MIRAGE_PREDICT_MS="$PREDICT_MS"
[ -n "$WORLDVIO_GAIN" ] && export MIRAGE_WORLDVIO_GAIN="$WORLDVIO_GAIN"
# DIAG (off): export MIRAGE_VIEW_TRACE=1 -> per-frame view trace to /tmp/mirage-view-trace.log
if [ "$HAS_GLASSES" != 1 ]; then
    # You may be focused on a low workspace (e.g. ws1) that a VIRT just claimed, which
    # drags focus onto that headless VIRT - mirage then maps THERE, not the laptop, and
    # you get "dropped to an empty workspace, can't see mirage". Force focus to the laptop
    # right before launch so mirage maps on it. ensure_visible.py is the backup + the log.
    hyprctl dispatch focusmonitor "$LAPTOP" >/dev/null 2>&1 || true
    ( python3 "$HERE/scripts/ensure_visible.py" "$LAPTOP" ) &
fi
./mirage >/tmp/mirage.log 2>&1
# mirage exited -> trap restore runs
