#!/usr/bin/env bash
# glasses.sh - SUPERVISOR for mirage that survives unplugging/replugging the glasses.
#
# Direct scanout = Hyprland page-flips mirage's buffer straight to the panel with
# ZERO compositing — but only if the glasses output (DP-1) has NOTHING on it: no
# bar, no notifications, no software cursor, and mirage hands over an opaque XR24
# fullscreen buffer.
#
#   bash scripts/glasses.sh        # real scene + head tracking on the glasses
#
# Quit mirage (Super+Shift+Q, or `pkill -x mirage`) and cursor/scanout come back.
#
# ROBUST HOTPLUG: glasses are no longer a launch-time, one-shot decision. This script
# does the shared setup ONCE (VIRT displays + window sweep + waybar), then runs a
# supervisor loop that:
#   - watches whether the glasses output is present (~1Hz, debounced),
#   - GLASSES PLUGGED  -> direct-scanout + head-tracking bridge on the glasses,
#   - GLASSES UNPLUGGED -> the same scene windowed on the laptop (trackpad to look),
#   - relaunches mirage on a mode switch or an unexpected crash,
#   - keeps the head-tracking bridge alive in glasses mode,
# so you can pull the cable mid-session and plug it back in and it just follows you.
# It exits (and restores the desktop) only when YOU quit - see the quit sentinel below.
set -uo pipefail
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$HERE"

# Detect the glasses output (RayNeo "SmartGlasses" / "VITURE" Beast) by description,
# and the laptop's own physical output (first non-VIRT). These are re-evaluated every
# loop tick, so plugging/unplugging is observed live rather than only at launch.
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

# Quit sentinel. The supervisor relaunches mirage whenever it dies - that's the whole
# point (crash recovery + mode switches). So we need an explicit signal for "the USER
# wants out", or the quit key would fight the relaunch. stop.sh (Super+Shift+Q) and the
# panic key touch this file FIRST; the loop sees it and exits cleanly. Cleared on start
# so a stale sentinel from a previous session can't quit us immediately.
SENTINEL=/tmp/mirage-quit
rm -f "$SENTINEL"

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

# Capture focus-follows-mouse ONCE, before we ever freeze it (windowed mode freezes it
# so the cursor over a terminal on a headless VIRT can't steal focus and pull mirage
# onto the wrong screen). Restored when we enter glasses mode and on final restore().
#
# 0 is NEVER a legitimate baseline: it's only ever OUR freeze sentinel. If a prior
# glasses.sh was SIGKILLed while in windowed mode its restore() never ran, leaving
# follow_mouse=0 on the desktop - and this launch would then capture that poisoned 0
# as "the original" and restore it forever, so glasses mode would never refocus the
# window under the injected cursor. Clamp a captured 0 back up to 1.
FOLLOW_MOUSE_ORIG="$(hyprctl getoption input:follow_mouse -j 2>/dev/null | python3 -c 'import sys,json;print(json.load(sys.stdin).get("int",1))' 2>/dev/null || echo 1)"
[ "$FOLLOW_MOUSE_ORIG" = 0 ] && FOLLOW_MOUSE_ORIG=1   # reject a leaked freeze as the baseline

restore() {
    echo "[glasses] restoring desktop..."
    rm -f "$SENTINEL"
    # Kill OUR mirage child only (by PID, not name - a superseding launch may own a newer one).
    [ -n "${MIRAGE_PID:-}" ] && kill "$MIRAGE_PID" 2>/dev/null || true
    # Restore focus-follows-mouse to whatever it was before launch (windowed mode froze it).
    hyprctl eval "hl.config({ input = { follow_mouse = ${FOLLOW_MOUSE_ORIG} } })" >/dev/null 2>&1 || true
    # Stop the head-tracking bridge - it runs under sudo and would otherwise outlive us.
    sudo -n pkill -x viture-bridge 2>/dev/null || pkill -x viture-bridge 2>/dev/null || true
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

# ---------------------------------------------------------------------------
# ONE-TIME setup: the VIRT displays, the window sweep, waybar and the mirage
# window rules are IDENTICAL in both modes, so do them once up front and keep
# them across hot-plug switches. Only the per-mode bits (scanout, cursor, DP
# mode, follow_mouse, the bridge, and which monitor ws90 lives on) flip below.
# ---------------------------------------------------------------------------
# Create/position the VIRTs and (re)pin every workspace under the lock so a dying
# instance's teardown can't race us to zero. setup_displays self-retries flaky
# `hyprctl output create` and exits non-zero if any VIRT is still missing; retry the
# whole call before giving up. Re-run on every hot-plug transition too: when the glasses
# output appears mid-session Hyprland would otherwise hand it ws1 and clobber VIRT1 - this
# re-pins the physical monitors to ws90/91 and reasserts the VIRT ws bindings. As a bonus
# it parks ws90 on the FIRST physical output, which is the glasses panel when present and
# the laptop when not - exactly where mirage (pinned to ws90) needs to land.
bring_up_displays() {
    (
        flock 9
        for attempt in 1 2 3 4; do
            python3 "$HERE/scripts/setup_displays.py" 2>&1 | sed 's/^/  [displays] /'
            [ "${PIPESTATUS[0]}" = 0 ] && break
            echo "[glasses] virtual screens incomplete (attempt $attempt) - retrying..."
            sleep 0.4
        done
    ) 9>"$DISP_LOCK"
}

echo "[glasses] bringing up virtual screens..."
( flock 9; echo "$MY_SESS" > "$DISP_OWNER" ) 9>"$DISP_LOCK"   # claim the displays (supersede older instances)
bring_up_displays

# Now that VIRT1 exists, restart waybar so it attaches a bar there too. mirage
# captures the whole VIRT1 output (waybar is a layer surface on it), so the bar
# rides along onto the wall - that's how you get your status bar inside mirage.
if [ "$WAYBAR_UP" = 1 ]; then
    systemctl --user restart waybar.service 2>/dev/null || true
    sleep 0.5
fi

echo "[glasses] sweeping your windows onto the virtual screens..."
python3 "$HERE/scripts/sweep.py" sweep 2>&1 | sed 's/^/  [sweep] /' || true

# mirage's window rules. PLACEMENT is a SINGLE rule - pin mirage to ws90 - and we move
# ws90 onto the right monitor per mode (apply_mode below). Doing it this way means we
# never stack conflicting per-mode placement rules across a hot-plug switch (Hyprland
# 0.55 has no clean runtime windowrule removal). The rest keep the surface opaque/sharp
# so direct scanout can take it. mirage requests fullscreen itself.
hyprctl eval "
hl.window_rule({ match = { class = 'mirage' }, workspace = '90' })
hl.window_rule({ match = { class = 'mirage' }, opacity = '1.0' })
hl.window_rule({ match = { class = 'mirage' }, no_blur = true })
hl.window_rule({ match = { class = 'mirage' }, rounding = 0 })
" >/dev/null

# ---------------------------------------------------------------------------
# Per-mode helpers. apply_mode() flips everything that differs between glasses
# and windowed; it's safe to call repeatedly (idempotent eval calls).
# ---------------------------------------------------------------------------

# Move mirage's workspace (ws90) onto a given monitor. Re-binds the persistent rule and
# pulls the live workspace across, so the next mirage map lands on the right screen.
move_ws90() {  # $1 = monitor name
    hyprctl eval "
hl.workspace_rule({ workspace='90', monitor='$1', default=true, persistent=true })
hl.dispatch(hl.dsp.workspace.move({ workspace='90', monitor='$1' }))
" >/dev/null 2>&1 || true
}

# Stop the VITURE head-tracking bridge (runs under sudo until a udev replug).
stop_bridge() {
    sudo -n pkill -x viture-bridge 2>/dev/null || pkill -x viture-bridge 2>/dev/null || true
}

apply_mode() {  # $1 = glasses|windowed ; uses $GLASSES / $LAPTOP set by the loop
    if [ "$1" = glasses ]; then
        echo "[glasses] -> GLASSES mode: direct scanout + head tracking on $GLASSES"
        hyprctl eval "hl.config({ render = { direct_scanout = 1 } })" >/dev/null
        # The Beast only exposes 1920x1080@60 over DP on Asahi (its 120Hz mode FREEZES the
        # apple-drm/DCP driver - never set it). Keep DP at the safe 60Hz mode.
        hyprctl eval "hl.monitor({ output='$GLASSES', mode='1920x1080@60', position='auto', scale=1 })" >/dev/null 2>&1 || true
        # Asahi has no HW cursor plane: park the cursor off the glasses output or it breaks scanout.
        hyprctl eval "hl.dispatch(hl.dsp.cursor.move({ x=300, y=300 }))" >/dev/null 2>&1 || true
        # Looking around is the IMU's job now; unfreeze focus-follows-mouse.
        hyprctl eval "hl.config({ input = { follow_mouse = ${FOLLOW_MOUSE_ORIG} } })" >/dev/null 2>&1 || true
        move_ws90 "$GLASSES"          # mirage (pinned to ws90) lands on the glasses panel
        # Head-tracking bridge. viture-bridge.sh is idempotent: it KEEPS a healthy bridge
        # and only replaces a wedged/dead one, so calling it here also serves as the restart
        # path when the bridge gives up (it exits after 10 self-recovery attempts).
        bash "$HERE/scripts/viture-bridge.sh" >>/tmp/viture-bridge.log 2>&1 || true
    else
        echo "[glasses] -> WINDOWED mode: composited on the laptop $LAPTOP (trackpad to look)"
        # No glasses: direct-scanout needs a dedicated panel, so turn it OFF and let mirage
        # composite as a normal fullscreen window on the laptop.
        hyprctl eval "hl.config({ render = { direct_scanout = 0 } })" >/dev/null
        stop_bridge                   # no IMU to read; mirage falls back to trackpad/identity
        move_ws90 "$LAPTOP"           # mirage (pinned to ws90) lands on the laptop
        # Freeze focus-follows-mouse FOR THE MAP ONLY: while mirage is mapping, your cursor
        # sitting over a terminal the sweep dragged onto a headless VIRT would otherwise pull
        # focus there and mirage (client-fullscreen) would map on the invisible VIRT. The ws90
        # pin already places it; this is belt-and-braces for the brief map window. We restore
        # follow_mouse the instant ensure_visible confirms mirage is up (below) - leaving it at
        # 0 for the whole session would break the 3D pointer: the injected wall cursor moves but
        # keyboard focus never follows it, so typing lands on the wrong window.
        hyprctl eval "hl.config({ input = { follow_mouse = 0 } })" >/dev/null 2>&1 || true
        hyprctl dispatch focusmonitor "$LAPTOP" >/dev/null 2>&1 || true
        hyprctl dispatch workspace 90 >/dev/null 2>&1 || true   # show ws90 on the laptop
        (
            python3 "$HERE/scripts/ensure_visible.py" "$LAPTOP"
            # mirage is now mapped & visible on the laptop - the map-time focus-drag risk is
            # over. Unfreeze focus-follows-mouse so the injected 3D-pointer cursor carries
            # keyboard focus, exactly as it does in glasses mode. Consistent across both modes.
            hyprctl eval "hl.config({ input = { follow_mouse = ${FOLLOW_MOUSE_ORIG} } })" >/dev/null 2>&1 || true
        ) >/dev/null 2>&1 &
    fi
}

# Is OUR mirage still running? mirage is our background child, so once it exits it lingers
# as a zombie until reaped - and kill -0/pgrep both still match a zombie. So read the
# process state from /proc and reap (wait) a zombie, returning "dead" for it.
mirage_alive() {  # 0 = running, 1 = dead (reaps a zombie if found)
    [ -n "$MIRAGE_PID" ] || return 1
    local stat st
    stat=$(cat "/proc/$MIRAGE_PID/stat" 2>/dev/null) || return 1   # gone/reaped
    st=${stat##*) }        # drop "pid (comm) "  (comm may contain spaces/parens)
    st=${st%% *}           # first token after that = state letter
    if [ -z "$st" ] || [ "$st" = "Z" ]; then
        wait "$MIRAGE_PID" 2>/dev/null || true
        return 1
    fi
    return 0
}

# Hard-stop just mirage (for a mode switch). No teardown - the displays stay up; only
# mirage is recycled so it re-maps under the new mode. SIGKILL fallback frees a wedged
# render loop's trackpad grab (the kernel drops EVIOCGRAB when the process dies).
kill_mirage() {
    pkill -INT -x mirage 2>/dev/null || true
    for _ in 1 2 3 4 5 6 7 8; do pgrep -x mirage >/dev/null || break; sleep 0.1; done
    if pgrep -x mirage >/dev/null; then
        pkill -KILL -x mirage 2>/dev/null || true
        for _ in 1 2 3 4 5; do pgrep -x mirage >/dev/null || break; sleep 0.1; done
    fi
    [ -n "$MIRAGE_PID" ] && wait "$MIRAGE_PID" 2>/dev/null || true   # reap our zombie child
    MIRAGE_PID=""
}

# Optional test knobs (empty = built-in defaults). Set MIRAGE_PREDICT_MS=0 to disable
# forward-prediction, or MIRAGE_WORLDVIO_GAIN=0 to disable world-cam parallax, when isolating.
export MIRAGE_WORLDVIO="${MIRAGE_WORLDVIO:-cv}"   # 6DoF-lite optical-flow backend (override with proj)
[ -n "${MIRAGE_PREDICT_MS:-}" ]    && export MIRAGE_PREDICT_MS
[ -n "${MIRAGE_WORLDVIO_GAIN:-}" ] && export MIRAGE_WORLDVIO_GAIN
# DIAG (off): export MIRAGE_VIEW_TRACE=1 -> per-frame view trace to /tmp/mirage-view-trace.log

launch_mirage() {
    ./mirage >/tmp/mirage.log 2>&1 &
    MIRAGE_PID=$!
    LAUNCH_TS=$(date +%s)
    echo "[glasses] mirage launched (pid $MIRAGE_PID) - double-Cmd recenter; Super+Shift+Q quits"
}

# ---------------------------------------------------------------------------
# SUPERVISOR LOOP. Observe glasses presence each tick; debounce so DP/USB
# enumeration flicker can't make it flap; apply mode + relaunch on a real
# transition; keep mirage and the bridge alive; exit on the quit sentinel.
# ---------------------------------------------------------------------------
CUR=""              # mode currently applied ("" = nothing applied yet)
STABLE=""           # last observed presence, for debouncing
COUNT=0             # consecutive ticks STABLE has held
MIRAGE_PID=""
echo "[glasses] supervisor up - hot-plug aware (Super+Shift+Q to quit)"
while true; do
    # User asked to quit (stop.sh / panic key set the sentinel before killing mirage).
    [ -f "$SENTINEL" ] && { echo "[glasses] quit requested - shutting down"; break; }

    # Observe. The glasses output is the signal that decides the DISPLAY mode (the Beast's
    # IMU rides the same USB-C cable, so it appears/disappears together).
    G="$(mon_json | PICK=glasses pick_mon 2>/dev/null || true)"
    LAPTOP="$(mon_json | PICK=laptop pick_mon 2>/dev/null || true)"
    if [ -n "$G" ]; then OBS=glasses; GLASSES="$G"; else OBS=windowed; fi

    # Debounce: require the same observation for a few ticks before switching, so a brief
    # DP renegotiation on plug-in doesn't bounce us. Plug-IN gets a longer settle (USB
    # enumerate + cdc_acm unbind) than unplug.
    if [ "$OBS" = "$STABLE" ]; then COUNT=$((COUNT+1)); else STABLE="$OBS"; COUNT=1; fi
    need=2; [ "$STABLE" = glasses ] && need=3

    WANT="$CUR"
    if [ -z "$CUR" ]; then
        WANT="$OBS"                                   # first launch: apply immediately
    elif [ "$STABLE" != "$CUR" ] && [ "$COUNT" -ge "$need" ]; then
        WANT="$STABLE"                                # a debounced, real transition
    fi

    alive=0; mirage_alive && alive=1

    if [ "$WANT" != "$CUR" ]; then
        if [ -n "$CUR" ]; then
            echo "[glasses] glasses $([ "$WANT" = glasses ] && echo plugged in || echo unplugged) - switching to $WANT"
            # Topology changed: re-pin workspaces so a freshly-appeared glasses output can't
            # steal ws1 from VIRT1, and so ws90 follows onto the right physical monitor.
            bring_up_displays
        fi
        [ "$alive" = 1 ] && kill_mirage
        apply_mode "$WANT"
        CUR="$WANT"
        BRIDGE_COOL=0
        launch_mirage
    elif [ "$alive" = 0 ]; then
        # mirage died but no mode change pending. If the user quit, the sentinel is set
        # (checked at the top, and again here to avoid a relaunch racing stop.sh). Otherwise
        # it crashed or was panic-killed without the sentinel -> bring it back.
        [ -f "$SENTINEL" ] && { echo "[glasses] quit requested - shutting down"; break; }
        # Crash backoff: if mirage keeps dying within a few seconds of launch (e.g. a bad
        # build or a wedged GPU state), don't relaunch at 1Hz forever - escalate the wait so
        # logs/USB get a breather, capped so a one-off crash still recovers promptly.
        if [ $(( $(date +%s) - ${LAUNCH_TS:-0} )) -lt 5 ]; then
            CRASHES=$(( ${CRASHES:-0} + 1 ))
        else
            CRASHES=0
        fi
        echo "[glasses] mirage exited unexpectedly in $CUR mode (crash #$CRASHES) - relaunching"
        [ "$CRASHES" -ge 3 ] && { back=$(( CRASHES<10 ? CRASHES*2 : 20 )); echo "[glasses] backing off ${back}s"; sleep "$back"; }
        [ -f "$SENTINEL" ] && { echo "[glasses] quit requested - shutting down"; break; }
        launch_mirage
    fi

    # Keep head tracking alive in glasses mode. The bridge self-recovers from USB stalls
    # (re-exec keeps its PID, so pgrep still matches) but gives up after ~10 attempts and
    # exits; if it's truly gone, restart it. Back off (~8s) between attempts so a bridge
    # that can't start - half-seated cable, missing SDK - doesn't spin the loop.
    if [ "$CUR" = glasses ]; then
        if pgrep -x viture-bridge >/dev/null 2>&1; then
            BRIDGE_COOL=0
        elif [ "${BRIDGE_COOL:-0}" -le 0 ]; then
            echo "[glasses] head-tracking bridge is down - restarting"
            bash "$HERE/scripts/viture-bridge.sh" >>/tmp/viture-bridge.log 2>&1 || true
            BRIDGE_COOL=8
        else
            BRIDGE_COOL=$((BRIDGE_COOL-1))
        fi
    fi

    sleep 1
done
# loop broke -> trap restore runs
