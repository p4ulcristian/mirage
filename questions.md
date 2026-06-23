# questions.md — the ONE Mac errand (everything I need, in one shot)

Bro — sorry for the back-and-forth. This is **everything**, in one script, one folder back.
Do this once and (hopefully) we never need the Mac again.

## TL;DR

```bash
cd /path/to/mirage
git checkout viture-beast && git pull
bash tools/capture_carina_3dof.sh
# then push the folder it makes:
git add mac-capture && git commit -m "mac-capture: full carina trace" && git push
```

That's it. The script auto-does the easy part first (no reboot), and only uses SIP if it's
already off. Details below if anything's weird.

---

## Why this exists (so it's the last time)

On Linux we now run VITURE's **exact same tracking engine** as your Mac (`libcarina_vio`).
It boots, it takes live IMU + camera — but its **3DoF orientation tracker emits no poses**,
so tracking never starts. SpaceWalker on your Mac configures that engine *correctly*; we
don't, because we don't know its exact setup. This capture records SpaceWalker's setup so I
can copy it 1:1. (The Beast's firmware does NOT send a finished pose to any computer — Mac
included — so "native mode" isn't the mechanism; the host engine is. That's why this matters.)

---

## What the script grabs (so I'm asking for ALL of it now)

It writes everything into a `mac-capture/` folder:

**Phase 1 — files on disk (NO SIP, no reboot):**
- Every `*.yaml / *.json / *config* / *ivctrl* / *huayu* / *slam* / *calib*` inside
  `SpaceWalker.app` (the engine's on-disk config — may already contain the full thing).
- Config/cache files SpaceWalker writes at runtime (`~/Library/Application Support/...`,
  caches, prefs, logs).
- `config.yaml` / `ivctrl_huayu.yaml` (the binary references these paths).
- `engine_info.txt`: the `libcarina_vio.dylib` md5 + version + the exact config-schema
  strings it reads (camera model, `T_cam_imu`, `orientation_tracker`, `imu_rate`, etc.) —
  confirms it matches our Linux copy.

**Phase 2 — live trace (ONLY if SIP is already off):** launches SpaceWalker under lldb and
logs, with args + backtraces, into `mac-capture/`:
- `carina_init_config.yaml` — 1st arg to `carina_vio_init` (the real VIO config: intrinsics,
  `T_cam_imu`, resolution, all of it).
- `carina_init_fusion.yaml` — **2nd arg, which we currently pass empty** (prime suspect for
  the 3DoF tracker config).
- `carina_trace.log` — the full call sequence: `jcx_write_3dof_para` (the 3DoF setup) with
  its args, `feed_imu` (confirms IMU order/units), `feed_images` (mono vs stereo), pose
  callbacks, init order. This is the recipe.

---

## Do I need to disable SIP?

**Try WITHOUT it first.** Just run the script. If Phase-1 already dumped a `config.yaml` /
`ivctrl_huayu.yaml` / a full slam config, that might be all I need — send the folder and I'll
tell you. **Don't reboot yet.**

**Only if I say Phase-1 wasn't enough**, then do SIP (once):
1. Shut down. Hold power → **Options** → Continue (Recovery).
2. Utilities → Terminal → `csrutil disable` → reboot to macOS.
3. Plug in Beast, quit SpaceWalker, re-run `bash tools/capture_carina_3dof.sh`.
   When SpaceWalker opens: **use it ~20s, move your head**, then Cmd+Q.
4. Push `mac-capture/`. Then re-enable: Recovery → `csrutil enable`.

---

## If lldb hits nothing (Phase 2 prints `hook ... -> 0 loc`)

Symbols are stripped. Fallback (tell me first, but here it is):
```bash
brew install frida-tools 2>/dev/null || pip3 install frida-tools
frida-trace -f "/Applications/SpaceWalker.app/Contents/MacOS/SpaceWalker" \
  -i "carina_vio_*" -i "jcx_*" 2>&1 | tee mac-capture/frida_trace.log
# use SpaceWalker ~20s, Cmd+Q, then push mac-capture/
```

---

## What I'll do the moment the folder lands

1. Diff the dylib md5/version vs our `libcarina_vio.so` (confirm identical behavior).
2. Copy SpaceWalker's exact `carina_vio_init` config (both args) into `src/viture_vio.cpp` —
   real intrinsics + `T_cam_imu` + resolution, and the fusion arg we pass empty.
3. If `jcx_write_3dof_para` is in the trace, replicate that call (right args, right order)
   before start — that's almost certainly what lights up the 3DoF tracker.
4. Re-run `./viture-vio --run` on the Beast (it's on the Linux box) → expect a real pose.
5. Wire it into mirage.

Send me the `mac-capture/` folder and I take it from there. 🙏
