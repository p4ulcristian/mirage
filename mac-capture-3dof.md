# mac-capture-3dof.md — capture SpaceWalker's Carina init on the Mac

Read this on the Mac. It's everything you need to grab the last missing piece so the
Beast's real VITURE VIO anchor works on the Linux box. ~15 min (most of it reboots).

---

## Where we are (so you know why this matters)

On the Linux/Asahi box the Beast's VITURE **Carina VIO** engine now:
- ✅ boots and initialises (we fixed the config: `T_cam_imu` key + `equidistant` fisheye model),
- ✅ streams live on hardware — IMU @137Hz + world cam (/dev/video1, 640×480) both feeding it,
- ❌ but the **3DoF orientation tracker inside it produces no poses**
  (`getPoseByTimestamp_3dof failed` spam) → the VIO never initialises → pose stays `nan`.

That 3DoF tracker is configured by something we DON'T pass. Two prime suspects:
1. **The 2nd argument to `carina_vio_init`** (the "fusion" config) — we pass `""` (empty).
   SpaceWalker probably passes the orientation-tracker config there.
2. **`jcx_write_3dof_para(...)`** — an export that "writes 3dof params"; SpaceWalker likely
   calls it to set up the tracker. Unknown signature.

This capture intercepts SpaceWalker's actual calls and dumps both — removing all guesswork.
Bonus: it also dumps the exact `cam0` block, giving us the **real fisheye intrinsics +
`T_cam_imu`**, which kills the calibration guesswork too.

---

## Step 1 — get the script (it's already in the repo on `viture-beast`)

```bash
cd /path/to/mirage
git fetch origin && git checkout viture-beast && git pull
ls tools/capture_carina_3dof.sh    # should exist
```

## Step 2 — disable SIP (one-time; required for lldb to attach to a signed app)

1. Shut down the Mac.
2. Apple Silicon: hold the **power button** until "Loading startup options" → click **Options** → **Continue** (Recovery Mode).
   (Intel: reboot holding **Cmd+R**.)
3. Menu bar → **Utilities → Terminal**, then:
   ```bash
   csrutil disable
   ```
4. Reboot back into macOS normally.

(Verify after reboot if you want: `csrutil status` → "System Integrity Protection status: disabled".)

## Step 3 — run the capture

- Plug the **Beast** into the Mac.
- **Quit SpaceWalker** if it's open (Cmd+Q).
- Run:
  ```bash
  cd /path/to/mirage
  bash tools/capture_carina_3dof.sh
  ```
- SpaceWalker launches under lldb. **Use it normally for ~20 seconds** — let head tracking
  actually engage, move your head around so the VIO initialises (that's when the interesting
  calls fire). Then **Cmd+Q** to quit SpaceWalker.

## Step 4 — send these 3 files back

```bash
ls -la /tmp/carina_init_config.yaml /tmp/carina_init_fusion.yaml /tmp/carina_3dof_capture.log
```
- `/tmp/carina_init_config.yaml` — the VIO config (cam0 intrinsics, T_cam_imu, etc.)
- `/tmp/carina_init_fusion.yaml` — **the 2nd init arg we pass empty** (likely the 3DoF fix)
- `/tmp/carina_3dof_capture.log` — full trace: every hooked call + args + backtraces

Easiest way to get them to me — drop into the repo and push:
```bash
mkdir -p mac-dump
cp /tmp/carina_init_config.yaml /tmp/carina_init_fusion.yaml /tmp/carina_3dof_capture.log mac-dump/ 2>/dev/null
git add mac-dump && git commit -m "mac-dump: captured SpaceWalker carina init (3dof)" && git push
```
(If a file is missing because that call never fired, just send whatever exists + tell me.)

## Step 5 — re-enable SIP (don't skip)

Reboot to Recovery Mode again (as in Step 2), Terminal:
```bash
csrutil enable
```
Reboot back to macOS.

---

## If it captures nothing (no breakpoints hit)

The script prints `hooked carina_vio_init -> N loc` style lines when it loads. If N=0, or
no `===== ... HIT =====` blocks appear in the log, the symbols are stripped or lazily
bound. Tell me and I'll switch to a **frida** approach (hook by address):
```bash
# only if lldb-by-name fails:
brew install frida-tools           # or: pip3 install frida-tools
frida-trace -f "/Applications/SpaceWalker.app/Contents/MacOS/SpaceWalker" \
  -i "carina_vio_init" -i "jcx_write_3dof_para"
```
…but try the lldb script first — with SIP off it should work.

---

## What I'll do with it (back on Linux)

1. If `carina_init_fusion.yaml` is non-empty → feed it as `carina_vio_init`'s 2nd arg in
   `src/viture_vio.cpp`. Likely unblocks the 3DoF tracker by itself.
2. If `jcx_write_3dof_para` fired → replicate that call with the captured args before start.
3. Swap our estimated `cam0` (intrinsics `285,285,320,240`, identity `T_cam_imu`) for the
   real captured values.
4. Re-run `./viture-vio --run --cam /dev/video1 --res 640 480` on the Beast → expect the
   pose to leave `nan` and start tracking. Then wire it into mirage.
