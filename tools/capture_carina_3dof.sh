#!/bin/bash
# capture_carina_3dof.sh - ONE-SHOT capture of EVERYTHING SpaceWalker does to drive the
# Carina engine, so we can replicate it on Linux. Two phases:
#   PHASE 1 (NO SIP NEEDED): harvest every config file on disk + the dylib version.
#   PHASE 2 (needs SIP off): lldb-trace the live carina_vio_* / jcx_* calls + their args.
#
# Output dir: ./mac-capture/   (everything lands here; commit+push it)
#
# USAGE:
#   bash tools/capture_carina_3dof.sh            # runs phase 1, then phase 2 if SIP is off
#   bash tools/capture_carina_3dof.sh files      # phase 1 only (no SIP, no lldb)

set -u
APP="/Applications/SpaceWalker.app"
BIN="$APP/Contents/MacOS/SpaceWalker"
OUT="$(cd "$(dirname "$0")/.." && pwd)/mac-capture"
mkdir -p "$OUT"
echo "=== output -> $OUT ==="

############################ PHASE 1 — files on disk (NO SIP) ############################
echo "=== PHASE 1: harvesting config files (no SIP needed) ==="

# 1a. every config-ish file in the app bundle -> copy the small ones, list all
find "$APP" -type f \( -iname "*.yaml" -o -iname "*.yml" -o -iname "*.json" \
   -o -iname "*config*" -o -iname "*ivctrl*" -o -iname "*huayu*" -o -iname "*slam*" \
   -o -iname "*calib*" \) 2>/dev/null | tee "$OUT/bundle_config_files.txt" | while read -r f; do
   sz=$(stat -f%z "$f" 2>/dev/null || echo 0)
   if [ "$sz" -lt 600000 ]; then
       dest="$OUT/bundle_$(echo "$f" | sed 's#.*/Contents/##; s#/#_#g')"
       cp "$f" "$dest" 2>/dev/null && echo "  copied $f ($sz b)"
   else
       echo "  (big, listed only) $f ($sz b)"
   fi
done

# 1b. config files SpaceWalker writes/reads at runtime (Application Support, caches, cwd)
for d in "$HOME/Library/Application Support/com.viture.spacewalker" \
         "$HOME/Library/Application Support/SpaceWalker" \
         "$HOME/Library/Caches/com.viture.spacewalker" \
         "$HOME/Library/Preferences"; do
   [ -d "$d" ] || continue
   echo "  scanning $d"
   find "$d" -type f \( -iname "*.yaml" -o -iname "*.yml" -o -iname "*.json" \
      -o -iname "*config*" -o -iname "*.plist" -o -iname "*.log" -o -iname "*calib*" \) 2>/dev/null \
      | while read -r f; do
         sz=$(stat -f%z "$f" 2>/dev/null || echo 0)
         [ "$sz" -lt 2000000 ] && cp "$f" "$OUT/appsupport_$(basename "$f")" 2>/dev/null && echo "    copied $(basename "$f") ($sz b)"
      done
done

# 1c. the engine dylib: version + md5 (so we confirm it matches our vendored .so) + the
#     config-schema strings (camera models, the 3dof/orientation keys it reads)
DYLIB=$(find "$APP" -iname "libcarina_vio*" 2>/dev/null | head -1)
echo "libcarina_vio.dylib: ${DYLIB:-NOT FOUND}" | tee "$OUT/engine_info.txt"
if [ -n "${DYLIB:-}" ]; then
   { echo "md5: $(md5 -q "$DYLIB" 2>/dev/null)"
     echo "size: $(stat -f%z "$DYLIB" 2>/dev/null)"
     echo "--- version-ish strings ---"
     strings "$DYLIB" | grep -iE "viture|v[0-9]+\.[0-9]+\.[0-9]+|slam_version|carina" | sort -u | head -30
     echo "--- camera/3dof/orientation config keys it reads ---"
     strings "$DYLIB" | grep -iE "camera_model|distortion_model|T_cam_imu|T_imu_cam|orientation_tracker|complementary_filter|kalman_filter|imu_rate|gravity|orb_database|config\.yaml|ivctrl|huayu|3dof|write_3dof" | sort -u
   } >> "$OUT/engine_info.txt"
   echo "  wrote engine_info.txt"
fi

# 1d. where does it look for config.yaml? (the binary references "%s/config.yaml" + ivctrl_huayu.yaml)
find "$APP" -iname "config.yaml" -o -iname "ivctrl_huayu.yaml" 2>/dev/null | tee -a "$OUT/bundle_config_files.txt"

echo "=== PHASE 1 done. Files in $OUT. ==="

[ "${1:-}" = "files" ] && { echo "(files-only mode; skipping lldb)"; exit 0; }

############################ PHASE 2 — live trace (needs SIP off) #########################
if csrutil status 2>/dev/null | grep -qi "enabled"; then
   echo ""
   echo "!!! SIP is ENABLED — lldb can't attach. Phase 2 skipped."
   echo "!!! If phase-1 files don't contain the full config, disable SIP (Recovery: csrutil disable) and re-run."
   echo "!!! Otherwise just send me the $OUT folder — it may already be enough."
   exit 0
fi
if pgrep -x SpaceWalker >/dev/null; then echo "ERROR: quit SpaceWalker first (Cmd+Q)"; exit 1; fi
[ -x "$BIN" ] || { echo "ERROR: $BIN not found"; exit 1; }

cat > "$OUT/lldb_trace.py" << 'PY'
import lldb
LOG = __import__("os").environ.get("CARINA_LOG", "/tmp/carina_trace.log")
CFG = __import__("os").path.dirname(LOG)
def log(m):
    open(LOG,"a").write(m+"\n"); print(m)
def rd_str(p, addr):
    e=lldb.SBError(); b=p.ReadMemory(addr,1,e)
    if not e.Success(): return None
    flag=b[0] if isinstance(b[0],int) else ord(b[0])
    if flag&1:
        dp=p.ReadPointerFromMemory(addr+16,e);
        return p.ReadCStringFromMemory(dp,300000,e) if (e.Success() and dp>0x1000) else None
    return p.ReadCStringFromMemory(addr+1,23,e)
def rd_vecf(p, addr):
    e=lldb.SBError()
    beg=p.ReadPointerFromMemory(addr,e); end=p.ReadPointerFromMemory(addr+8,e)
    if not e.Success() or beg<0x1000 or end<beg or end-beg>4096: return None
    n=(end-beg)//4; out=[]
    for i in range(n):
        d=p.ReadMemory(beg+4*i,4,e)
        if not e.Success(): break
        import struct; out.append(round(struct.unpack('<f',d)[0],5))
    return out
def bt(fr):
    t=fr.GetThread(); return "\n".join("    "+str(t.GetFrameAtIndex(i)) for i in range(min(14,t.GetNumFrames())))
def init_cb(fr,bl,d):
    p=fr.GetThread().GetProcess()
    c=rd_str(p,fr.FindRegister("x0").GetValueAsUnsigned())
    f=rd_str(p,fr.FindRegister("x1").GetValueAsUnsigned())
    log("\n===== carina_vio_init ====="); log("ARG0 config (%d b):"%(len(c) if c else 0)); log(c or "<none>")
    log("ARG1 fusion (%d b):"%(len(f) if f else 0)); log(f or "<EMPTY>")
    if c: open(CFG+"/carina_init_config.yaml","w").write(c)
    if f: open(CFG+"/carina_init_fusion.yaml","w").write(f)
    log("backtrace:\n"+bt(fr)); return False
_imu=[0]
def imu_cb(fr,bl,d):
    if _imu[0]>=3: return False
    _imu[0]+=1; p=fr.GetThread().GetProcess()
    v=rd_vecf(p,fr.FindRegister("x0").GetValueAsUnsigned())
    log("feed_imu #%d vec=%s (order: SpaceWalker's; we send accel,accel,accel,gyro,gyro,gyro)"%(_imu[0],v)); return False
_img=[0]
def img_cb(fr,bl,d):
    if _img[0]>=2: return False
    _img[0]+=1
    x0=fr.FindRegister("x0").GetValueAsUnsigned(); x1=fr.FindRegister("x1").GetValueAsUnsigned()
    log("feed_images: img0=0x%x img1=0x%x (img1==0 -> MONO)"%(x0,x1)); log(bt(fr)); return False
def jcx_cb(fr,bl,d):
    p=fr.GetThread().GetProcess(); log("\n===== jcx_write_3dof_para =====")
    for r in ("x0","x1","x2","x3","x4"):
        v=fr.FindRegister(r).GetValueAsUnsigned(); s=rd_str(p,v) if v>0x1000 else None
        e=lldb.SBError(); cs=p.ReadCStringFromMemory(v,2000,e) if v>0x1000 else None
        log("  %s=0x%x str=%r cstr=%r"%(r,v,(s[:150] if s else None),(cs[:150] if (cs and e.Success()) else None)))
    log(bt(fr)); return False
def order_cb_factory(name):
    def cb(fr,bl,d): log("\n--- %s ---\n%s"%(name,bt(fr))); return False
    return cb
tgt=lldb.debugger.GetSelectedTarget()
def H(sym,cb):
    b=tgt.BreakpointCreateByName(sym); b.SetScriptCallbackFunction("lldb_trace."+cb); log("hook %s -> %d loc"%(sym,b.GetNumLocations()))
for s in ("carina_vio_init","_carina_vio_init"): H(s,"init_cb")
for s in ("carina_vio_feed_imu","_carina_vio_feed_imu"): H(s,"imu_cb")
for s in ("carina_vio_feed_images2","carina_vio_feed_images4","_carina_vio_feed_images2"): H(s,"img_cb")
for s in ("jcx_write_3dof_para","_jcx_write_3dof_para"): H(s,"jcx_cb")
for s in ("carina_vio_set_pose_callback","carina_vio_activate_localization_mode","carina_vio_reset","carina_vio_set_slam_states_string"):
    b=tgt.BreakpointCreateByName(s); b.SetScriptCallbackFunction("lldb_trace.order_cb_factory")  # noqa
PY

: > "$OUT/carina_trace.log"
echo ""
echo "=== PHASE 2: launching SpaceWalker under lldb ==="
echo ">>> When it opens: USE it ~20s, let tracking start, MOVE your head, then Cmd+Q. <<<"
echo ""
PYTHONPATH="$OUT" CARINA_LOG="$OUT/carina_trace.log" \
  lldb -o "command script import $OUT/lldb_trace.py" -o run "$BIN"

echo ""
echo "=== DONE. Send back the whole folder: $OUT ==="
ls -la "$OUT"
