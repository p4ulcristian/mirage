#!/bin/bash
# capture_carina_3dof.sh - Capture HOW SpaceWalker sets up the Carina 3DoF orientation
# tracker + VIO, on macOS, via lldb. This is the deterministic unblock for the Linux
# bring-up: on Linux the VIO boots to stage 1 but the 3DoF orientation tracker emits no
# poses ("getPoseByTimestamp_3dof failed") so the VIO never initialises. We need the
# EXACT init sequence + args SpaceWalker uses.
#
# It hooks (and dumps args + backtrace for) every relevant entry point:
#   carina_vio_init(std::string config, std::string fusion)  <- BOTH args (we pass fusion="")
#   jcx_write_3dof_para(...)                                  <- likely the 3DoF tracker setup
#   carina_vio_set_pose_callback / set_slam_states_string    <- any pre-start config
#   carina_vio_feed_imu / feed_images2                        <- first call only (units/order/rate)
#
# Output files (collect these and send back):
#   /tmp/carina_init_config.yaml   - 1st arg to carina_vio_init (VIO config)
#   /tmp/carina_init_fusion.yaml   - 2nd arg (fusion/orientation config - THE likely 3DoF fix)
#   /tmp/carina_3dof_capture.log   - full trace: every hooked call, args, backtraces
#
# PREREQS: SIP disabled (csrutil disable in Recovery), SpaceWalker quit, Beast plugged in.
# USAGE:   bash tools/capture_carina_3dof.sh   (then USE SpaceWalker normally for ~20s)

set -e
APP="/Applications/SpaceWalker.app/Contents/MacOS/SpaceWalker"
[ -x "$APP" ] || { echo "ERROR: $APP not found"; exit 1; }
if pgrep -x SpaceWalker >/dev/null; then echo "ERROR: quit SpaceWalker first (Cmd+Q)"; exit 1; fi
: > /tmp/carina_3dof_capture.log

cat > /tmp/lldb_3dof.py << 'PY'
import lldb

def read_stdstring(process, addr):
    """Read a libc++ std::string (arm64) at `addr`. Handles SSO + long form."""
    err = lldb.SBError()
    b0 = process.ReadMemory(addr, 1, err)
    if not err.Success(): return None
    flag = b0[0] if isinstance(b0[0], int) else ord(b0[0])
    if flag & 1:                                  # long: [cap][size][data*]
        dptr = process.ReadPointerFromMemory(addr + 16, err)
        if not err.Success() or dptr < 0x1000: return None
        s = process.ReadCStringFromMemory(dptr, 200000, err)
    else:                                         # short: data inline at offset 1
        s = process.ReadCStringFromMemory(addr + 1, 23, err)
    return s if err.Success() else None

def log(msg):
    with open("/tmp/carina_3dof_capture.log", "a") as f: f.write(msg + "\n")
    print(msg)

def _bt(frame):
    out = []
    t = frame.GetThread()
    for i in range(min(12, t.GetNumFrames())):
        out.append("    " + str(t.GetFrameAtIndex(i)))
    return "\n".join(out)

def on_init(frame, bp_loc, d):
    p = frame.GetThread().GetProcess()
    x0 = frame.FindRegister("x0").GetValueAsUnsigned()
    x1 = frame.FindRegister("x1").GetValueAsUnsigned()
    cfg = read_stdstring(p, x0); fus = read_stdstring(p, x1)
    log("\n===== carina_vio_init HIT =====")
    log("--- arg0 config (%d bytes) ---" % (len(cfg) if cfg else 0))
    log(cfg or "<none>")
    log("--- arg1 fusion (%d bytes) ---" % (len(fus) if fus else 0))
    log(fus or "<EMPTY>")
    if cfg:
        open("/tmp/carina_init_config.yaml","w").write(cfg)
    if fus:
        open("/tmp/carina_init_fusion.yaml","w").write(fus)
    log("backtrace:\n" + _bt(frame))
    return False

def on_3dof(frame, bp_loc, d):
    p = frame.GetThread().GetProcess()
    log("\n===== jcx_write_3dof_para HIT =====")
    for r in ("x0","x1","x2","x3","x4","x5"):
        v = frame.FindRegister(r).GetValueAsUnsigned()
        s = read_stdstring(p, v) if v > 0x1000 else None
        err = lldb.SBError()
        cstr = p.ReadCStringFromMemory(v, 4000, err) if v > 0x1000 else None
        log("  %s = 0x%x | stdstr=%r | cstr=%r" % (r, v, (s[:200] if s else None), (cstr[:200] if (cstr and err.Success()) else None)))
    log("backtrace:\n" + _bt(frame))
    return False

def make_logger(name):
    def cb(frame, bp_loc, d):
        log("\n----- %s HIT -----" % name)
        log(_bt(frame))
        return False
    return cb

tgt = lldb.debugger.GetSelectedTarget()
def hook(sym, cb):
    bp = tgt.BreakpointCreateByName(sym)
    bp.SetScriptCallbackFunction("lldb_3dof.%s" % cb)
    print("hooked %s -> %d loc" % (sym, bp.GetNumLocations()))

# extern "C" -> macOS prepends underscore. Try both just in case.
for s in ("carina_vio_init","_carina_vio_init"):
    bp = tgt.BreakpointCreateByName(s); bp.SetScriptCallbackFunction("lldb_3dof.on_init")
for s in ("jcx_write_3dof_para","_jcx_write_3dof_para"):
    bp = tgt.BreakpointCreateByName(s); bp.SetScriptCallbackFunction("lldb_3dof.on_3dof")
PY

cat > /tmp/lldb_3dof.cmds << EOF
command script import /tmp/lldb_3dof.py
run
EOF

echo "=== launching SpaceWalker under lldb ==="
echo "When it opens: USE IT for ~20s (let tracking start, move your head), then Cmd+Q."
echo "Then send back: /tmp/carina_init_config.yaml, /tmp/carina_init_fusion.yaml, /tmp/carina_3dof_capture.log"
echo ""
PYTHONPATH=/tmp lldb -o "command script import /tmp/lldb_3dof.py" -o run "$APP"
