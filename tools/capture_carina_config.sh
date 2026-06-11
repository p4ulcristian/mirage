#!/bin/bash
# capture_carina_config.sh - Capture the config string SpaceWalker passes to carina_vio_init
#
# Prerequisites: Quit SpaceWalker first!
#
# Usage: bash capture_carina_config.sh
#
# This launches SpaceWalker under lldb, sets a breakpoint on carina_vio_init,
# and dumps the first argument (the config YAML string).

set -e
cd "$(dirname "$0")"

if pgrep -x SpaceWalker >/dev/null; then
    echo "ERROR: SpaceWalker is running. Please quit it first (Cmd+Q)."
    exit 1
fi

echo "=== Capturing carina_vio_init config from SpaceWalker ==="
echo ""
echo "SpaceWalker will launch. When it does:"
echo "  1. Wait for lldb to hit the breakpoint"
echo "  2. The config will be dumped to /tmp/carina_config_captured.yaml"
echo "  3. Press Ctrl+C to exit (or let SpaceWalker run)"
echo ""
echo "Starting in 3 seconds..."
sleep 3

# Create lldb commands file
cat > /tmp/lldb_capture.txt << 'EOF'
# Set breakpoint on carina_vio_init
breakpoint set -n _carina_vio_init

# Run the app
run

# When we hit the breakpoint, dump the first argument
# On ARM64 macOS, first arg is in x0 (pointer to std::string)
# std::string on libc++ (macOS): if small string optimization (SSO) is active,
# the string data is inline; otherwise it's a pointer.
# For our purposes, we'll try to read it as a C string from the data pointer.

# This is a simple approach - read x0 as a std::string pointer:
# - x0 points to the string object
# - offset 0: data pointer (or inline data if SSO)
# - We'll try to read the first few KB as a string

script
import lldb
def dump_config(frame, bp_loc, dict):
    thread = frame.GetThread()
    process = thread.GetProcess()
    target = process.GetTarget()

    x0 = frame.FindRegister("x0").GetValueAsUnsigned()
    error = lldb.SBError()

    # Try to read the string. std::string has data at offset 0 for long strings.
    # For short strings (SSO), data is inline starting at offset 0.
    # Try reading as C string from various offsets.

    data = None
    for offset in [0, 8, 16]:
        ptr = process.ReadPointerFromMemory(x0 + offset, error)
        if error.Success() and ptr > 0x1000:
            # Try to read as C string
            data = process.ReadCStringFromMemory(ptr, 50000, error)
            if error.Success() and data and len(data) > 10 and "cam0" in data:
                break
            data = None

    # If that didn't work, try reading directly from x0 (SSO case)
    if not data:
        data = process.ReadCStringFromMemory(x0, 50000, error)

    if data and len(data) > 10:
        print("\n" + "="*60)
        print("CAPTURED carina_vio_init CONFIG:")
        print("="*60)
        print(data[:5000])  # Print first 5KB
        if len(data) > 5000:
            print(f"\n... ({len(data)} total bytes)")
        print("="*60 + "\n")

        with open("/tmp/carina_config_captured.yaml", "w") as f:
            f.write(data)
        print("Saved to: /tmp/carina_config_captured.yaml")
    else:
        print("Could not read config string")

    return False  # Don't stop

target = lldb.debugger.GetSelectedTarget()
bp = target.BreakpointCreateByName("_carina_vio_init")
bp.SetScriptCallbackFunction("dump_config")
EOF

lldb -s /tmp/lldb_capture.txt /Applications/SpaceWalker.app/Contents/MacOS/SpaceWalker
