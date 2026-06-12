#!/usr/bin/env python3
"""
Beast HID protocol test - SDK-free communication
Requires: pip install hidapi
"""
import hid
import time
import struct
import sys

VID = 0x35CA
PID = 0x1201

def main():
    print("Looking for Beast...")
    for dev in hid.enumerate(VID, PID):
        print(f"  Found: {dev['product_string']} if={dev['interface_number']}")
    
    h = hid.device()
    h.open(VID, PID)
    h.set_nonblocking(True)
    print(f"Opened: {h.get_product_string()}")
    
    def cmd(data, desc=""):
        pkt = bytes(data + [0]*(64-len(data)))
        h.write(pkt)
        time.sleep(0.05)
        r = h.read(64, timeout_ms=100)
        if r:
            hex_str = ' '.join(f'{b:02x}' for b in r[:32])
            print(f"{desc}: {hex_str}")
        return r
    
    # Query firmware
    cmd([0x10, 0x00, 0x03, 0x30], "Firmware query")
    cmd([0x10, 0x00, 0x01, 0x32], "Version")
    cmd([0x10, 0x00, 0x24, 0x31], "Display size")
    cmd([0x10, 0x00, 0x01, 0x31], "Config (special)")
    
    # Try SpaceWalker config commands
    cmd([0x10, 0x00, 0x01, 0x34, 0x08, 0x00, 0x1c, 0x00, 0x07, 0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06], "SW Config 1")
    cmd([0x10, 0x00, 0x02, 0x34, 0x07, 0x00, 0x15, 0x00, 0x06, 0x00, 0x01, 0x02, 0x03, 0x04, 0x05], "SW Config 2")
    
    # Poll for data
    print("\n--- Polling (move glasses!) ---")
    for i in range(100):
        r = h.read(64, timeout_ms=50)
        if r and r[0] != 0 and r[3] not in [0x50, 0x51, 0x52, 0x54, 0x55]:  # Skip known responses
            print(f"[{i}] {' '.join(f'{b:02x}' for b in r[:24])}")
        time.sleep(0.02)
    
    h.close()

if __name__ == "__main__":
    main()
