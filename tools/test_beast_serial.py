#!/usr/bin/env python3
"""
Quick test: read IMU data directly from Beast via serial (no SDK)
"""
import serial
import struct
import sys
import time

# Beast serial port (macOS)
PORT = "/dev/cu.usbmodem000000011"
BAUD = 115200

# Packet format
PACKET_SIZE = 64
HDR_IMU = bytes([0xFF, 0xFC])
HDR_MCU = bytes([0xFF, 0xFE])
CMD_SET_IMU = 0x15

def crc16_ccitt(data):
    """CRC-16-CCITT (poly 0x1021, init 0xFFFF)"""
    crc = 0xFFFF
    for byte in data:
        crc ^= byte << 8
        for _ in range(8):
            if crc & 0x8000:
                crc = (crc << 1) ^ 0x1021
            else:
                crc <<= 1
            crc &= 0xFFFF
    return crc

def build_mcu_cmd(cmd_id, data_byte):
    """Build a 64-byte MCU command packet"""
    pkt = bytearray(PACKET_SIZE)
    pkt[0:2] = HDR_MCU
    # payload_len at offset 4-5 (little-endian)
    payload_len = 12
    pkt[4] = payload_len & 0xFF
    pkt[5] = (payload_len >> 8) & 0xFF
    # cmd_id at offset 14-15
    pkt[14] = cmd_id & 0xFF
    pkt[15] = (cmd_id >> 8) & 0xFF
    # data at offset 18
    pkt[18] = data_byte
    # CRC over bytes 4+
    crc = crc16_ccitt(pkt[4:])
    pkt[2] = crc & 0xFF
    pkt[3] = (crc >> 8) & 0xFF
    return bytes(pkt)

def main():
    port = sys.argv[1] if len(sys.argv) > 1 else PORT
    print(f"Opening {port}...")

    try:
        ser = serial.Serial(port, BAUD, timeout=1)
    except Exception as e:
        print(f"Error: {e}")
        print("Make sure SpaceWalker is NOT running!")
        return 1

    # Send enable IMU command
    print("Sending enable IMU command...")
    cmd = build_mcu_cmd(CMD_SET_IMU, 0x01)
    ser.write(cmd)
    time.sleep(0.1)

    print("Reading packets... (Ctrl-C to stop)")
    print("-" * 60)

    n = 0
    last_print = time.time()

    try:
        while True:
            # Read bytes until we find IMU header
            data = ser.read(PACKET_SIZE)
            if len(data) < PACKET_SIZE:
                continue

            # Check header
            if data[0:2] == HDR_IMU:
                # Parse euler angles from offset 18
                try:
                    roll = struct.unpack('<f', data[18:22])[0]
                    pitch = struct.unpack('<f', data[22:26])[0]
                    yaw = struct.unpack('<f', data[26:30])[0]
                    yaw = -yaw  # negate per protocol

                    n += 1
                    if time.time() - last_print >= 0.25:
                        print(f"IMU #{n:6d} | yaw={yaw:7.1f} pitch={pitch:7.1f} roll={roll:7.1f}")
                        last_print = time.time()
                except:
                    pass
            elif data[0:2] == HDR_MCU:
                print(f"MCU response: cmd={data[14]:02x} data={data[18]:02x}")
            else:
                # Unknown header, show first few bytes
                print(f"Unknown: {data[:10].hex()}")

    except KeyboardInterrupt:
        print(f"\nStopped after {n} packets")

    # Disable IMU
    print("Disabling IMU...")
    cmd = build_mcu_cmd(CMD_SET_IMU, 0x00)
    ser.write(cmd)
    ser.close()
    return 0

if __name__ == "__main__":
    sys.exit(main())
