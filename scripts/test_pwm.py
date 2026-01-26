#!/usr/bin/env python3
"""
RP2040 Built-in LED Control Test Script

Tests binary UART protocol:
- Frame: [CMD][VALUE][CRC8] - 3 bytes
- CMD 0x01 = SET LED (0=OFF, 1+=ON)
- Response: ACK 0xFF or NACK 0xFE + error code
"""

import serial
import time
import sys

# Protocol constants
CMD_SET_LED = 0x01
RESP_ACK = 0xFF
RESP_NACK = 0xFE
ERR_CRC = 0x01
ERR_INVALID_CMD = 0x02
ERR_INVALID_VAL = 0x03


def crc8(data: bytes) -> int:
    """Calculate CRC-8 (Polynomial 0x07, Initial 0x00)"""
    crc = 0x00
    for byte in data:
        crc ^= byte
        for _ in range(8):
            if crc & 0x80:
                crc = (crc << 1) ^ 0x07
            else:
                crc <<= 1
    return crc & 0xFF


def send_led_command(ser: serial.Serial, state: int) -> bool:
    """Send SET LED command and check response (0=OFF, 1+=ON)"""
    if state < 0 or state > 255:
        print(f"  ERROR: Invalid value {state}")
        return False

    cmd = bytes([CMD_SET_LED, state])
    crc = crc8(cmd)
    frame = cmd + bytes([crc])

    state_str = "OFF" if state == 0 else f"ON ({state})"
    print(f"  Sending: CMD=0x{CMD_SET_LED:02X} VALUE={state} CRC=0x{crc:02X}")
    ser.write(frame)

    # Read response
    resp = ser.read(1)
    if not resp:
        print("  ERROR: No response")
        return False

    resp_byte = resp[0]
    if resp_byte == RESP_ACK:
        print(f"  -> ACK (0x{resp_byte:02X}) ✓")
        return True
    elif resp_byte == RESP_NACK:
        err = ser.read(1)
        err_code = err[0] if err else 0
        err_name = {
            ERR_CRC: "CRC error",
            ERR_INVALID_CMD: "Invalid command",
            ERR_INVALID_VAL: "Invalid value",
        }.get(err_code, f"Unknown error {err_code}")
        print(f"  -> NACK (0x{resp_byte:02X}) {err_name}")
        return False
    else:
        print(f"  -> Unknown response 0x{resp_byte:02X}")
        return False


def find_port() -> str:
    """Auto-detect USB CDC ACM port"""
    import glob

    # Linux
    ports = glob.glob("/dev/ttyACM*")
    if ports:
        return ports[0]

    # macOS
    ports = glob.glob("/dev/cu.usbmodem*")
    if ports:
        return ports[0]

    # Windows
    try:
        import serial.tools.list_ports
        for port in serial.tools.list_ports.comports():
            if "ACM" in port.description or "USB" in port.description:
                return port.device
    except ImportError:
        pass

    return None


def main():
    print("RP2040 Built-in LED Control - Test Script")
    print("=" * 50)

    # Find port
    port = find_port()
    if not port:
        print("ERROR: No USB CDC ACM device found!")
        print("Connect the RP2040 and try again.")
        sys.exit(1)

    print(f"Port: {port}")

    # Open serial connection
    try:
        ser = serial.Serial(port, 115200, timeout=1)
        time.sleep(0.1)

        # Flush any initial data
        ser.reset_input_buffer()
        ser.reset_output_buffer()

        # Wait for boot messages
        time.sleep(0.5)
        if ser.in_waiting:
            print("\nBoot messages:")
            print(ser.read(ser.in_waiting).decode('utf-8', errors='ignore'))

    except serial.SerialException as e:
        print(f"ERROR: Cannot open port: {e}")
        sys.exit(1)

    print("\n" + "=" * 50)
    print("Testing LED commands (GPIO25 - built-in):")
    print("=" * 50)

    # Test sequence
    tests = [
        (1, "LED ON"),
        (0, "LED OFF"),
        (1, "LED ON"),
        (0, "LED OFF"),
        (1, "LED ON"),
        (1, "LED ON (stay ON)"),
        (0, "LED OFF"),
    ]

    passed = 0
    failed = 0

    for value, description in tests:
        print(f"\n{description}:")
        if send_led_command(ser, value):
            passed += 1
        else:
            failed += 1
        time.sleep(0.3)  # Longer delay to see LED change

    # Test error cases
    print("\n" + "=" * 50)
    print("Testing error cases:")
    print("=" * 50)

    # Send bad CRC manually
    print("\nBad CRC (should fail):")
    bad_frame = bytes([CMD_SET_LED, 1, 0xFF])  # Wrong CRC
    ser.write(bad_frame)
    resp = ser.read(2)
    if resp:
        print(f"  -> NACK {resp.hex()}")

    # Summary
    print("\n" + "=" * 50)
    print(f"Results: {passed} passed, {failed} failed")
    print("=" * 50)

    ser.close()


if __name__ == "__main__":
    main()
