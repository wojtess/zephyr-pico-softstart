#!/usr/bin/env python3
"""
Test script for ADC communication with RP2040.

Reads ADC value from GPIO26 (12-bit, 0-3.3V range).
"""

import sys
import glob
import time

try:
    import serial
except ImportError:
    print("ERROR: pyserial not installed")
    sys.exit(1)

# Add parent directory to path for protocol import
sys.path.insert(0, '.')

from protocol import CMD_READ_ADC, build_adc_frame, parse_adc_response, crc8


def find_acm_port():
    """Find USB CDC ACM port (Linux/macOS/Windows)."""
    # Linux
    ports = glob.glob("/dev/ttyACM*")
    if ports:
        return ports[0]

    # macOS
    ports = glob.glob("/dev/cu.usbmodem*")
    if ports:
        return ports[0]

    # Windows - try serial.tools
    try:
        import serial.tools.list_ports
        for port in serial.tools.list_ports.comports():
            if "ACM" in port.description:
                return port.device
    except (ImportError, OSError):
        pass

    return None


def test_adc():
    """Test ADC read functionality."""
    port = find_acm_port()
    if not port:
        print("ERROR: No ACM port found")
        return 1

    print(f"Connecting to {port}...")

    try:
        ser = serial.Serial(port, 115200, timeout=2)
        time.sleep(0.5)  # Wait for device to be ready

        # Flush buffers
        ser.reset_input_buffer()
        ser.reset_output_buffer()

        print("Connected. Reading ADC...")

        # Send ADC read command
        frame = build_adc_frame()
        print(f"Sending: {frame.hex()}")

        ser.write(frame)
        ser.flush()

        # Read response (4 bytes: [CMD][ADC_H][ADC_L][CRC])
        resp = ser.read(4)

        if len(resp) == 0:
            print("ERROR: No response (timeout)")
            return 1

        print(f"Received: {resp.hex()}")

        # Parse response
        adc_value, err_code = parse_adc_response(resp)

        if adc_value >= 0:
            # Convert to voltage (0-3.3V)
            voltage = (adc_value / 4095.0) * 3.3
            print(f"ADC Raw: {adc_value} (0x{adc_value:04X})")
            print(f"Voltage: {voltage:.3f} V")
            print(f"Percentage: {(adc_value / 4095.0) * 100:.1f}%")
        else:
            print(f"ERROR: {err_code}")

        ser.close()
        return 0

    except serial.SerialException as e:
        print(f"Serial error: {e}")
        return 1
    except KeyboardInterrupt:
        print("\nInterrupted")
        return 1


if __name__ == "__main__":
    sys.exit(test_adc())
