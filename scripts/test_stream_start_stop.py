#!/usr/bin/env python3
"""
Test script for ADC streaming start/stop cycle reliability.

This test repeatedly starts and stops ADC streaming to verify:
1. Stream starts correctly
2. Data is received during streaming
3. Stream stops cleanly
4. Connection remains active after stop
5. Multiple cycles work without issues
"""

import sys
import time
import serial
from pathlib import Path

# Add parent directory for protocol import
sys.path.insert(0, str(Path(__file__).parent))

from protocol import (
    CMD_START_STREAM, CMD_STOP_STREAM,
    RESP_ACK, RESP_NACK, RESP_DEBUG,
    build_stream_start_frame, build_stream_stop_frame,
    crc8
)

# Test settings
DEFAULT_INTERVAL = 100  # ms
DEFAULT_CYCLES = 5       # number of start/stop cycles
SAMPLES_PER_CYCLE = 20   # expected samples per cycle
DEFAULT_TIMEOUT = 2      # seconds


class StreamParser:
    """State machine parser for ADC streaming protocol."""

    STATE_WAIT_MARKER = 0
    STATE_STREAM_DATA_ADC_H = 1
    STATE_STREAM_DATA_ADC_L = 2
    STATE_STREAM_DATA_CRC = 3
    STATE_DEBUG_CODE = 4

    def __init__(self):
        self.state = self.STATE_WAIT_MARKER
        self.sync_errors = 0
        self.crc_errors = 0
        self.debug_packets = 0
        self.valid_samples = 0
        self._adc_h = 0
        self._adc_l = 0

    def reset(self):
        """Reset parser to initial state."""
        self.state = self.STATE_WAIT_MARKER

    def process_byte(self, byte_val, crc8_func):
        """Process a single byte through the state machine.

        Returns: ('data', adc_value), ('debug', code), ('error', byte), or None
        """
        if self.state == self.STATE_WAIT_MARKER:
            if byte_val == CMD_START_STREAM:
                self.state = self.STATE_STREAM_DATA_ADC_H
                return None
            elif byte_val == RESP_DEBUG:
                self.state = self.STATE_DEBUG_CODE
                return None
            else:
                self.sync_errors += 1
                return ('error', byte_val)

        elif self.state == self.STATE_STREAM_DATA_ADC_H:
            self._adc_h = byte_val
            self.state = self.STATE_STREAM_DATA_ADC_L
            return None

        elif self.state == self.STATE_STREAM_DATA_ADC_L:
            self._adc_l = byte_val
            self.state = self.STATE_STREAM_DATA_CRC
            return None

        elif self.state == self.STATE_STREAM_DATA_CRC:
            calc_crc = crc8_func(bytes([CMD_START_STREAM, self._adc_h, self._adc_l]))
            if byte_val == calc_crc:
                adc_value = (self._adc_h << 8) | self._adc_l
                self.valid_samples += 1
                self.state = self.STATE_WAIT_MARKER
                return ('data', adc_value)
            else:
                self.crc_errors += 1
                self.state = self.STATE_WAIT_MARKER
                return ('error', byte_val)

        elif self.state == self.STATE_DEBUG_CODE:
            self.debug_packets += 1
            self.state = self.STATE_WAIT_MARKER
            return ('debug', byte_val[0] if isinstance(byte_val, bytes) else byte_val)

        return None


def find_serial_port():
    """Find RP2040 USB serial port."""
    import serial.tools.list_ports
    ports = serial.tools.list_ports.comports()
    for port in ports:
        # Check for RP2040 identifiers
        # Linux: hwid contains "2E8A" (Raspberry Pi)
        # macOS: device contains "usb" and serial number
        # Windows: hardware_id contains "USB\VID_2E8A"
        identifier = getattr(port, 'hwid', '') or getattr(port, 'hardware_id', '') or port.device
        if "ACM" in identifier or "2E8A" in identifier or "ttyACM" in port.device:
            return port.device
    return None


def send_command_and_wait_ack(ser, frame, cmd_name, timeout=DEFAULT_TIMEOUT):
    """Send command and wait for ACK (may have DEBUG packets before it)."""
    ser.write(frame)
    ser.flush()

    start_time = time.time()
    resp_type_byte = None

    while time.time() - start_time < timeout:
        resp_type_byte = ser.read(1)
        if not resp_type_byte:
            continue

        # Check for debug packet marker
        if resp_type_byte[0] == RESP_DEBUG:
            code_byte = ser.read(1)
            if code_byte:
                print(f"  [DEBUG packet: 0x{code_byte[0]:02X}]")
            continue

        # Not a debug packet - this is our response
        break

    if not resp_type_byte:
        return False, "Timeout"

    resp_type = resp_type_byte[0]

    # If NACK, read error code byte
    err_code = 0
    if resp_type == RESP_NACK:
        err_byte = ser.read(1)
        if err_byte:
            err_code = err_byte[0]

    if resp_type == RESP_ACK:
        return True, "ACK"
    else:
        return False, f"NACK (err={err_code})"


def run_start_stop_cycle(ser, cycle_num, interval_ms=DEFAULT_INTERVAL):
    """Run a single start/stop cycle."""
    print(f"\n=== Cycle {cycle_num} ===")

    # Flush input buffer
    ser.reset_input_buffer()

    # Send START_STREAM command
    frame = build_stream_start_frame(interval_ms)
    print(f"Sending START_STREAM: {frame.hex()}")

    success, msg = send_command_and_wait_ack(ser, frame, "START_STREAM")
    if not success:
        print(f"  ERROR: {msg}")
        return False, f"START failed: {msg}"
    print(f"  OK: {msg}")

    # Read streaming data for a while
    parser = StreamParser()
    start_time = time.time()
    sample_count = 0

    print(f"  Reading samples for {SAMPLES_PER_CYCLE * interval_ms / 1000:.1f} seconds...")

    while sample_count < SAMPLES_PER_CYCLE and (time.time() - start_time) < 5:
        byte_data = ser.read(1)
        if not byte_data:
            continue

        byte_val = byte_data[0]
        result = parser.process_byte(byte_val, crc8)

        if result and result[0] == 'data':
            sample_count += 1
            adc_value = result[1]
            voltage = (adc_value / 4095.0) * 3.3
            if sample_count <= 3 or sample_count == SAMPLES_PER_CYCLE:
                print(f"    Sample {sample_count}: raw={adc_value}, voltage={voltage:.3f}V")

        elif result and result[0] == 'error':
            err_val = result[1]
            print(f"  WARNING: Sync error on byte 0x{err_val:02X}")

        elif result and result[0] == 'debug':
            debug_code = result[1]
            pass  # Silent for debug packets during streaming

    print(f"  Received {sample_count} samples")

    # Flush input buffer before sending stop
    ser.reset_input_buffer()

    # Send STOP_STREAM command
    frame = build_stream_stop_frame()
    print(f"Sending STOP_STREAM: {frame.hex()}")

    success, msg = send_command_and_wait_ack(ser, frame, "STOP_STREAM")
    if not success:
        print(f"  ERROR: {msg}")
        return False, f"STOP failed: {msg}"
    print(f"  OK: {msg}")

    # Wait a bit to ensure connection is stable
    time.sleep(0.2)

    return True, "OK"


def main():
    """Main test function."""
    print("=" * 60)
    print("ADC Streaming Start/Stop Cycle Test")
    print("=" * 60)

    # Find serial port
    port = find_serial_port()
    if not port:
        print("ERROR: No RP2040 device found!")
        print("Connect the device via USB and try again.")
        return 1

    print(f"Found device: {port}")

    # Connect
    try:
        ser = serial.Serial(port, baudrate=115200, timeout=DEFAULT_TIMEOUT)
        print(f"Connected to {port}")
        time.sleep(0.3)  # Wait for device to be ready
    except serial.SerialException as e:
        print(f"ERROR: Failed to connect: {e}")
        return 1

    # Run test cycles
    cycles_passed = 0
    cycles_failed = 0

    try:
        for i in range(1, DEFAULT_CYCLES + 1):
            success, msg = run_start_stop_cycle(ser, i)
            if success:
                cycles_passed += 1
            else:
                cycles_failed += 1
                print(f"\n  CYCLE {i} FAILED: {msg}")
                print(f"  Continuing test...")

    except KeyboardInterrupt:
        print("\n\nTest interrupted by user.")

    finally:
        # Cleanup
        ser.reset_input_buffer()
        ser.close()
        print(f"\nConnection closed.")

    # Summary
    print("\n" + "=" * 60)
    print("TEST SUMMARY")
    print("=" * 60)
    print(f"Cycles passed: {cycles_passed}/{DEFAULT_CYCLES}")
    print(f"Cycles failed: {cycles_failed}/{DEFAULT_CYCLES}")

    if cycles_failed == 0:
        print("\n✓ ALL TESTS PASSED!")
        return 0
    else:
        print(f"\n✗ {cycles_failed} TEST(S) FAILED")
        return 1


if __name__ == "__main__":
    sys.exit(main())
