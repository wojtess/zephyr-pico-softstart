#!/usr/bin/env python3
"""
RP2040 ADC Streaming Test Script

Tests continuous ADC streaming functionality.
Sends START_STREAM command, receives ADC samples, displays statistics.
Supports Ctrl+C to stop streaming gracefully.

Usage:
    python3 scripts/test_adc_stream.py [interval_ms]

Arguments:
    interval_ms: Sampling interval in milliseconds (default: 100)
"""

import sys
import serial
import time
from protocol import (
    CMD_START_STREAM, CMD_STOP_STREAM, RESP_DEBUG,
    build_stream_start_frame, build_stream_stop_frame,
    parse_stream_data, parse_response, get_error_name, get_debug_name
)

# Default settings
DEFAULT_INTERVAL = 100  # ms
DEFAULT_TIMEOUT = 1.0   # seconds
BAUDRATE = 115200


def find_acm_port():
    """Find USB CDC ACM port (Linux, macOS, Windows)"""
    import glob
    import platform

    system = platform.system()

    if system == "Linux":
        ports = glob.glob("/dev/ttyACM*")
    elif system == "Darwin":  # macOS
        ports = glob.glob("/dev/cu.usbmodem*")
    else:  # Windows
        import serial.tools.list_ports
        ports = [
            p.device for p in serial.tools.list_ports.comports()
            if "ACM" in p.description or "USB Serial" in p.description
        ]

    if ports:
        return ports[0]
    return None


class StreamParser:
    """State machine parser for ADC streaming protocol with error recovery."""

    # Parser states
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

    def reset(self):
        """Reset parser to initial state."""
        self.state = self.STATE_WAIT_MARKER

    def process_byte(self, byte_val):
        """Process a single byte through the state machine.

        Args:
            byte_val: Integer byte value (0-255)

        Returns:
            Tuple of (result, value, error_code):
                - result: 'data', 'debug', 'error', or None
                - value: ADC value (0-4095) for 'data', debug code for 'debug', 0 for 'error'
                - error_code: Error code for errors
        """
        from protocol import crc8, ERR_CRC

        if self.state == self.STATE_WAIT_MARKER:
            # Looking for packet type marker
            if byte_val == CMD_START_STREAM:
                # Stream data marker: [0x04][ADC_H][ADC_L][CRC]
                self.state = self.STATE_STREAM_DATA_ADC_H
                return None, 0, 0
            elif byte_val == RESP_DEBUG:
                # Debug marker: [0xFD][code]
                self.state = self.STATE_DEBUG_CODE
                return None, 0, 0
            else:
                # Unexpected byte - synchronization error
                self.sync_errors += 1
                return 'error', byte_val, 0

        elif self.state == self.STATE_STREAM_DATA_ADC_H:
            # Store ADC high byte
            self._adc_h = byte_val
            self.state = self.STATE_STREAM_DATA_ADC_L
            return None, 0, 0

        elif self.state == self.STATE_STREAM_DATA_ADC_L:
            # Store ADC low byte
            self._adc_l = byte_val
            self.state = self.STATE_STREAM_DATA_CRC
            return None, 0, 0

        elif self.state == self.STATE_STREAM_DATA_CRC:
            # Verify CRC and return data
            calc_crc = crc8(bytes([CMD_START_STREAM, self._adc_h, self._adc_l]))
            if byte_val == calc_crc:
                adc_value = (self._adc_h << 8) | self._adc_l
                self.valid_samples += 1
                self.state = self.STATE_WAIT_MARKER
                return 'data', adc_value, 0
            else:
                self.crc_errors += 1
                self.state = self.STATE_WAIT_MARKER
                return 'error', 0, ERR_CRC

        elif self.state == self.STATE_DEBUG_CODE:
            # Debug packet complete
            self.debug_packets += 1
            self.state = self.STATE_WAIT_MARKER
            return 'debug', byte_val, 0

        return None, 0, 0


def test_adc_streaming(ser, interval_ms):
    """Test ADC streaming

    Args:
        ser: Open serial port
        interval_ms: Sampling interval in milliseconds
    """
    print(f"\n{'='*60}")
    print(f"ADC Streaming Test")
    print(f"{'='*60}")
    print(f"Port: {ser.port}")
    print(f"Interval: {interval_ms} ms")
    print(f"Expected rate: {1000/interval_ms:.1f} samples/sec")
    print(f"Press Ctrl+C to stop...")
    print(f"{'='*60}\n")

    # Flush input buffer
    ser.reset_input_buffer()

    # Send START_STREAM command
    frame = build_stream_start_frame(interval_ms)
    print(f"Sending START_STREAM command: {frame.hex()}")
    ser.write(frame)

    # Wait for ACK (may have DEBUG packets before it)
    ser.timeout = DEFAULT_TIMEOUT
    resp_type = 0
    err = 0

    # Keep reading until we get ACK or NACK
    while resp_type not in (0xFF, 0xFE):
        first = ser.read(1)
        if not first:
            print("ERROR: Timeout waiting for response")
            return

        if first[0] == RESP_DEBUG:
            code_byte = ser.read(1)
            # Debug pakiet - konsumujemy bez wyświetlania
            if len(code_byte) == 1:
                pass  # Opcjonalnie: print(f"  [DEBUG] {get_debug_name(code_byte[0])}")
            continue

        resp = first + ser.read(1)
        resp_type, err = parse_response(resp)

    if resp_type != 0xFF:  # Not ACK
        print(f"ERROR: Failed to start streaming: {get_error_name(err)}")
        return

    print("Streaming started! Receiving data...\n")
    print(f"{'Sample':>8} | {'ADC':>6} | {'Voltage':>10} | {'Hex':>8}")
    print(f"{'-'*8}-+-{'-'*6}-+-{'-'*10}-+-{'-'*8}")

    count = 0
    total_adc = 0
    start_time = time.time()

    # Initialize state machine parser
    parser = StreamParser()

    try:
        while True:
            # Read one byte at a time for proper state machine processing
            byte_data = ser.read(1)
            if not byte_data:
                time.sleep(0.001)
                continue

            byte_val = byte_data[0]
            result, value, err_code = parser.process_byte(byte_val)

            if result == 'data':
                # Valid ADC sample received
                count += 1
                total_adc += value
                voltage = (value / 4095) * 3.3

                if count % 10 == 0:
                    # Reconstruct packet for display
                    pkt_hex = bytes([CMD_START_STREAM, (value >> 8) & 0xFF, value & 0xFF]).hex()
                    print(f"{count:>8} | {value:>6} | {voltage:>9.3f}V | {pkt_hex:>8}")

            elif result == 'error':
                # Synchronization or CRC error
                if err_code:
                    print(f"ERROR: CRC error detected at sample {count}")
                else:
                    print(f"ERROR: Synchronization error - unexpected byte 0x{value:02X}")
                    # After sync error, we're already resynchronized (parser reset automatically)

            elif result == 'debug':
                # Debug packet - silently consumed (counted in parser stats)
                pass

            time.sleep(0.001)

    except KeyboardInterrupt:
        pass
    finally:
        elapsed = time.time() - start_time
        rate = count / elapsed if elapsed > 0 else 0

        print(f"\n{'-'*8}-+-{'-'*6}-+-{'-'*10}-+-{'-'*8}")
        print(f"\nStatistics:")
        print(f"  Valid samples: {count}")
        print(f"  Time elapsed: {elapsed:.2f} seconds")
        print(f"  Actual rate: {rate:.2f} samples/sec")
        print(f"  Debug packets: {parser.debug_packets}")
        print(f"  Sync errors: {parser.sync_errors}")
        print(f"  CRC errors: {parser.crc_errors}")
        if count > 0:
            print(f"  Average ADC: {total_adc / count:.1f}")
            print(f"  Avg voltage: {(total_adc / count / 4095 * 3.3):.3f}V")

        # Send STOP_STREAM command
        print(f"\nStopping stream...")
        frame = build_stream_stop_frame()
        ser.write(frame)

        # Wait for ACK (may have DEBUG packets before it)
        resp_type = 0
        err = 0

        while resp_type not in (0xFF, 0xFE):
            first = ser.read(1)
            if not first:
                break

            if first[0] == RESP_DEBUG:
                code_byte = ser.read(1)
                # Debug pakiet - konsumujemy bez wyświetlania
                if len(code_byte) == 1:
                    pass
                continue

            resp = first + ser.read(1)
            resp_type, err = parse_response(resp)

        if resp_type == 0xFF:
            print("Stream stopped successfully!")
        else:
            print(f"WARNING: Stop failed: {get_error_name(err)}")


def main():
    interval = int(sys.argv[1]) if len(sys.argv) > 1 else DEFAULT_INTERVAL

    if not (10 <= interval <= 60000):
        print(f"ERROR: Interval must be 10-60000 ms")
        sys.exit(1)

    # Find port
    port = find_acm_port()
    if not port:
        print("ERROR: No USB CDC ACM port found")
        print("Connect the device and try again")
        sys.exit(1)

    print(f"Found port: {port}")

    try:
        ser = serial.Serial(port, BAUDRATE, timeout=DEFAULT_TIMEOUT)
        time.sleep(0.1)  # Wait for serial to be ready

        test_adc_streaming(ser, interval)

        ser.close()
        print("\nDone.")

    except serial.SerialException as e:
        print(f"ERROR: Serial port: {e}")
        sys.exit(1)


if __name__ == "__main__":
    main()
