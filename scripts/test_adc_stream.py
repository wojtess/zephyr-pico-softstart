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

    try:
        while True:
            # Read one byte to determine packet type
            first = ser.read(1)
            if not first:
                time.sleep(0.01)
                continue

            pkt_type = first[0]

            # Debug packet: [0xFD][code] - konsumujemy bez wyświetlania
            if pkt_type == RESP_DEBUG:
                code_byte = ser.read(1)
                if len(code_byte) == 1:
                    pass  # Opcjonalnie: print(f"  [DEBUG] {get_debug_name(debug_code)}")
                continue

            # Stream data: [0x04][ADC_H][ADC_L][CRC]
            if pkt_type == CMD_START_STREAM:
                adc_h = ser.read(1)
                adc_l = ser.read(1)
                crc = ser.read(1)
                if len(adc_h) == 1 and len(adc_l) == 1 and len(crc) == 1:
                    resp = bytes([pkt_type]) + adc_h + adc_l + crc
                    adc_value, err = parse_stream_data(resp)

                    if adc_value >= 0:
                        count += 1
                        total_adc += adc_value
                        voltage = (adc_value / 4095) * 3.3

                        if count % 10 == 0:
                            print(f"{count:>8} | {adc_value:>6} | {voltage:>9.3f}V | {resp.hex():>8}")

                    elif err:
                        print(f"Sample {count}: ERROR - {get_error_name(err)}")
                continue

            # Unknown byte - skip and continue
            if pkt_type != 0x00:
                print(f"  Unknown byte: 0x{pkt_type:02X}")

            time.sleep(0.001)

    except KeyboardInterrupt:
        pass
    finally:
        elapsed = time.time() - start_time
        rate = count / elapsed if elapsed > 0 else 0

        print(f"\n{'-'*8}-+-{'-'*6}-+-{'-'*10}-+-{'-'*8}")
        print(f"\nStatistics:")
        print(f"  Total samples: {count}")
        print(f"  Time elapsed: {elapsed:.2f} seconds")
        print(f"  Actual rate: {rate:.2f} samples/sec")
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
