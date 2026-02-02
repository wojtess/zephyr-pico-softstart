#!/usr/bin/env python3
"""
RP2040 LED Control - Protocol Module

Shared protocol implementation for GUI and CLI tools.
Implements binary UART protocol with CRC-8.

Protocol:
    Standard frame: [CMD][VALUE][CRC8] - 3 bytes
    ADC frame: [CMD][CRC8] - 2 bytes
    Stream start: [CMD][INT_H][INT_L][CRC8] - 4 bytes
    Stream stop: [CMD][CRC8] - 2 bytes

    CMD 0x01 = SET LED (0=OFF, 1+=ON) - Legacy ON/OFF
    CMD 0x02 = SET PWM DUTY (0-100, 0%=OFF, 100%=full)
    CMD 0x03 = READ ADC - Request ADC reading, response: [0x03][ADC_H][ADC_L][CRC8]
    CMD 0x04 = START ADC STREAMING - Start periodic ADC sampling
    CMD 0x05 = STOP ADC STREAMING - Stop periodic ADC sampling

    Response: ACK 0xFF or NACK 0xFE + error code
    ADC response: 12-bit value (0-4095) representing 0-3.3V

    CRC-8: Polynomial 0x07, Initial 0x00
"""

from typing import Tuple, Dict

# Protocol constants
CMD_SET_LED = 0x01
CMD_SET_PWM = 0x02
CMD_READ_ADC = 0x03
CMD_START_STREAM = 0x04
CMD_STOP_STREAM = 0x05
CMD_SET_MODE = 0x06  # Set mode (0=manual, 1=P-control)
CMD_GET_P_STATUS = 0x0B  # Get PI-controller status
CMD_SET_P_SETPOINT = 0x07  # Set PI-controller setpoint (0-4095)
CMD_SET_P_GAIN = 0x08  # Set PI-controller gain (float value sent as int scaled)
CMD_SET_P_KI = 0x0D  # Set PI-controller Ki (integral gain, float value sent as int scaled)
CMD_START_P_STREAM = 0x09  # Start PI-controller streaming
CMD_STOP_P_STREAM = 0x0A  # Stop PI-controller streaming
CMD_SET_FEED_FORWARD = 0x0C  # Set feed-forward PWM (0-100)
CMD_SET_FILTER_ALPHA = 0x0E  # Set IIR filter alpha (numerator/denominator)
CMD_SET_FILTER_MODE = 0x0F  # Set ADC filter mode (0=IIR, 1=OS, 2=OS+IIR)

RESP_ACK = 0xFF
RESP_NACK = 0xFE
RESP_DEBUG = 0xFD

ERR_CRC = 0x01
ERR_INVALID_CMD = 0x02
ERR_INVALID_VAL = 0x03
ERR_INVALID_MODE = 0x10
ERR_INVALID_SETPOINT = 0x11
ERR_INVALID_GAIN = 0x12
ERR_INVALID_FF = 0x13
ERR_INVALID_KI = 0x14  # Invalid integral gain (Ki)


def crc8(data: bytes) -> int:
    """Calculate CRC-8 (Polynomial 0x07, Initial 0x00)

    Args:
        data: Input bytes to calculate CRC

    Returns:
        CRC-8 checksum (0-255)
    """
    crc = 0x00
    for byte in data:
        crc ^= byte
        for _ in range(8):
            if crc & 0x80:
                crc = (crc << 1) ^ 0x07
            else:
                crc <<= 1
    return crc & 0xFF


def build_frame(cmd: int, value: int) -> bytes:
    """Build a standard 3-byte protocol frame with CRC

    Args:
        cmd: Command byte (e.g., CMD_SET_LED, CMD_SET_PWM)
        value: Value byte (0-255)

    Returns:
        Complete 3-byte frame: [CMD][VALUE][CRC8]
    """
    cmd_value = bytes([cmd, value])
    checksum = crc8(cmd_value)
    return cmd_value + bytes([checksum])


def build_adc_frame() -> bytes:
    """Build an ADC read request frame (2-byte frame)

    Frame format: [CMD_READ_ADC][CRC8]

    Returns:
        Complete 2-byte frame: [0x03][CRC8]
    """
    cmd_byte = bytes([CMD_READ_ADC])
    checksum = crc8(cmd_byte)
    return cmd_byte + bytes([checksum])


def parse_response(resp: bytes) -> Tuple[int, int]:
    """Parse standard ACK/NACK response from device

    Args:
        resp: Response bytes (1 or 2 bytes)

    Returns:
        Tuple of (response_type, error_code)
        - response_type: RESP_ACK or RESP_NACK
        - error_code: 0 for ACK, error code for NACK
    """
    if not resp:
        return RESP_NACK, 0

    resp_type = resp[0]

    if resp_type == RESP_ACK:
        return RESP_ACK, 0
    elif resp_type == RESP_NACK:
        error_code = resp[1] if len(resp) > 1 else 0
        return RESP_NACK, error_code

    return RESP_NACK, 0


def parse_adc_response(resp: bytes) -> Tuple[int, int]:
    """Parse ADC response from device

    Response format: [CMD_READ_ADC][ADC_H][ADC_L][CRC8]

    Args:
        resp: Response bytes (4 bytes)

    Returns:
        Tuple of (adc_value, error_code)
        - adc_value: 12-bit ADC value (0-4095), or -1 on error
        - error_code: 0 for success, error code otherwise
    """
    if not resp or len(resp) < 4:
        return -1, 0

    # Verify response type
    if resp[0] != CMD_READ_ADC:
        return -1, ERR_INVALID_CMD

    # Verify CRC
    calculated_crc = crc8(resp[:3])
    if resp[3] != calculated_crc:
        return -1, ERR_CRC

    # Parse ADC value (12-bit, big-endian)
    adc_value = (resp[1] << 8) | resp[2]
    return adc_value, 0


def get_error_name(error_code: int) -> str:
    """Get human-readable error name from error code

    Args:
        error_code: Error code from NACK response

    Returns:
        Human-readable error description
    """
    errors = {
        ERR_CRC: "CRC error",
        ERR_INVALID_CMD: "Invalid command",
        ERR_INVALID_VAL: "Invalid value",
        ERR_INVALID_MODE: "Invalid mode",
        ERR_INVALID_SETPOINT: "Invalid setpoint",
        ERR_INVALID_GAIN: "Invalid gain",
        ERR_INVALID_FF: "Invalid feed-forward",
    }
    return errors.get(error_code, f"Unknown error {error_code}")


# Debug code names for ADC streaming
DEBUG_NAMES = {
    1: "TIMER_START",
    2: "TIMER_CB",
    3: "ADC_READ",
    4: "ADC_ERROR",
    5: "TX_PUT",
}


def get_debug_name(debug_code: int) -> str:
    """Get human-readable debug name from debug code

    Args:
        debug_code: Debug code from DEBUG response

    Returns:
        Human-readable debug description
    """
    return DEBUG_NAMES.get(debug_code, f"DEBUG_{debug_code}")


def build_stream_start_frame(interval_ms: int) -> bytes:
    """Build a stream start request frame (4-byte frame)

    Frame format: [CMD_START_STREAM][INTERVAL_H][INTERVAL_L][CRC8]

    Args:
        interval_ms: Sampling interval in milliseconds (10-60000)

    Returns:
        Complete 4-byte frame
    """
    if not (10 <= interval_ms <= 60000):
        raise ValueError(f"Interval must be 10-60000 ms, got {interval_ms}")

    interval_h = (interval_ms >> 8) & 0xFF
    interval_l = interval_ms & 0xFF
    cmd_interval = bytes([CMD_START_STREAM, interval_h, interval_l])
    checksum = crc8(cmd_interval)
    return cmd_interval + bytes([checksum])


def build_stream_stop_frame() -> bytes:
    """Build a stream stop request frame (2-byte frame)

    Frame format: [CMD_STOP_STREAM][CRC8]

    Returns:
        Complete 2-byte frame
    """
    cmd_byte = bytes([CMD_STOP_STREAM])
    checksum = crc8(cmd_byte)
    return cmd_byte + bytes([checksum])


def parse_stream_data(resp: bytes) -> Tuple[int, int]:
    """Parse ADC streaming data response from device

    Response format: [CMD_START_STREAM][ADC_H][ADC_L][CRC8]
    (Note: Uses CMD_START_STREAM as data marker)

    Args:
        resp: Response bytes (4 bytes)

    Returns:
        Tuple of (adc_value, error_code)
        - adc_value: 12-bit ADC value (0-4095), or -1 on error
        - error_code: 0 for success, error code otherwise
    """
    if not resp or len(resp) < 4:
        return -1, 0

    # Verify response type (should be CMD_START_STREAM as data marker)
    if resp[0] != CMD_START_STREAM:
        return -1, ERR_INVALID_CMD

    # Verify CRC
    calculated_crc = crc8(resp[:3])
    if resp[3] != calculated_crc:
        return -1, ERR_CRC

    # Parse ADC value (12-bit, big-endian)
    adc_value = (resp[1] << 8) | resp[2]
    return adc_value, 0


def parse_p_stream_data(resp: bytes) -> Tuple[int, int, int, int]:
    """Parse PI-controller streaming data response from device

    Response format: [CMD_START_P_STREAM][SP_H][SP_L][MEAS_H][MEAS_L][PWM][CRC8]
    - SP_H, SP_L: Setpoint (16-bit, 0-4095)
    - MEAS_H, MEAS_L: Measured ADC value (16-bit, 0-4095)
    - PWM: PWM output (0-100)
    - CRC8: Checksum

    Args:
        resp: Response bytes (7 bytes)

    Returns:
        Tuple of (setpoint, measured, pwm, error_code)
        - setpoint: Setpoint value (0-4095), or -1 on error
        - measured: Measured ADC value (0-4095), or -1 on error
        - pwm: PWM output (0-100), or -1 on error
        - error_code: 0 for success, error code otherwise
    """
    if not resp or len(resp) < 7:
        return -1, -1, -1, 0

    # Verify response type (should be CMD_START_P_STREAM as data marker)
    if resp[0] != CMD_START_P_STREAM:
        return -1, -1, -1, ERR_INVALID_CMD

    # Verify CRC
    calculated_crc = crc8(resp[:6])
    if resp[6] != calculated_crc:
        return -1, -1, -1, ERR_CRC

    # Parse values (16-bit big-endian for setpoint and measured, 8-bit for PWM)
    setpoint = (resp[1] << 8) | resp[2]
    measured = (resp[3] << 8) | resp[4]
    pwm = resp[5]
    return setpoint, measured, pwm, 0


def build_set_mode_frame(mode: int) -> bytes:
    """Build a set mode frame (3-byte frame)

    Frame format: [CMD_SET_MODE][MODE][CRC8]

    Args:
        mode: Mode (0=manual, 1=p-control)

    Returns:
        Complete 3-byte frame
    """
    return build_frame(CMD_SET_MODE, mode)


def build_set_p_setpoint_frame(setpoint: int) -> bytes:
    """Build a set P setpoint frame (4-byte frame)

    Frame format: [CMD_SET_P_SETPOINT][SP_H][SP_L][CRC8]

    Args:
        setpoint: Setpoint value (0-4095)

    Returns:
        Complete 4-byte frame
    """
    sp_h = (setpoint >> 8) & 0xFF
    sp_l = setpoint & 0xFF
    cmd_sp = bytes([CMD_SET_P_SETPOINT, sp_h, sp_l])
    checksum = crc8(cmd_sp)
    return cmd_sp + bytes([checksum])


def build_set_p_gain_frame(gain: float) -> bytes:
    """Build a set P gain frame (4-byte frame)

    Gain is sent as integer value * 100 (e.g., 1.5 -> 150) in 16-bit H/L format

    Frame format: [CMD_SET_P_GAIN][GAIN_H][GAIN_L][CRC8]

    Args:
        gain: P gain value (0.0-10.0)

    Returns:
        Complete 4-byte frame
    """
    gain_int = int(gain * 100)
    gain_int = max(0, min(1000, gain_int))  # Clamp to valid range
    gain_h = (gain_int >> 8) & 0xFF
    gain_l = gain_int & 0xFF
    cmd_gain = bytes([CMD_SET_P_GAIN, gain_h, gain_l])
    checksum = crc8(cmd_gain)
    return cmd_gain + bytes([checksum])


def build_set_p_ki_frame(ki: float) -> bytes:
    """Build a set P Ki frame (4-byte frame)

    Ki is sent as integer value * 100 (e.g., 1.5 -> 150) in 16-bit H/L format

    Frame format: [CMD_SET_P_KI][KI_H][KI_L][CRC8]

    Args:
        ki: P integral gain value (0.0-10.0)

    Returns:
        Complete 4-byte frame
    """
    ki_int = int(ki * 100)
    ki_int = max(0, min(1000, ki_int))  # Clamp to valid range
    ki_h = (ki_int >> 8) & 0xFF
    ki_l = ki_int & 0xFF
    cmd_ki = bytes([CMD_SET_P_KI, ki_h, ki_l])
    checksum = crc8(cmd_ki)
    return cmd_ki + bytes([checksum])


def build_set_feed_forward_frame(pwm: int) -> bytes:
    """Build a set feed-forward frame (3-byte frame)

    Frame format: [CMD_SET_FEED_FORWARD][PWM][CRC8]

    Args:
        pwm: Feed-forward PWM value (0-100)

    Returns:
        Complete 3-byte frame
    """
    return build_frame(CMD_SET_FEED_FORWARD, pwm)


def build_p_stream_start_frame(interval_ms: int) -> bytes:
    """Build a P-stream start request frame (4-byte frame)

    Frame format: [CMD_START_P_STREAM][INTERVAL_H][INTERVAL_L][CRC8]

    Args:
        interval_ms: Sampling interval in milliseconds (10-60000)

    Returns:
        Complete 4-byte frame
    """
    if not (10 <= interval_ms <= 60000):
        raise ValueError(f"Interval must be 10-60000 ms, got {interval_ms}")

    interval_h = (interval_ms >> 8) & 0xFF
    interval_l = interval_ms & 0xFF
    cmd_interval = bytes([CMD_START_P_STREAM, interval_h, interval_l])
    checksum = crc8(cmd_interval)
    return cmd_interval + bytes([checksum])


def build_p_stream_stop_frame() -> bytes:
    """Build a P-stream stop request frame (2-byte frame)

    Frame format: [CMD_STOP_P_STREAM][CRC8]

    Returns:
        Complete 2-byte frame
    """
    cmd_byte = bytes([CMD_STOP_P_STREAM])
    checksum = crc8(cmd_byte)
    return cmd_byte + bytes([checksum])


def build_get_p_status_frame() -> bytes:
    """Build a get PI-controller status frame (2-byte frame)

    Frame format: [CMD_GET_P_STATUS][CRC8]

    Returns:
        Complete 2-byte frame
    """
    cmd_byte = bytes([CMD_GET_P_STATUS])
    checksum = crc8(cmd_byte)
    return cmd_byte + bytes([checksum])


def build_set_filter_alpha_frame(numerator: int, denominator: int) -> bytes:
    """Build a set filter alpha frame (4-byte frame)

    Frame format: [CMD_SET_FILTER_ALPHA][NUM][DEN][CRC8]

    Args:
        numerator: Alpha numerator (1-255)
        denominator: Alpha denominator (1-255, power of 2 recommended)

    Returns:
        Complete 4-byte frame
    """
    if not (1 <= numerator <= 255):
        raise ValueError(f"Numerator must be 1-255, got {numerator}")
    if not (1 <= denominator <= 255):
        raise ValueError(f"Denominator must be 1-255, got {denominator}")
    if numerator > denominator:
        raise ValueError(f"Numerator ({numerator}) must be <= denominator ({denominator})")

    cmd_num_den = bytes([CMD_SET_FILTER_ALPHA, numerator, denominator])
    checksum = crc8(cmd_num_den)
    return cmd_num_den + bytes([checksum])


def build_set_filter_mode_frame(mode: int) -> bytes:
    """Build a set filter mode frame (3-byte frame)

    Frame format: [CMD_SET_FILTER_MODE][MODE][CRC8]

    Args:
        mode: Filter mode (0=IIR only, 1=Oversample only, 2=Oversample+IIR)

    Returns:
        Complete 3-byte frame
    """
    if mode not in [0, 1, 2]:
        raise ValueError(f"Invalid filter mode: {mode} (must be 0-2)")

    return build_frame(CMD_SET_FILTER_MODE, mode)


def parse_p_status_response(resp: bytes) -> Tuple[int, int, int, int]:
    """Parse PI-controller status response from device

    Response format: [CMD_GET_P_STATUS][MODE][SP_H][SP_L][PWM][CRC8]
    - MODE: Operation mode (0=manual, 1=auto)
    - SP_H, SP_L: Setpoint (16-bit, 0-4095)
    - PWM: Current PWM output (0-100)
    - CRC8: Checksum

    Args:
        resp: Response bytes (6 bytes)

    Returns:
        Tuple of (mode, setpoint, pwm, error_code)
        - mode: Operation mode (0=manual, 1=auto), or -1 on error
        - setpoint: Setpoint value (0-4095), or -1 on error
        - pwm: PWM output (0-100), or -1 on error
        - error_code: 0 for success, error code otherwise
    """
    if not resp or len(resp) < 6:
        return -1, -1, -1, 0

    # Verify response type
    if resp[0] != CMD_GET_P_STATUS:
        return -1, -1, -1, ERR_INVALID_CMD

    # Verify CRC
    calculated_crc = crc8(resp[:5])
    if resp[5] != calculated_crc:
        return -1, -1, -1, ERR_CRC

    # Parse values
    mode = resp[1]
    setpoint = (resp[2] << 8) | resp[3]
    pwm = resp[4]
    return mode, setpoint, pwm, 0
