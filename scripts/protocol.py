#!/usr/bin/env python3
"""
RP2040 LED Control - Protocol Module

Shared protocol implementation for GUI and CLI tools.
Implements binary UART protocol with CRC-8.

Protocol:
    Standard frame: [CMD][VALUE][CRC8] - 3 bytes
    ADC frame: [CMD][CRC8] - 2 bytes

    CMD 0x01 = SET LED (0=OFF, 1+=ON) - Legacy ON/OFF
    CMD 0x02 = SET PWM DUTY (0-100, 0%=OFF, 100%=full)
    CMD 0x03 = READ ADC - Request ADC reading, response: [0x03][ADC_H][ADC_L][CRC8]

    Response: ACK 0xFF or NACK 0xFE + error code
    ADC response: 12-bit value (0-4095) representing 0-3.3V

    CRC-8: Polynomial 0x07, Initial 0x00
"""

from typing import Tuple

# Protocol constants
CMD_SET_LED = 0x01
CMD_SET_PWM = 0x02
CMD_READ_ADC = 0x03

RESP_ACK = 0xFF
RESP_NACK = 0xFE

ERR_CRC = 0x01
ERR_INVALID_CMD = 0x02
ERR_INVALID_VAL = 0x03


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
    }
    return errors.get(error_code, f"Unknown error {error_code}")
