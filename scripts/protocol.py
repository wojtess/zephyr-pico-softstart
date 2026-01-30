#!/usr/bin/env python3
"""
RP2040 LED Control - Protocol Module

Shared protocol implementation for GUI and CLI tools.
Implements binary UART protocol with CRC-8.

Protocol:
    Frame: [CMD][VALUE][CRC8] - 3 bytes
    CMD 0x01 = SET LED (0=OFF, 1+=ON) - Legacy ON/OFF
    CMD 0x02 = SET PWM DUTY (0-100, 0%=OFF, 100%=full)
    Response: ACK 0xFF or NACK 0xFE + error code
    CRC-8: Polynomial 0x07, Initial 0x00
"""

from typing import Tuple

# Protocol constants
CMD_SET_LED = 0x01
CMD_SET_PWM = 0x02

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
    """Build a protocol frame with CRC

    Args:
        cmd: Command byte (e.g., CMD_SET_LED)
        value: Value byte (0-255)

    Returns:
        Complete 3-byte frame: [CMD][VALUE][CRC8]
    """
    cmd_value = bytes([cmd, value])
    checksum = crc8(cmd_value)
    return cmd_value + bytes([checksum])


def parse_response(resp: bytes) -> Tuple[int, int]:
    """Parse response from device

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
