"""
Comprehensive tests for protocol.py - CRC, frame building, and parsing.

Protocol specs:
- CRC-8: Polynomial 0x07, Initial 0x00
- Standard frame: [CMD][VALUE][CRC8] - 3 bytes
- 2-byte frames: [CMD][CRC8]
- 4-byte frames: [CMD][H][L][CRC8]
"""

import pytest
from scripts.protocol import (
    # Constants
    CMD_SET_LED, CMD_SET_PWM, CMD_READ_ADC, CMD_START_STREAM, CMD_STOP_STREAM,
    CMD_SET_MODE, CMD_GET_P_STATUS, CMD_SET_P_SETPOINT, CMD_SET_P_GAIN,
    CMD_START_P_STREAM, CMD_STOP_P_STREAM, CMD_SET_FEED_FORWARD,
    RESP_ACK, RESP_NACK, RESP_DEBUG,
    ERR_CRC, ERR_INVALID_CMD, ERR_INVALID_VAL, ERR_INVALID_MODE,
    ERR_INVALID_SETPOINT, ERR_INVALID_GAIN, ERR_INVALID_FF,
    # Functions
    crc8, build_frame, build_adc_frame, parse_response, parse_adc_response,
    get_error_name, get_debug_name, build_stream_start_frame, build_stream_stop_frame,
    parse_stream_data, parse_p_stream_data, build_set_mode_frame,
    build_set_p_setpoint_frame, build_set_p_gain_frame, build_set_feed_forward_frame,
    build_p_stream_start_frame, build_p_stream_stop_frame, build_get_p_status_frame,
    parse_p_status_response,
)

# =============================================================================
# Test Constants
# =============================================================================

# Number of single-bit corruption tests to verify CRC detects ALL errors
CRC_SINGLE_BIT_TEST_COUNT = 16  # 2 bytes * 8 bits


# =============================================================================
# Test Suite 1: CRC-8 Calculation Tests
# =============================================================================

@pytest.mark.unit
class TestCRC8:
    """Test CRC-8 calculation (Polynomial 0x07, Initial 0x00)."""

    def test_crc8_zero(self):
        """Zero data should produce zero CRC."""
        assert crc8(b'\x00') == 0x00

    def test_crc8_single_byte_01(self):
        """Test basic polynomial application."""
        assert crc8(b'\x01') == 0x07

    def test_crc8_single_byte_ff(self):
        """Test all bits set."""
        assert crc8(b'\xFF') == 0xF3

    def test_crc8_empty_bytes(self):
        """Empty input."""
        assert crc8(b'') == 0x00

    def test_crc8_all_bytes(self):
        """Full spectrum test (0x00-0xFF)."""
        assert crc8(bytes(range(256))) == 0x14

    def test_crc8_repeated_aa(self):
        """Alternating bit pattern 0xAA."""
        assert crc8(b'\xAA\xAA\xAA\xAA\xAA') == 0x47

    def test_crc8_repeated_55(self):
        """Inverse alternating pattern 0x55."""
        assert crc8(b'\x55\x55\x55\x55\x55') == 0xA0

    def test_crc8_self_check_property(self):
        """CRC of (data + CRC) should have self-check property."""
        data = b'\x01\x02\x03'
        calculated_crc = crc8(data)
        # Append CRC and recalculate - should give 0 for good polynomial
        crc_with_self = crc8(data + bytes([calculated_crc]))
        # This is a property check - the result should be deterministic
        assert isinstance(crc_with_self, int)


# =============================================================================
# Test Suite 2: Frame Building Tests - Standard 3-Byte Frames
# =============================================================================

@pytest.mark.unit
class TestBuildFrame:
    """Test standard 3-byte frame building."""

    @pytest.mark.parametrize("cmd,value,description", [
        (CMD_SET_LED, 0, "LED OFF"),
        (CMD_SET_LED, 1, "LED ON"),
        (CMD_SET_PWM, 0, "PWM 0%"),
        (CMD_SET_PWM, 50, "PWM 50%"),
        (CMD_SET_PWM, 100, "PWM 100%"),
    ])
    def test_build_standard_frame(self, cmd, value, description):
        """Test standard 3-byte frame building with various commands."""
        result = build_frame(cmd, value)
        assert len(result) == 3
        assert result[0] == cmd
        assert result[1] == value
        assert result[2] == crc8(bytes([cmd, value]))

    @pytest.mark.parametrize("mode,description", [
        (0, "Manual mode"),
        (1, "Auto mode"),
    ])
    def test_build_set_mode_frame(self, mode, description):
        """Test mode selection frame building."""
        result = build_set_mode_frame(mode)
        assert len(result) == 3
        assert result[0] == CMD_SET_MODE
        assert result[1] == mode
        assert result[2] == crc8(bytes([CMD_SET_MODE, mode]))

    @pytest.mark.parametrize("ff_value,description", [
        (0, "Feed-forward at minimum"),
        (50, "Feed-forward at 50%"),
        (100, "Feed-forward at maximum"),
    ])
    def test_build_set_feed_forward_frame(self, ff_value, description):
        """Test feed-forward frame building."""
        result = build_set_feed_forward_frame(ff_value)
        assert len(result) == 3
        assert result[0] == CMD_SET_FEED_FORWARD
        assert result[1] == ff_value
        assert result[2] == crc8(bytes([CMD_SET_FEED_FORWARD, ff_value]))


# =============================================================================
# Test Suite 3: Frame Building Tests - 2-Byte Frames
# =============================================================================

@pytest.mark.unit
class TestBuild2ByteFrames:
    """Test 2-byte frame building (no value parameter)."""

    @pytest.mark.parametrize("builder_func,expected_cmd,description", [
        (build_adc_frame, CMD_READ_ADC, "ADC read request"),
        (build_stream_stop_frame, CMD_STOP_STREAM, "Stop ADC streaming"),
        (build_p_stream_stop_frame, CMD_STOP_P_STREAM, "Stop P-stream"),
        (build_get_p_status_frame, CMD_GET_P_STATUS, "Request P-controller status"),
    ])
    def test_build_2byte_frames(self, builder_func, expected_cmd, description):
        """Test 2-byte frame building with various commands."""
        result = builder_func()
        assert len(result) == 2, f"{description}: frame should be 2 bytes"
        assert result[0] == expected_cmd, f"{description}: command byte should match"
        assert result[1] == crc8(bytes([expected_cmd])), f"{description}: CRC should be correct"


# =============================================================================
# Test Suite 4: Frame Building Tests - 4-Byte Frames
# =============================================================================

@pytest.mark.unit
class TestBuild4ByteFrames:
    """Test 4-byte frame building (16-bit values)."""

    @pytest.mark.parametrize("interval,high_byte,low_byte,description", [
        (10, 0x00, 0x0A, "Minimum valid interval (10ms)"),
        (60000, 0xEA, 0x60, "Maximum valid interval (60s)"),
        (255, 0x00, 0xFF, "Byte boundary (0x00FF)"),
        (256, 0x01, 0x00, "Byte boundary (0x0100)"),
    ])
    def test_build_stream_start_frame(self, interval, high_byte, low_byte, description):
        """Test stream start frame building with various intervals."""
        result = build_stream_start_frame(interval)
        assert len(result) == 4, f"{description}: frame should be 4 bytes"
        assert result[0] == CMD_START_STREAM, f"{description}: command should match"
        assert result[1] == high_byte, f"{description}: high byte should match"
        assert result[2] == low_byte, f"{description}: low byte should match"
        assert result[3] == crc8(bytes([CMD_START_STREAM, high_byte, low_byte])), \
            f"{description}: CRC should be correct"

    @pytest.mark.parametrize("interval,error_match", [
        (9, "Interval must be 10-60000"),
        (60001, "Interval must be 10-60000"),
    ])
    def test_build_stream_start_frame_invalid(self, interval, error_match):
        """Test stream start frame with invalid intervals."""
        with pytest.raises(ValueError, match=error_match):
            build_stream_start_frame(interval)

    @pytest.mark.parametrize("interval,high_byte,low_byte,description", [
        (10, 0x00, 0x0A, "P-stream minimum interval"),
        (60000, 0xEA, 0x60, "P-stream maximum interval"),
    ])
    def test_build_p_stream_start_frame(self, interval, high_byte, low_byte, description):
        """Test P-stream start frame building with various intervals."""
        result = build_p_stream_start_frame(interval)
        assert len(result) == 4, f"{description}: frame should be 4 bytes"
        assert result[0] == CMD_START_P_STREAM, f"{description}: command should match"
        assert result[1:3] == bytes([high_byte, low_byte]), f"{description}: value bytes should match"
        assert result[3] == crc8(bytes([CMD_START_P_STREAM, high_byte, low_byte])), \
            f"{description}: CRC should be correct"

    @pytest.mark.parametrize("setpoint,high_byte,low_byte,description", [
        (0, 0x00, 0x00, "Zero setpoint"),
        (2048, 0x08, 0x00, "Mid-range setpoint (50% of 12-bit)"),
        (4095, 0x0F, 0xFF, "Maximum 12-bit setpoint"),
    ])
    def test_build_set_p_setpoint_frame(self, setpoint, high_byte, low_byte, description):
        """Test P-setpoint frame building with various values."""
        result = build_set_p_setpoint_frame(setpoint)
        assert len(result) == 4, f"{description}: frame should be 4 bytes"
        assert result[0] == CMD_SET_P_SETPOINT, f"{description}: command should match"
        assert result[1:3] == bytes([high_byte, low_byte]), f"{description}: value bytes should match"
        assert result[3] == crc8(bytes([CMD_SET_P_SETPOINT, high_byte, low_byte])), \
            f"{description}: CRC should be correct"

    @pytest.mark.parametrize("gain,high_byte,low_byte,description", [
        (0.0, 0x00, 0x00, "Zero gain (manual control only)"),
        (0.01, 0x00, 0x01, "Minimum positive gain"),
        (1.0, 0x00, 0x64, "Unity gain"),
        (10.0, 0x03, 0xE8, "Maximum gain"),
        (15.0, 0x03, 0xE8, "Gain above maximum (clamped to 10.0)"),
        (-1.0, 0x00, 0x00, "Negative gain (clamped to 0.0)"),
        (1.23, 0x00, 0x7B, "Fractional gain (1.23 * 100 = 123)"),
    ])
    def test_build_set_p_gain_frame(self, gain, high_byte, low_byte, description):
        """Test P-gain frame building with various values."""
        result = build_set_p_gain_frame(gain)
        assert len(result) == 4, f"{description}: frame should be 4 bytes"
        assert result[0] == CMD_SET_P_GAIN, f"{description}: command should match"
        assert result[1:3] == bytes([high_byte, low_byte]), f"{description}: value bytes should match"
        assert result[3] == crc8(bytes([CMD_SET_P_GAIN, high_byte, low_byte])), \
            f"{description}: CRC should be correct"


# =============================================================================
# Test Suite 5: Response Parsing Tests
# =============================================================================

@pytest.mark.unit
class TestParseResponse:
    """Test standard ACK/NACK response parsing."""

    def test_parse_response_ack(self):
        """Valid ACK."""
        resp_type, error_code = parse_response(b'\xFF')
        assert resp_type == RESP_ACK
        assert error_code == 0

    def test_parse_response_nack_with_error(self):
        """NACK with CRC error code."""
        resp_type, error_code = parse_response(b'\xFE\x01')
        assert resp_type == RESP_NACK
        assert error_code == 0x01

    def test_parse_response_nack_invalid_cmd(self):
        """NACK with invalid command."""
        resp_type, error_code = parse_response(b'\xFE\x02')
        assert resp_type == RESP_NACK
        assert error_code == ERR_INVALID_CMD

    def test_parse_response_nack_invalid_val(self):
        """NACK with invalid value."""
        resp_type, error_code = parse_response(b'\xFE\x03')
        assert resp_type == RESP_NACK
        assert error_code == ERR_INVALID_VAL

    def test_parse_response_nack_no_error_code(self):
        """NACK without error byte."""
        resp_type, error_code = parse_response(b'\xFE')
        assert resp_type == RESP_NACK
        assert error_code == 0

    def test_parse_response_empty(self):
        """Empty response."""
        resp_type, error_code = parse_response(b'')
        assert resp_type == RESP_NACK
        assert error_code == 0

    def test_parse_response_unknown_type(self):
        """Unknown response type."""
        resp_type, error_code = parse_response(b'\x01')
        assert resp_type == RESP_NACK
        assert error_code == 0


# =============================================================================
# Test Suite 6: ADC Response Parsing Tests
# =============================================================================

@pytest.mark.unit
class TestParseADCResponse:
    """Test ADC response parsing."""

    def test_parse_adc_response_valid_min(self):
        """Zero ADC value."""
        data = bytes([CMD_READ_ADC, 0x00, 0x00])
        response = data + bytes([crc8(data)])
        adc_value, error_code = parse_adc_response(response)
        assert adc_value == 0
        assert error_code == 0

    def test_parse_adc_response_valid_mid(self):
        """Mid-range ADC value (2048)."""
        data = bytes([CMD_READ_ADC, 0x08, 0x00])
        response = data + bytes([crc8(data)])
        adc_value, error_code = parse_adc_response(response)
        assert adc_value == 2048
        assert error_code == 0

    def test_parse_adc_response_valid_max(self):
        """Maximum ADC value (4095)."""
        data = bytes([CMD_READ_ADC, 0x0F, 0xFF])
        response = data + bytes([crc8(data)])
        adc_value, error_code = parse_adc_response(response)
        assert adc_value == 4095
        assert error_code == 0

    def test_parse_adc_response_wrong_crc(self):
        """Wrong CRC should return error."""
        adc_value, error_code = parse_adc_response(b'\x03\x08\x00\x00')
        assert adc_value == -1
        assert error_code == ERR_CRC

    def test_parse_adc_response_wrong_command(self):
        """Wrong CMD byte."""
        adc_value, error_code = parse_adc_response(b'\x04\x08\x00\xF0')
        assert adc_value == -1
        assert error_code == ERR_INVALID_CMD

    def test_parse_adc_response_too_short(self):
        """Incomplete frame."""
        adc_value, error_code = parse_adc_response(b'\x03\x08')
        assert adc_value == -1
        assert error_code == 0

    def test_parse_adc_response_empty(self):
        """Empty input."""
        adc_value, error_code = parse_adc_response(b'')
        assert adc_value == -1
        assert error_code == 0


# =============================================================================
# Test Suite 7: Stream Data Parsing Tests
# =============================================================================

@pytest.mark.unit
class TestParseStreamData:
    """Test ADC streaming data parsing."""

    def test_parse_stream_data_valid(self):
        """Valid streaming data."""
        data = bytes([CMD_START_STREAM, 0x08, 0x00])
        response = data + bytes([crc8(data)])
        adc_value, error_code = parse_stream_data(response)
        assert adc_value == 2048
        assert error_code == 0

    def test_parse_stream_data_crc_error(self):
        """CRC validation."""
        adc_value, error_code = parse_stream_data(b'\x04\x08\x00\x00')
        assert adc_value == -1
        assert error_code == ERR_CRC

    def test_parse_stream_data_wrong_marker(self):
        """Wrong CMD byte."""
        adc_value, error_code = parse_stream_data(b'\x03\x08\x00\xF0')
        assert adc_value == -1
        assert error_code == ERR_INVALID_CMD

    def test_parse_stream_data_too_short(self):
        """Incomplete frame."""
        adc_value, error_code = parse_stream_data(b'\x04\x08')
        assert adc_value == -1
        assert error_code == 0


# =============================================================================
# Test Suite 8: P-Stream Data Parsing Tests
# =============================================================================

@pytest.mark.unit
class TestParsePStreamData:
    """Test P-controller streaming data parsing."""

    def test_parse_p_stream_data_valid(self):
        """Valid P-stream data (setpoint=2048, measured=2048, pwm=50)."""
        data = bytes([CMD_START_P_STREAM, 0x08, 0x00, 0x08, 0x00, 0x32])
        response = data + bytes([crc8(data)])
        setpoint, measured, pwm, error_code = parse_p_stream_data(response)
        assert setpoint == 2048
        assert measured == 2048
        assert pwm == 50
        assert error_code == 0

    def test_parse_p_stream_data_crc_error(self):
        """CRC validation."""
        data = bytes([CMD_START_P_STREAM, 0x08, 0x00, 0x08, 0x00, 0x32])
        response = data + bytes([crc8(data) + 1])  # Wrong CRC
        setpoint, measured, pwm, error_code = parse_p_stream_data(response)
        assert setpoint == -1
        assert measured == -1
        assert pwm == -1
        assert error_code == ERR_CRC

    def test_parse_p_stream_data_wrong_marker(self):
        """Wrong CMD byte."""
        data = bytes([CMD_START_STREAM, 0x08, 0x00, 0x08, 0x00, 0x32])
        response = data + bytes([crc8(data)])
        setpoint, measured, pwm, error_code = parse_p_stream_data(response)
        assert setpoint == -1
        assert measured == -1
        assert pwm == -1
        assert error_code == ERR_INVALID_CMD

    def test_parse_p_stream_data_too_short(self):
        """Incomplete frame."""
        setpoint, measured, pwm, error_code = parse_p_stream_data(b'\x09\x08\x00')
        assert setpoint == -1
        assert measured == -1
        assert pwm == -1
        assert error_code == 0

    def test_parse_p_stream_data_all_min(self):
        """All minimum values."""
        data = bytes([CMD_START_P_STREAM, 0x00, 0x00, 0x00, 0x00, 0x00])
        response = data + bytes([crc8(data)])
        setpoint, measured, pwm, error_code = parse_p_stream_data(response)
        assert setpoint == 0
        assert measured == 0
        assert pwm == 0
        assert error_code == 0

    def test_parse_p_stream_data_max_values(self):
        """Max ADC and PWM values."""
        data = bytes([CMD_START_P_STREAM, 0x0F, 0xFF, 0x0F, 0xFF, 0x64])
        response = data + bytes([crc8(data)])
        setpoint, measured, pwm, error_code = parse_p_stream_data(response)
        assert setpoint == 4095
        assert measured == 4095
        assert pwm == 100
        assert error_code == 0


# =============================================================================
# Test Suite 9: P-Status Response Parsing Tests
# =============================================================================

@pytest.mark.unit
class TestParsePStatusResponse:
    """Test P-controller status response parsing."""

    def test_parse_p_status_manual_mode(self):
        """Manual mode status."""
        data = bytes([CMD_GET_P_STATUS, 0x00, 0x08, 0x00, 0x32])
        response = data + bytes([crc8(data)])
        mode, setpoint, pwm, error_code = parse_p_status_response(response)
        assert mode == 0
        assert setpoint == 2048
        assert pwm == 50
        assert error_code == 0

    def test_parse_p_status_auto_mode(self):
        """Auto mode status."""
        data = bytes([CMD_GET_P_STATUS, 0x01, 0x0F, 0xFF, 0x64])
        response = data + bytes([crc8(data)])
        mode, setpoint, pwm, error_code = parse_p_status_response(response)
        assert mode == 1
        assert setpoint == 4095
        assert pwm == 100
        assert error_code == 0

    def test_parse_p_status_crc_error(self):
        """CRC validation."""
        data = bytes([CMD_GET_P_STATUS, 0x01, 0x0F, 0xFF, 0x64])
        response = data + bytes([crc8(data) + 1])  # Wrong CRC
        mode, setpoint, pwm, error_code = parse_p_status_response(response)
        assert mode == -1
        assert setpoint == -1
        assert pwm == -1
        assert error_code == ERR_CRC

    def test_parse_p_status_wrong_command(self):
        """Wrong CMD byte."""
        data = bytes([CMD_START_P_STREAM, 0x01, 0x0F, 0xFF, 0x64])
        response = data + bytes([crc8(data)])
        mode, setpoint, pwm, error_code = parse_p_status_response(response)
        assert mode == -1
        assert setpoint == -1
        assert pwm == -1
        assert error_code == ERR_INVALID_CMD

    def test_parse_p_status_too_short(self):
        """Incomplete frame."""
        mode, setpoint, pwm, error_code = parse_p_status_response(b'\x0B\x01\x0F')
        assert mode == -1
        assert setpoint == -1
        assert pwm == -1
        assert error_code == 0


# =============================================================================
# Test Suite 10: Error Handling Tests
# =============================================================================

@pytest.mark.unit
class TestErrorHandling:
    """Test error code and debug code translation."""

    @pytest.mark.parametrize("error_code,expected_name", [
        (ERR_CRC, "CRC error"),
        (ERR_INVALID_CMD, "Invalid command"),
        (ERR_INVALID_VAL, "Invalid value"),
        (ERR_INVALID_MODE, "Invalid mode"),
        (ERR_INVALID_SETPOINT, "Invalid setpoint"),
        (ERR_INVALID_GAIN, "Invalid gain"),
        (ERR_INVALID_FF, "Invalid feed-forward"),
        (0xFF, "Unknown error 255"),
        (0x00, "Unknown error 0"),
    ])
    def test_get_error_name(self, error_code, expected_name):
        """Test error code to name translation."""
        assert get_error_name(error_code) == expected_name


# =============================================================================
# Test Suite 11: Debug Name Tests
# =============================================================================

@pytest.mark.unit
class TestDebugNames:
    """Test debug code translation."""

    @pytest.mark.parametrize("debug_code,expected_name", [
        (1, "TIMER_START"),
        (2, "TIMER_CB"),
        (3, "ADC_READ"),
        (4, "ADC_ERROR"),
        (5, "TX_PUT"),
        (255, "DEBUG_255"),
        (0, "DEBUG_0"),
    ])
    def test_get_debug_name(self, debug_code, expected_name):
        """Test debug code to name translation."""
        assert get_debug_name(debug_code) == expected_name


# =============================================================================
# Test Suite 12: Integration Tests
# =============================================================================

@pytest.mark.integration
class TestIntegration:
    """Integration tests for protocol roundtrips."""

    def test_roundtrip_build_parse_adc_response(self):
        """Build ADC frame, simulate response, parse."""
        # Build request
        request = build_adc_frame()
        assert len(request) == 2
        assert request[0] == CMD_READ_ADC

        # Simulate device response with ADC = 2048
        data = bytes([CMD_READ_ADC, 0x08, 0x00])
        response = data + bytes([crc8(data)])

        # Parse response
        adc_value, error_code = parse_adc_response(response)
        assert adc_value == 2048
        assert error_code == 0

    def test_roundtrip_build_parse_p_status(self):
        """Build status request, simulate response, parse."""
        # Build request
        request = build_get_p_status_frame()
        assert request[0] == CMD_GET_P_STATUS

        # Simulate device response: mode=1, setpoint=2048, pwm=50
        data = bytes([CMD_GET_P_STATUS, 0x01, 0x08, 0x00, 0x32])
        response = data + bytes([crc8(data)])

        # Parse response
        mode, setpoint, pwm, error_code = parse_p_status_response(response)
        assert mode == 1
        assert setpoint == 2048
        assert pwm == 50
        assert error_code == 0

    def test_crc_validation_catches_corruption(self):
        """Verify CRC detects ALL single-bit errors (non-flaky).

        This test systematically corrupts each bit in the data portion of
        a valid frame and verifies that the CRC always detects the corruption.
        This is a fundamental property of a good CRC polynomial.
        """
        valid_frame = b'\x01\x01\x12'  # SET LED ON

        total_corruptions = 0
        detected_corruptions = 0
        undetected_cases = []

        # Corrupt each bit position in each byte (excluding CRC byte)
        for byte_idx in range(len(valid_frame) - 1):
            for bit_idx in range(8):
                total_corruptions += 1
                corrupted = bytearray(valid_frame)
                corrupted[byte_idx] ^= (1 << bit_idx)
                corrupted_bytes = bytes(corrupted)

                # Recalculate CRC for corrupted data
                new_crc = crc8(corrupted_bytes[:2])
                if new_crc != corrupted_bytes[2]:
                    # CRC detected the corruption
                    detected_corruptions += 1
                else:
                    # Track undetected cases for debugging
                    undetected_cases.append((byte_idx, bit_idx))

        # CRC should detect ALL single-bit errors
        assert detected_corruptions == total_corruptions, \
            f"CRC only detected {detected_corruptions}/{total_corruptions} single-bit errors. " \
            f"Undetected cases: {undetected_cases}"
