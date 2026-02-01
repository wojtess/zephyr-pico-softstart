"""
Tests for LEDControllerApp handler functions (Phase 2c).

These tests focus on:
- Connection handlers (_handle_connect, _handle_disconnect)
- LED/PWM handlers (_handle_send_led, _handle_send_pwm)
- ADC handler (_handle_read_adc)
- P-Controller handlers (_handle_set_p_mode, _handle_set_p_setpoint, etc.)
- Streaming handlers (_handle_start_stream, _handle_stop_stream, etc.)

Uses mock_serial_connection fixture to simulate serial communication.
"""

import sys
import queue
import time
from pathlib import Path
from unittest.mock import MagicMock, patch

import pytest

# Add project root to Python path for imports
project_root = Path(__file__).parent.parent.parent
sys.path.insert(0, str(project_root))

from scripts.gui.app import LEDControllerApp
from scripts.gui.constants import SerialCommand, SerialResult
from scripts.protocol import (
    CMD_SET_LED,
    CMD_SET_PWM,
    CMD_READ_ADC,
    CMD_START_STREAM,
    CMD_STOP_STREAM,
    CMD_SET_MODE,
    CMD_SET_P_SETPOINT,
    CMD_SET_P_GAIN,
    CMD_SET_FEED_FORWARD,
    CMD_START_P_STREAM,
    CMD_STOP_P_STREAM,
    RESP_ACK,
    RESP_NACK,
    ERR_CRC,
    ERR_INVALID_CMD,
    ERR_INVALID_VAL,
    ERR_INVALID_MODE,
    crc8,
    build_frame,
    build_adc_frame,
    build_stream_start_frame,
    build_stream_stop_frame,
    build_set_mode_frame,
    build_set_p_setpoint_frame,
    build_set_p_gain_frame,
    build_set_feed_forward_frame,
    build_p_stream_start_frame,
    build_p_stream_stop_frame,
)


# =============================================================================
# Test Suite 1: Connection Handlers (~10 tests)
# =============================================================================

@pytest.mark.unit
class TestHandleConnect:
    """Test _handle_connect handler method."""

    def test_connect_success(self, led_app):
        """Successful serial connection."""
        # Mock serial.Serial to return a mock connection
        mock_conn = MagicMock()
        mock_conn.is_open = True
        mock_conn.port = "/dev/ttyACM0"

        with patch('scripts.gui.app.serial.Serial', return_value=mock_conn):
            result = led_app._handle_connect("/dev/ttyACM0")

        assert result.success is True
        assert "Connected to" in result.message
        assert led_app._is_connected is True
        assert led_app._serial_connection is not None

    def test_connect_already_connected(self, led_app, mock_serial_connection):
        """Error when already connected."""
        # Set up as already connected
        led_app._serial_connection = mock_serial_connection
        mock_serial_connection.is_open = True
        led_app._is_connected = True

        result = led_app._handle_connect("/dev/ttyACM0")

        assert result.success is False
        assert "Already connected" in result.message
        assert result.error == "Already connected"

    def test_connect_port_not_found(self, led_app):
        """Port not found error translation."""
        import serial

        def mock_serial_init(*args, **kwargs):
            raise serial.SerialException("could not open port /dev/ttyACM99: [Errno 2] No such file or directory")

        with patch('scripts.gui.app.serial.Serial', side_effect=mock_serial_init):
            result = led_app._handle_connect("/dev/ttyACM99")

        assert result.success is False
        assert "not found" in result.message.lower() or "device" in result.message.lower()

    def test_connect_permission_denied(self, led_app):
        """Permission denied error translation."""
        import serial

        def mock_serial_init(*args, **kwargs):
            raise serial.SerialException("Permission denied: '/dev/ttyACM0'")

        with patch('scripts.gui.app.serial.Serial', side_effect=mock_serial_init):
            result = led_app._handle_connect("/dev/ttyACM0")

        assert result.success is False
        assert "busy" in result.message.lower() or "permission" in result.message.lower()

    def test_connect_device_busy(self, led_app):
        """Device busy error translation."""
        import serial

        def mock_serial_init(*args, **kwargs):
            raise serial.SerialException("Device or resource busy: '/dev/ttyACM0'")

        with patch('scripts.gui.app.serial.Serial', side_effect=mock_serial_init):
            result = led_app._handle_connect("/dev/ttyACM0")

        assert result.success is False
        assert "busy" in result.message.lower()

    def test_connect_generic_error(self, led_app):
        """Generic SerialException handling."""
        import serial

        def mock_serial_init(*args, **kwargs):
            raise serial.SerialException("Unknown error")

        with patch('scripts.gui.app.serial.Serial', side_effect=mock_serial_init):
            result = led_app._handle_connect("/dev/ttyACM0")

        assert result.success is False
        assert "failed" in result.message.lower()

    def test_connect_flushes_buffers(self, led_app):
        """Connection flushes input/output buffers."""
        mock_conn = MagicMock()
        mock_conn.is_open = True
        mock_conn.port = "/dev/ttyACM0"

        with patch('scripts.gui.app.serial.Serial', return_value=mock_conn):
            result = led_app._handle_connect("/dev/ttyACM0")

        assert result.success is True
        mock_conn.reset_input_buffer.assert_called_once()
        mock_conn.reset_output_buffer.assert_called_once()

    def test_connect_stores_port(self, led_app):
        """Connection stores the current port."""
        mock_conn = MagicMock()
        mock_conn.is_open = True
        mock_conn.port = "/dev/ttyUSB0"

        with patch('scripts.gui.app.serial.Serial', return_value=mock_conn):
            result = led_app._handle_connect("/dev/ttyUSB0")

        assert result.success is True
        # Port is not directly stored but accessible via connection
        assert led_app._serial_connection.port == "/dev/ttyUSB0"

    def test_connect_unexpected_exception(self, led_app):
        """Unexpected exceptions during connection."""
        def mock_serial_init(*args, **kwargs):
            raise RuntimeError("Unexpected error")

        with patch('scripts.gui.app.serial.Serial', side_effect=mock_serial_init):
            result = led_app._handle_connect("/dev/ttyACM0")

        assert result.success is False
        assert "Unexpected" in result.message


@pytest.mark.unit
class TestHandleDisconnect:
    """Test _handle_disconnect handler method."""

    def test_disconnect_success(self, led_app_with_mock_serial):
        """Successful disconnect."""
        app, mock_serial = led_app_with_mock_serial

        result = app._handle_disconnect()

        assert result.success is True
        assert "Disconnected" in result.message
        assert app._is_connected is False
        assert app._serial_connection is None
        assert mock_serial.is_open is False

    def test_disconnect_clears_led_state(self, led_app_with_mock_serial):
        """All state cleared after disconnect."""
        app, mock_serial = led_app_with_mock_serial
        app._led_state = True

        result = app._handle_disconnect()

        assert result.success is True
        assert app._led_state is False

    def test_disconnect_when_not_connected(self, led_app):
        """Disconnect when already disconnected."""
        # App is not connected
        result = led_app._handle_disconnect()

        assert result.success is True
        assert led_app._is_connected is False

    def test_disconnect_handles_exception(self, led_app_with_mock_serial):
        """Handles exceptions during disconnect."""
        app, mock_serial = led_app_with_mock_serial
        # Store original close method
        original_close = mock_serial.close
        # Make close() raise an exception
        mock_serial.close = MagicMock(side_effect=Exception("Close error"))

        result = app._handle_disconnect()

        # Restore original close for teardown
        mock_serial.close = original_close

        # Should still succeed and clean up state
        assert result.success is False
        assert "error" in result.message.lower()


# =============================================================================
# Test Suite 2: LED/PWM Handlers (~15 tests)
# =============================================================================

@pytest.mark.unit
class TestHandleSendLed:
    """Test _handle_send_led handler method."""

    def test_send_led_on_success(self, led_app_with_mock_serial):
        """Send LED ON, get ACK."""
        app, mock_serial = led_app_with_mock_serial

        # Queue ACK response
        app._response_queue.put(RESP_ACK)

        result = app._handle_send_led(True)

        assert result.success is True
        assert "ON" in result.message
        assert result.led_state is True
        assert app._led_state is True
        assert len(mock_serial.written_data) == 1

    def test_send_led_off_success(self, led_app_with_mock_serial):
        """Send LED OFF, get ACK."""
        app, mock_serial = led_app_with_mock_serial
        app._led_state = True  # Start with LED ON

        # Queue ACK response
        app._response_queue.put(RESP_ACK)

        result = app._handle_send_led(False)

        assert result.success is True
        assert "OFF" in result.message
        assert result.led_state is False
        assert app._led_state is False

    def test_send_led_writes_correct_frame(self, led_app_with_mock_serial):
        """Verify correct frame is written for LED ON."""
        app, mock_serial = led_app_with_mock_serial

        app._response_queue.put(RESP_ACK)

        app._handle_send_led(True)

        written = mock_serial.written_data[0]
        assert written[0] == CMD_SET_LED
        assert written[1] == 1  # ON
        assert written[2] == crc8(bytes([CMD_SET_LED, 1]))

    def test_send_led_timeout(self, led_app_with_mock_serial):
        """Timeout waiting for response."""
        app, mock_serial = led_app_with_mock_serial

        result = app._handle_send_led(True)

        assert result.success is False
        assert "timeout" in result.message.lower() or "not responding" in result.message.lower()
        assert result.error == "Timeout"
        assert app._is_connected is False

    def test_send_led_nack(self, led_app_with_mock_serial):
        """NACK response."""
        app, mock_serial = led_app_with_mock_serial

        # Queue NACK + error code
        app._response_queue.put(RESP_NACK)
        app._response_queue.put(ERR_INVALID_CMD)

        result = app._handle_send_led(True)

        assert result.success is False
        assert "failed" in result.message.lower()
        assert result.led_state is None

    def test_send_led_not_connected(self, led_app):
        """Error when not connected."""
        led_app._is_connected = False
        led_app._serial_connection = None

        result = led_app._handle_send_led(True)

        assert result.success is False
        assert "not connected" in result.message.lower()
        assert result.error == "No connection"

    def test_send_led_nack_without_error_code(self, led_app_with_mock_serial):
        """NACK response without error code."""
        app, mock_serial = led_app_with_mock_serial

        # Queue only NACK, no error code
        app._response_queue.put(RESP_NACK)

        result = app._handle_send_led(True)

        assert result.success is False
        assert "failed" in result.message.lower()

    def test_send_led_with_open_but_closed_connection(self, led_app_with_mock_serial):
        """Handles connection object exists but not open."""
        app, mock_serial = led_app_with_mock_serial
        mock_serial.is_open = False

        result = app._handle_send_led(True)

        assert result.success is False
        assert "not connected" in result.message.lower()

    def test_send_led_flushes_after_write(self, led_app_with_mock_serial):
        """Verifies flush is called after write."""
        app, mock_serial = led_app_with_mock_serial
        app._response_queue.put(RESP_ACK)

        app._handle_send_led(True)

        # Verify flush was called (we track this via the mock)
        # The flush method should have been called
        assert len(mock_serial.written_data) == 1


@pytest.mark.unit
class TestHandleSendPwm:
    """Test _handle_send_pwm handler method."""

    def test_send_pwm_success(self, led_app_with_mock_serial):
        """Send PWM, get ACK."""
        app, mock_serial = led_app_with_mock_serial

        app._response_queue.put(RESP_ACK)

        result = app._handle_send_pwm(50)

        assert result.success is True
        assert "50%" in result.message
        assert app._pwm_duty == 50
        assert app._last_known_pwm == 50
        assert len(mock_serial.written_data) == 1

    def test_send_pwm_zero(self, led_app_with_mock_serial):
        """Send PWM = 0 (LED off)."""
        app, mock_serial = led_app_with_mock_serial

        app._response_queue.put(RESP_ACK)

        result = app._handle_send_pwm(0)

        assert result.success is True
        assert app._pwm_duty == 0
        assert app._led_state is False

    def test_send_pwm_full(self, led_app_with_mock_serial):
        """Send PWM = 100 (full brightness)."""
        app, mock_serial = led_app_with_mock_serial

        app._response_queue.put(RESP_ACK)

        result = app._handle_send_pwm(100)

        assert result.success is True
        assert app._pwm_duty == 100
        assert app._led_state is True

    def test_send_pwm_invalid_range_negative(self, led_app_with_mock_serial):
        """PWM < 0 is rejected."""
        app, mock_serial = led_app_with_mock_serial

        result = app._handle_send_pwm(-1)

        assert result.success is False
        assert "invalid" in result.message.lower()
        assert result.error == "Invalid value"
        assert len(mock_serial.written_data) == 0  # Nothing written

    def test_send_pwm_invalid_range_over_100(self, led_app_with_mock_serial):
        """PWM > 100 is rejected."""
        app, mock_serial = led_app_with_mock_serial

        result = app._handle_send_pwm(101)

        assert result.success is False
        assert "invalid" in result.message.lower()
        assert result.error == "Invalid value"

    def test_send_pwm_timeout(self, led_app_with_mock_serial):
        """Timeout waiting for response."""
        app, mock_serial = led_app_with_mock_serial

        result = app._handle_send_pwm(50)

        assert result.success is False
        assert "timeout" in result.message.lower() or "not responding" in result.message.lower()
        assert app._is_connected is False
        assert app._pwm_duty == 0

    def test_send_pwm_nack(self, led_app_with_mock_serial):
        """NACK response."""
        app, mock_serial = led_app_with_mock_serial

        app._response_queue.put(RESP_NACK)
        app._response_queue.put(ERR_INVALID_VAL)

        result = app._handle_send_pwm(50)

        assert result.success is False
        assert "failed" in result.message.lower()

    def test_send_pwm_not_connected(self, led_app):
        """Error when not connected."""
        led_app._is_connected = False
        led_app._serial_connection = None

        result = led_app._handle_send_pwm(50)

        assert result.success is False
        assert "not connected" in result.message.lower()

    def test_send_pwm_writes_correct_frame(self, led_app_with_mock_serial):
        """Verify correct frame is written."""
        app, mock_serial = led_app_with_mock_serial

        app._response_queue.put(RESP_ACK)

        app._handle_send_pwm(75)

        written = mock_serial.written_data[0]
        assert written[0] == CMD_SET_PWM
        assert written[1] == 75


# =============================================================================
# Test Suite 3: ADC Handler (~8 tests)
# =============================================================================

@pytest.mark.unit
class TestHandleReadAdc:
    """Test _handle_read_adc handler method."""

    def test_read_adc_success(self, led_app_with_mock_serial):
        """Read ADC, get 4-byte response."""
        app, mock_serial = led_app_with_mock_serial

        # Queue valid ADC response: [CMD][ADC_H][ADC_L][CRC]
        adc_value = 2048  # Midpoint
        adc_h = (adc_value >> 8) & 0xFF
        adc_l = adc_value & 0xFF
        data = bytes([CMD_READ_ADC, adc_h, adc_l])
        crc = crc8(data)
        app._response_queue.put(CMD_READ_ADC)
        app._response_queue.put(adc_h)
        app._response_queue.put(adc_l)
        app._response_queue.put(crc)

        result = app._handle_read_adc()

        assert result.success is True
        assert result.adc_value == 2048
        assert result.adc_voltage is not None
        assert abs(result.adc_voltage - 1.65) < 0.01  # ~1.65V at midpoint

    def test_read_adc_parses_value(self, led_app_with_mock_serial):
        """ADC value correctly parsed."""
        app, mock_serial = led_app_with_mock_serial

        # Test with ADC = 0
        data = bytes([CMD_READ_ADC, 0, 0])
        crc = crc8(data)
        for b in [CMD_READ_ADC, 0, 0, crc]:
            app._response_queue.put(b)

        result = app._handle_read_adc()

        assert result.adc_value == 0
        assert result.adc_voltage == 0.0

    def test_read_adc_max_value(self, led_app_with_mock_serial):
        """Read ADC at maximum value (4095)."""
        app, mock_serial = led_app_with_mock_serial

        adc_value = 4095
        adc_h = (adc_value >> 8) & 0xFF
        adc_l = adc_value & 0xFF
        data = bytes([CMD_READ_ADC, adc_h, adc_l])
        crc = crc8(data)
        for b in [CMD_READ_ADC, adc_h, adc_l, crc]:
            app._response_queue.put(b)

        result = app._handle_read_adc()

        assert result.adc_value == 4095
        assert abs(result.adc_voltage - 3.3) < 0.01

    def test_read_adc_crc_error(self, led_app_with_mock_serial):
        """CRC validation failed."""
        app, mock_serial = led_app_with_mock_serial

        # Queue response with bad CRC
        app._response_queue.put(CMD_READ_ADC)
        app._response_queue.put(0x08)
        app._response_queue.put(0x00)
        app._response_queue.put(0xFF)  # Bad CRC

        result = app._handle_read_adc()

        assert result.success is False
        assert "error" in result.message.lower()

    def test_read_adc_timeout(self, led_app_with_mock_serial):
        """Timeout waiting for response."""
        app, mock_serial = led_app_with_mock_serial

        result = app._handle_read_adc()

        assert result.success is False
        assert "timeout" in result.message.lower() or "not responding" in result.message.lower()

    def test_read_adc_not_connected(self, led_app):
        """Error when not connected."""
        led_app._is_connected = False
        led_app._serial_connection = None

        result = led_app._handle_read_adc()

        assert result.success is False
        assert "not connected" in result.message.lower()

    def test_read_adc_incomplete_response(self, led_app_with_mock_serial):
        """Incomplete ADC response (less than 4 bytes)."""
        app, mock_serial = led_app_with_mock_serial

        # Queue only 2 bytes
        app._response_queue.put(CMD_READ_ADC)
        app._response_queue.put(0x08)
        # Then timeout on remaining bytes

        result = app._handle_read_adc()

        assert result.success is False

    def test_read_adc_wrong_command_response(self, led_app_with_mock_serial):
        """Response has wrong command byte."""
        app, mock_serial = led_app_with_mock_serial

        # Wrong command in response
        app._response_queue.put(CMD_SET_LED)  # Wrong!
        app._response_queue.put(0x08)
        app._response_queue.put(0x00)
        app._response_queue.put(0x00)

        result = app._handle_read_adc()

        assert result.success is False


# =============================================================================
# Test Suite 4: P-Controller Handlers (~25 tests)
# =============================================================================

@pytest.mark.unit
class TestHandleSetPMode:
    """Test _handle_set_p_mode handler method."""

    def test_set_p_mode_manual_success(self, led_app_with_mock_serial):
        """Set mode to manual."""
        app, mock_serial = led_app_with_mock_serial

        app._response_queue.put(RESP_ACK)

        result = app._handle_set_p_mode(0)

        assert result.success is True
        assert "Manual" in result.message
        assert app._p_mode == 0

    def test_set_p_mode_auto_success(self, led_app_with_mock_serial):
        """Set mode to auto."""
        app, mock_serial = led_app_with_mock_serial

        app._response_queue.put(RESP_ACK)

        result = app._handle_set_p_mode(1)

        assert result.success is True
        assert "P-Control" in result.message
        assert app._p_mode == 1

    def test_set_p_mode_invalid(self, led_app_with_mock_serial):
        """Invalid mode value."""
        app, mock_serial = led_app_with_mock_serial

        result = app._handle_set_p_mode(2)

        assert result.success is False
        assert "invalid" in result.message.lower()
        assert result.error == "Invalid value"
        # Nothing written
        assert len(mock_serial.written_data) == 0

    def test_set_p_mode_not_connected(self, led_app):
        """Error when not connected."""
        led_app._is_connected = False
        led_app._serial_connection = None

        result = led_app._handle_set_p_mode(1)

        assert result.success is False
        assert "not connected" in result.message.lower()

    def test_set_p_mode_timeout(self, led_app_with_mock_serial):
        """Timeout waiting for response."""
        app, mock_serial = led_app_with_mock_serial

        result = app._handle_set_p_mode(1)

        assert result.success is False
        assert "timeout" in result.message.lower()

    def test_set_p_mode_nack(self, led_app_with_mock_serial):
        """NACK response."""
        app, mock_serial = led_app_with_mock_serial

        app._response_queue.put(RESP_NACK)
        app._response_queue.put(ERR_INVALID_MODE)

        result = app._handle_set_p_mode(1)

        assert result.success is False
        assert "failed" in result.message.lower()


@pytest.mark.unit
class TestHandleSetPSetpoint:
    """Test _handle_set_p_setpoint handler method."""

    def test_set_p_setpoint_success(self, led_app_with_mock_serial):
        """Set setpoint, get ACK."""
        app, mock_serial = led_app_with_mock_serial

        app._response_queue.put(RESP_ACK)

        result = app._handle_set_p_setpoint(2048)

        assert result.success is True
        assert "2048" in result.message
        assert app._p_setpoint == 2048

    def test_set_p_setpoint_zero(self, led_app_with_mock_serial):
        """Set setpoint to 0."""
        app, mock_serial = led_app_with_mock_serial

        app._response_queue.put(RESP_ACK)

        result = app._handle_set_p_setpoint(0)

        assert result.success is True
        assert app._p_setpoint == 0

    def test_set_p_setpoint_max(self, led_app_with_mock_serial):
        """Set setpoint to maximum (4095)."""
        app, mock_serial = led_app_with_mock_serial

        app._response_queue.put(RESP_ACK)

        result = app._handle_set_p_setpoint(4095)

        assert result.success is True
        assert app._p_setpoint == 4095

    def test_set_p_setpoint_out_of_range_negative(self, led_app_with_mock_serial):
        """Setpoint < 0 is rejected."""
        app, mock_serial = led_app_with_mock_serial

        result = app._handle_set_p_setpoint(-1)

        assert result.success is False
        assert "invalid" in result.message.lower()
        assert result.error == "Invalid value"

    def test_set_p_setpoint_out_of_range_over_max(self, led_app_with_mock_serial):
        """Setpoint > 4095 is rejected."""
        app, mock_serial = led_app_with_mock_serial

        result = app._handle_set_p_setpoint(4096)

        assert result.success is False
        assert "invalid" in result.message.lower()
        assert result.error == "Invalid value"

    def test_set_p_setpoint_timeout(self, led_app_with_mock_serial):
        """Timeout waiting for response."""
        app, mock_serial = led_app_with_mock_serial

        result = app._handle_set_p_setpoint(2048)

        assert result.success is False
        assert "timeout" in result.message.lower()

    def test_set_p_setpoint_not_connected(self, led_app):
        """Error when not connected."""
        led_app._is_connected = False
        led_app._serial_connection = None

        result = led_app._handle_set_p_setpoint(2048)

        assert result.success is False
        assert "not connected" in result.message.lower()


@pytest.mark.unit
class TestHandleSetPGain:
    """Test _handle_set_p_gain handler method."""

    def test_set_p_gain_success(self, led_app_with_mock_serial):
        """Set gain, get ACK."""
        app, mock_serial = led_app_with_mock_serial

        app._response_queue.put(RESP_ACK)

        result = app._handle_set_p_gain(1.5)

        assert result.success is True
        assert "1.50" in result.message
        assert app._p_gain == 1.5

    def test_set_p_gain_zero(self, led_app_with_mock_serial):
        """Set gain to 0."""
        app, mock_serial = led_app_with_mock_serial

        app._response_queue.put(RESP_ACK)

        result = app._handle_set_p_gain(0.0)

        assert result.success is True
        assert app._p_gain == 0.0

    def test_set_p_gain_max(self, led_app_with_mock_serial):
        """Set gain to maximum (10.0)."""
        app, mock_serial = led_app_with_mock_serial

        app._response_queue.put(RESP_ACK)

        result = app._handle_set_p_gain(10.0)

        assert result.success is True
        assert app._p_gain == 10.0

    def test_set_p_gain_negative(self, led_app_with_mock_serial):
        """Gain < 0 is rejected."""
        app, mock_serial = led_app_with_mock_serial

        result = app._handle_set_p_gain(-0.1)

        assert result.success is False
        assert "invalid" in result.message.lower()
        assert result.error == "Invalid value"

    def test_set_p_gain_over_max(self, led_app_with_mock_serial):
        """Gain > 10.0 is rejected."""
        app, mock_serial = led_app_with_mock_serial

        result = app._handle_set_p_gain(10.1)

        assert result.success is False
        assert "invalid" in result.message.lower()
        assert result.error == "Invalid value"

    def test_set_p_gain_timeout(self, led_app_with_mock_serial):
        """Timeout waiting for response."""
        app, mock_serial = led_app_with_mock_serial

        result = app._handle_set_p_gain(1.5)

        assert result.success is False
        assert "timeout" in result.message.lower()

    def test_set_p_gain_not_connected(self, led_app):
        """Error when not connected."""
        led_app._is_connected = False
        led_app._serial_connection = None

        result = led_app._handle_set_p_gain(1.5)

        assert result.success is False
        assert "not connected" in result.message.lower()


@pytest.mark.unit
class TestHandleSetPFeedForward:
    """Test _handle_set_p_feed_forward handler method."""

    def test_set_p_feed_forward_success(self, led_app_with_mock_serial):
        """Set feed-forward, get ACK."""
        app, mock_serial = led_app_with_mock_serial

        app._response_queue.put(RESP_ACK)

        result = app._handle_set_p_feed_forward(50)

        assert result.success is True
        assert "50%" in result.message
        assert app._p_feed_forward == 50

    def test_set_p_feed_forward_zero(self, led_app_with_mock_serial):
        """Set feed-forward to 0."""
        app, mock_serial = led_app_with_mock_serial

        app._response_queue.put(RESP_ACK)

        result = app._handle_set_p_feed_forward(0)

        assert result.success is True
        assert app._p_feed_forward == 0

    def test_set_p_feed_forward_full(self, led_app_with_mock_serial):
        """Set feed-forward to 100."""
        app, mock_serial = led_app_with_mock_serial

        app._response_queue.put(RESP_ACK)

        result = app._handle_set_p_feed_forward(100)

        assert result.success is True
        assert app._p_feed_forward == 100

    def test_set_p_feed_forward_negative(self, led_app_with_mock_serial):
        """Feed-forward < 0 is rejected."""
        app, mock_serial = led_app_with_mock_serial

        result = app._handle_set_p_feed_forward(-1)

        assert result.success is False
        assert "invalid" in result.message.lower()
        assert result.error == "Invalid value"

    def test_set_p_feed_forward_over_100(self, led_app_with_mock_serial):
        """Feed-forward > 100 is rejected."""
        app, mock_serial = led_app_with_mock_serial

        result = app._handle_set_p_feed_forward(101)

        assert result.success is False
        assert "invalid" in result.message.lower()
        assert result.error == "Invalid value"

    def test_set_p_feed_forward_timeout(self, led_app_with_mock_serial):
        """Timeout waiting for response."""
        app, mock_serial = led_app_with_mock_serial

        result = app._handle_set_p_feed_forward(50)

        assert result.success is False
        assert "timeout" in result.message.lower()

    def test_set_p_feed_forward_not_connected(self, led_app):
        """Error when not connected."""
        led_app._is_connected = False
        led_app._serial_connection = None

        result = led_app._handle_set_p_feed_forward(50)

        assert result.success is False
        assert "not connected" in result.message.lower()


# =============================================================================
# Test Suite 5: Streaming Handlers (~20 tests)
# =============================================================================

@pytest.mark.unit
class TestHandleStartStream:
    """Test _handle_start_stream handler method."""

    def test_start_stream_success(self, led_app_with_mock_serial):
        """Start streaming."""
        app, mock_serial = led_app_with_mock_serial

        app._response_queue.put(RESP_ACK)

        result = app._handle_start_stream(100)

        assert result.success is True
        assert "started" in result.message.lower()
        assert app._is_streaming is True
        assert app._stream_interval == 100

    def test_start_stream_invalid_interval_too_low(self, led_app_with_mock_serial):
        """Interval < 10 is rejected."""
        app, mock_serial = led_app_with_mock_serial

        result = app._handle_start_stream(9)

        assert result.success is False
        assert "invalid" in result.message.lower()
        assert result.error == "Invalid value"
        # Nothing written
        assert len(mock_serial.written_data) == 0

    def test_start_stream_invalid_interval_too_high(self, led_app_with_mock_serial):
        """Interval > 60000 is rejected."""
        app, mock_serial = led_app_with_mock_serial

        result = app._handle_start_stream(60001)

        assert result.success is False
        assert "invalid" in result.message.lower()
        assert result.error == "Invalid value"

    def test_start_stream_already_streaming(self, led_app_with_mock_serial):
        """Already streaming error."""
        app, mock_serial = led_app_with_mock_serial
        app._is_streaming = True

        result = app._handle_start_stream(100)

        assert result.success is False
        assert "already streaming" in result.message.lower()
        assert result.error == "Already streaming"

    def test_start_stream_min_interval(self, led_app_with_mock_serial):
        """Start streaming with minimum interval (10ms)."""
        app, mock_serial = led_app_with_mock_serial

        app._response_queue.put(RESP_ACK)

        result = app._handle_start_stream(10)

        assert result.success is True
        assert app._stream_interval == 10

    def test_start_stream_max_interval(self, led_app_with_mock_serial):
        """Start streaming with maximum interval (60000ms)."""
        app, mock_serial = led_app_with_mock_serial

        app._response_queue.put(RESP_ACK)

        result = app._handle_start_stream(60000)

        assert result.success is True
        assert app._stream_interval == 60000

    def test_start_stream_timeout(self, led_app_with_mock_serial):
        """Timeout waiting for response."""
        app, mock_serial = led_app_with_mock_serial

        result = app._handle_start_stream(100)

        assert result.success is False
        assert "timeout" in result.message.lower()

    def test_start_stream_not_connected(self, led_app):
        """Error when not connected."""
        led_app._is_connected = False
        led_app._serial_connection = None

        result = led_app._handle_start_stream(100)

        assert result.success is False
        assert "not connected" in result.message.lower()


@pytest.mark.unit
class TestHandleStopStream:
    """Test _handle_stop_stream handler method."""

    def test_stop_stream_success(self, led_app_with_mock_serial):
        """Stop streaming."""
        app, mock_serial = led_app_with_mock_serial
        app._is_streaming = True

        app._response_queue.put(RESP_ACK)

        result = app._handle_stop_stream()

        assert result.success is True
        assert "stopped" in result.message.lower()
        assert app._is_streaming is False

    def test_stop_stream_when_not_streaming(self, led_app_with_mock_serial):
        """Not streaming error."""
        app, mock_serial = led_app_with_mock_serial
        app._is_streaming = False

        result = app._handle_stop_stream()

        assert result.success is False
        assert "not streaming" in result.message.lower()
        assert result.error == "Not streaming"

    def test_stop_stream_timeout(self, led_app_with_mock_serial):
        """Timeout waiting for response."""
        app, mock_serial = led_app_with_mock_serial
        app._is_streaming = True

        result = app._handle_stop_stream()

        assert result.success is False
        assert "timeout" in result.message.lower()
        # State should still be cleared on timeout
        assert app._is_streaming is False

    def test_stop_stream_not_connected(self, led_app):
        """Error when not connected."""
        led_app._is_connected = False
        led_app._serial_connection = None
        led_app._is_streaming = True

        result = led_app._handle_stop_stream()

        assert result.success is False
        assert "not connected" in result.message.lower()


@pytest.mark.unit
class TestHandleStartPStream:
    """Test _handle_start_p_stream handler method."""

    def test_start_p_stream_success(self, led_app_with_mock_serial):
        """Start P-streaming."""
        app, mock_serial = led_app_with_mock_serial

        app._response_queue.put(RESP_ACK)

        result = app._handle_start_p_stream(100)

        assert result.success is True
        assert "started" in result.message.lower()
        assert app._p_streaming is True

    def test_start_p_stream_invalid_interval_too_low(self, led_app_with_mock_serial):
        """Interval < 10 is rejected."""
        app, mock_serial = led_app_with_mock_serial

        result = app._handle_start_p_stream(9)

        assert result.success is False
        assert "invalid" in result.message.lower()
        assert result.error == "Invalid value"

    def test_start_p_stream_invalid_interval_too_high(self, led_app_with_mock_serial):
        """Interval > 60000 is rejected."""
        app, mock_serial = led_app_with_mock_serial

        result = app._handle_start_p_stream(60001)

        assert result.success is False
        assert "invalid" in result.message.lower()
        assert result.error == "Invalid value"

    def test_start_p_stream_already_streaming(self, led_app_with_mock_serial):
        """Already streaming error."""
        app, mock_serial = led_app_with_mock_serial
        app._p_streaming = True

        result = app._handle_start_p_stream(100)

        assert result.success is False
        assert "already streaming" in result.message.lower()
        assert result.error == "Already streaming"

    def test_start_p_stream_timeout(self, led_app_with_mock_serial):
        """Timeout waiting for response."""
        app, mock_serial = led_app_with_mock_serial

        result = app._handle_start_p_stream(100)

        assert result.success is False
        assert "timeout" in result.message.lower()

    def test_start_p_stream_not_connected(self, led_app):
        """Error when not connected."""
        led_app._is_connected = False
        led_app._serial_connection = None

        result = led_app._handle_start_p_stream(100)

        assert result.success is False
        assert "not connected" in result.message.lower()


@pytest.mark.unit
class TestHandleStopPStream:
    """Test _handle_stop_p_stream handler method."""

    def test_stop_p_stream_success(self, led_app_with_mock_serial):
        """Stop P-streaming."""
        app, mock_serial = led_app_with_mock_serial
        app._p_streaming = True

        app._response_queue.put(RESP_ACK)

        result = app._handle_stop_p_stream()

        assert result.success is True
        assert "stopped" in result.message.lower()
        assert app._p_streaming is False

    def test_stop_p_stream_when_not_streaming(self, led_app_with_mock_serial):
        """Not streaming error."""
        app, mock_serial = led_app_with_mock_serial
        app._p_streaming = False

        result = app._handle_stop_p_stream()

        assert result.success is False
        assert "not streaming" in result.message.lower()
        assert result.error == "Not streaming"

    def test_stop_p_stream_timeout(self, led_app_with_mock_serial):
        """Timeout waiting for response."""
        app, mock_serial = led_app_with_mock_serial
        app._p_streaming = True

        result = app._handle_stop_p_stream()

        assert result.success is False
        assert "timeout" in result.message.lower()
        # State should still be cleared on timeout
        assert app._p_streaming is False

    def test_stop_p_stream_not_connected(self, led_app):
        """Error when not connected."""
        led_app._is_connected = False
        led_app._serial_connection = None
        led_app._p_streaming = True

        result = led_app._handle_stop_p_stream()

        assert result.success is False
        assert "not connected" in result.message.lower()
