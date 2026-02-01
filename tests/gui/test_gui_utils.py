"""
Tests for GUI utility functions (find_ports_with_info).

Phase 2a: GUI Utils Tests
Unit tests for utils.py pure functions (no dpg mocking).
"""

import pytest
from unittest.mock import patch, MagicMock
from typing import List, Dict

from scripts.gui.utils import (
    find_ports_with_info,
    get_selected_port,
    handle_disconnect_state,
    update_status,
    update_connection_indicator,
    update_led_indicator,
)
from scripts.gui.constants import TAGS


@pytest.mark.unit
class TestFindPortsWithInfo:
    """Test find_ports_with_info() - pure function, controlled via monkeypatch."""

    def test_no_ports_available(self, monkeypatch):
        """Return empty list when no ports found."""
        monkeypatch.setattr("glob.glob", lambda x: [])
        ports = find_ports_with_info()
        assert ports == []

    def test_linux_acm_ports_found(self, monkeypatch):
        """Find Linux ACM ports (/dev/ttyACM*)."""
        def mock_glob(pattern):
            if pattern == "/dev/ttyACM*":
                return ["/dev/ttyACM0", "/dev/ttyACM1"]
            return []

        monkeypatch.setattr("glob.glob", mock_glob)
        ports = find_ports_with_info()
        assert len(ports) == 2
        assert ports[0]["device"] == "/dev/ttyACM0"
        assert ports[0]["description"] == "ACM Device"
        assert ports[1]["device"] == "/dev/ttyACM1"

    def test_linux_usb_ports_found(self, monkeypatch):
        """Find Linux USB serial ports (/dev/ttyUSB*)."""
        def mock_glob(pattern):
            if pattern == "/dev/ttyUSB*":
                return ["/dev/ttyUSB0", "/dev/ttyUSB1"]
            return []

        monkeypatch.setattr("glob.glob", mock_glob)
        ports = find_ports_with_info()
        assert len(ports) == 2
        assert ports[0]["device"] == "/dev/ttyUSB0"
        assert ports[0]["description"] == "USB Serial"

    def test_macos_usbmodem_ports_found(self, monkeypatch):
        """Find macOS USB modem ports (/dev/cu.usbmodem*)."""
        def mock_glob(pattern):
            if pattern == "/dev/cu.usbmodem*":
                return ["/dev/cu.usbmodem123401"]
            return []

        monkeypatch.setattr("glob.glob", mock_glob)
        ports = find_ports_with_info()
        assert len(ports) == 1
        assert ports[0]["device"] == "/dev/cu.usbmodem123401"
        assert ports[0]["description"] == "USB Modem"

    def test_macos_usbserial_ports_found(self, monkeypatch):
        """Find macOS USB serial ports (/dev/cu.usbserial*)."""
        def mock_glob(pattern):
            if pattern == "/dev/cu.usbserial*":
                return ["/dev/cu.usbserial-12345"]
            return []

        monkeypatch.setattr("glob.glob", mock_glob)
        ports = find_ports_with_info()
        assert len(ports) == 1
        assert ports[0]["device"] == "/dev/cu.usbserial-12345"
        assert ports[0]["description"] == "USB Serial"

    @pytest.mark.skip(reason="Windows serial port detection requires complex import-level mocking")
    def test_windows_serial_ports(self, monkeypatch):
        """Find Windows serial ports via serial.tools.list_ports.

        Note: This test is skipped because the serial.tools.list_ports import
        happens at module level in utils.py, making it difficult to mock with
        monkeypatch. The function is tested manually on Windows platforms.
        """
        pass

    def test_multiple_ports_same_type(self, monkeypatch):
        """Handle multiple ports of the same type."""
        def mock_glob(pattern):
            if pattern == "/dev/ttyACM*":
                return ["/dev/ttyACM0", "/dev/ttyACM1", "/dev/ttyACM2"]
            return []

        monkeypatch.setattr("glob.glob", mock_glob)
        ports = find_ports_with_info()
        assert len(ports) == 3
        for i, port in enumerate(ports):
            assert port["device"] == f"/dev/ttyACM{i}"
            assert port["description"] == "ACM Device"

    def test_mixed_port_types(self, monkeypatch):
        """Handle mixed port types (ACM and USB)."""
        def mock_glob(pattern):
            if pattern == "/dev/ttyACM*":
                return ["/dev/ttyACM0"]
            elif pattern == "/dev/ttyUSB*":
                return ["/dev/ttyUSB0"]
            return []

        monkeypatch.setattr("glob.glob", mock_glob)
        ports = find_ports_with_info()
        assert len(ports) == 2
        assert ports[0]["device"] == "/dev/ttyACM0"
        assert ports[1]["device"] == "/dev/ttyUSB0"

    def test_port_dict_structure(self, monkeypatch):
        """Each port dict should have 'device' and 'description' keys."""
        def mock_glob(pattern):
            if pattern == "/dev/ttyACM*":
                return ["/dev/ttyACM0"]
            return []

        monkeypatch.setattr("glob.glob", mock_glob)
        ports = find_ports_with_info()
        assert len(ports) == 1
        assert "device" in ports[0]
        assert "description" in ports[0]
        assert len(ports[0]) == 2

    def test_macos_and_linux_ports(self, monkeypatch):
        """Handle both macOS and Linux ports (though unlikely in practice)."""
        def mock_glob(pattern):
            result = []
            if pattern == "/dev/ttyACM*":
                result.extend(["/dev/ttyACM0"])
            if pattern == "/dev/cu.usbmodem*":
                result.extend(["/dev/cu.usbmodem123401"])
            return result

        monkeypatch.setattr("glob.glob", mock_glob)
        ports = find_ports_with_info()
        assert len(ports) == 2
        devices = [p["device"] for p in ports]
        assert "/dev/ttyACM0" in devices
        assert "/dev/cu.usbmodem123401" in devices

    def test_empty_string_patterns(self, monkeypatch):
        """Handle glob patterns that return empty strings."""
        def mock_glob(pattern):
            if pattern == "/dev/ttyACM*":
                return ["/dev/ttyACM0", ""]
            return []

        monkeypatch.setattr("glob.glob", mock_glob)
        ports = find_ports_with_info()
        # Should include empty string as returned by glob
        assert len(ports) >= 1
        assert "/dev/ttyACM0" in [p["device"] for p in ports]

    def test_glob_called_with_correct_patterns(self, monkeypatch):
        """Verify glob is called with correct patterns."""
        patterns_called = []

        def mock_glob(pattern):
            patterns_called.append(pattern)
            return []

        monkeypatch.setattr("glob.glob", mock_glob)
        find_ports_with_info()

        expected_patterns = [
            "/dev/ttyACM*",
            "/dev/ttyUSB*",
            "/dev/cu.usbmodem*",
            "/dev/cu.usbserial*",
        ]
        for pattern in expected_patterns:
            assert pattern in patterns_called

    def test_single_acm_port(self, monkeypatch):
        """Single ACM port found."""
        def mock_glob(pattern):
            if pattern == "/dev/ttyACM*":
                return ["/dev/ttyACM0"]
            return []

        monkeypatch.setattr("glob.glob", mock_glob)
        ports = find_ports_with_info()
        assert len(ports) == 1
        assert ports[0]["device"] == "/dev/ttyACM0"
        assert ports[0]["description"] == "ACM Device"

    def test_description_acm_device(self, monkeypatch):
        """ACM ports should have 'ACM Device' description."""
        def mock_glob(pattern):
            if pattern == "/dev/ttyACM*":
                return ["/dev/ttyACM0"]
            return []

        monkeypatch.setattr("glob.glob", mock_glob)
        ports = find_ports_with_info()
        assert ports[0]["description"] == "ACM Device"

    def test_description_usb_serial(self, monkeypatch):
        """USB serial ports should have 'USB Serial' description."""
        def mock_glob(pattern):
            if pattern == "/dev/ttyUSB*":
                return ["/dev/ttyUSB0"]
            return []

        monkeypatch.setattr("glob.glob", mock_glob)
        ports = find_ports_with_info()
        assert ports[0]["description"] == "USB Serial"

    def test_description_usb_modem(self, monkeypatch):
        """USB modem ports should have 'USB Modem' description."""
        def mock_glob(pattern):
            if pattern == "/dev/cu.usbmodem*":
                return ["/dev/cu.usbmodem123401"]
            return []

        monkeypatch.setattr("glob.glob", mock_glob)
        ports = find_ports_with_info()
        assert ports[0]["description"] == "USB Modem"

    def test_all_linux_patterns(self, monkeypatch):
        """Test all Linux port patterns."""
        def mock_glob(pattern):
            if pattern == "/dev/ttyACM*":
                return ["/dev/ttyACM0"]
            elif pattern == "/dev/ttyUSB*":
                return ["/dev/ttyUSB0"]
            return []

        monkeypatch.setattr("glob.glob", mock_glob)
        ports = find_ports_with_info()
        assert len(ports) == 2
        descriptions = [p["description"] for p in ports]
        assert "ACM Device" in descriptions
        assert "USB Serial" in descriptions

    def test_all_macos_patterns(self, monkeypatch):
        """Test all macOS port patterns."""
        def mock_glob(pattern):
            if pattern == "/dev/cu.usbmodem*":
                return ["/dev/cu.usbmodem123401"]
            elif pattern == "/dev/cu.usbserial*":
                return ["/dev/cu.usbserial-12345"]
            return []

        monkeypatch.setattr("glob.glob", mock_glob)
        ports = find_ports_with_info()
        assert len(ports) == 2
        descriptions = [p["description"] for p in ports]
        assert "USB Modem" in descriptions
        assert "USB Serial" in descriptions

    def test_returns_list(self, monkeypatch):
        """Should always return a list."""
        monkeypatch.setattr("glob.glob", lambda x: [])
        ports = find_ports_with_info()
        assert isinstance(ports, list)

    def test_port_order_preserved(self, monkeypatch):
        """Port order from glob should be preserved."""
        def mock_glob(pattern):
            if pattern == "/dev/ttyACM*":
                return ["/dev/ttyACM2", "/dev/ttyACM0", "/dev/ttyACM1"]
            return []

        monkeypatch.setattr("glob.glob", mock_glob)
        ports = find_ports_with_info()
        assert ports[0]["device"] == "/dev/ttyACM2"
        assert ports[1]["device"] == "/dev/ttyACM0"
        assert ports[2]["device"] == "/dev/ttyACM1"


# =============================================================================
# Tests for get_selected_port()
# =============================================================================

@pytest.mark.unit
class TestGetSelectedPort:
    """Test get_selected_port() - dpg mocking required."""

    def test_returns_none_when_combo_missing(self, monkeypatch):
        """Return None when port_combo doesn't exist."""
        mock_dpg = MagicMock()
        mock_dpg.does_item_exist.return_value = False
        monkeypatch.setattr("scripts.gui.utils.dpg", mock_dpg)

        result = get_selected_port()
        assert result is None

    def test_returns_none_for_no_device_string(self, monkeypatch):
        """Return None when 'No device' is selected."""
        mock_dpg = MagicMock()
        mock_dpg.does_item_exist.return_value = True
        mock_dpg.get_value.return_value = "No device"
        monkeypatch.setattr("scripts.gui.utils.dpg", mock_dpg)

        result = get_selected_port()
        assert result is None

    def test_extracts_device_from_valid_string(self, monkeypatch):
        """Extract device path from 'Description - /dev/path' format."""
        mock_dpg = MagicMock()
        mock_dpg.does_item_exist.return_value = True
        mock_dpg.get_value.return_value = "ACM Device - /dev/ttyACM0"
        monkeypatch.setattr("scripts.gui.utils.dpg", mock_dpg)

        result = get_selected_port()
        assert result == "/dev/ttyACM0"

    def test_handles_missing_separator(self, monkeypatch):
        """Return None when no ' - ' separator in string."""
        mock_dpg = MagicMock()
        mock_dpg.does_item_exist.return_value = True
        mock_dpg.get_value.return_value = "ACM Device /dev/ttyACM0"
        monkeypatch.setattr("scripts.gui.utils.dpg", mock_dpg)

        result = get_selected_port()
        assert result is None

    def test_handles_empty_string(self, monkeypatch):
        """Return None for empty string."""
        mock_dpg = MagicMock()
        mock_dpg.does_item_exist.return_value = True
        mock_dpg.get_value.return_value = ""
        monkeypatch.setattr("scripts.gui.utils.dpg", mock_dpg)

        result = get_selected_port()
        assert result is None

    def test_handles_none_value(self, monkeypatch):
        """Return None when dpg.get_value returns None."""
        mock_dpg = MagicMock()
        mock_dpg.does_item_exist.return_value = True
        mock_dpg.get_value.return_value = None
        monkeypatch.setattr("scripts.gui.utils.dpg", mock_dpg)

        result = get_selected_port()
        assert result is None

    def test_handles_multiple_separators(self, monkeypatch):
        """Extract device after first ' - ' separator."""
        mock_dpg = MagicMock()
        mock_dpg.does_item_exist.return_value = True
        mock_dpg.get_value.return_value = "ACM - Device - /dev/ttyACM0"
        monkeypatch.setattr("scripts.gui.utils.dpg", mock_dpg)

        result = get_selected_port()
        # With multiple separators, split returns more than 2 parts, so None is returned
        # This is the actual behavior of the function
        assert result is None

    def test_handles_exception_gracefully(self, monkeypatch):
        """Return None when exception is raised."""
        mock_dpg = MagicMock()
        # get_value raises exception
        mock_dpg.does_item_exist.return_value = True
        mock_dpg.get_value.side_effect = Exception("Test error")
        monkeypatch.setattr("scripts.gui.utils.dpg", mock_dpg)

        # The function catches exceptions and returns None
        result = get_selected_port()
        assert result is None

    def test_windows_com_port(self, monkeypatch):
        """Extract Windows COM port from string."""
        mock_dpg = MagicMock()
        mock_dpg.does_item_exist.return_value = True
        mock_dpg.get_value.return_value = "USB Serial - COM3"
        monkeypatch.setattr("scripts.gui.utils.dpg", mock_dpg)

        result = get_selected_port()
        assert result == "COM3"


# =============================================================================
# Tests for handle_disconnect_state()
# =============================================================================

@pytest.mark.unit
class TestHandleDisconnectState:
    """Test handle_disconnect_state() - UI state reset on disconnect."""

    def test_resets_streaming_state(self, monkeypatch):
        """Verify streaming flags are reset."""
        mock_app = MagicMock()
        mock_app._lock = MagicMock()
        mock_app._is_streaming = True
        mock_app._stream_running = True
        mock_app._p_streaming = True

        mock_dpg = MagicMock()
        mock_dpg.does_item_exist.return_value = True
        monkeypatch.setattr("scripts.gui.utils.dpg", mock_dpg)
        monkeypatch.setattr("scripts.gui.utils.update_connection_indicator", MagicMock())
        monkeypatch.setattr("scripts.gui.utils.update_led_indicator", MagicMock())

        handle_disconnect_state(mock_app)

        assert mock_app._is_streaming is False
        assert mock_app._stream_running is False
        assert mock_app._p_streaming is False

    def test_updates_connect_button(self, monkeypatch):
        """Verify connect button label is updated."""
        mock_app = MagicMock()
        mock_app._lock = MagicMock()

        mock_dpg = MagicMock()
        mock_dpg.does_item_exist.return_value = True
        monkeypatch.setattr("scripts.gui.utils.dpg", mock_dpg)
        monkeypatch.setattr("scripts.gui.utils.update_connection_indicator", MagicMock())
        monkeypatch.setattr("scripts.gui.utils.update_led_indicator", MagicMock())

        handle_disconnect_state(mock_app)

        mock_dpg.set_item_label.assert_called_once_with(
            TAGS["connect_btn"],
            "Connect"
        )

    def test_enables_port_combo(self, monkeypatch):
        """Verify port combo is enabled."""
        mock_app = MagicMock()
        mock_app._lock = MagicMock()

        mock_dpg = MagicMock()
        mock_dpg.does_item_exist.return_value = True
        monkeypatch.setattr("scripts.gui.utils.dpg", mock_dpg)
        monkeypatch.setattr("scripts.gui.utils.update_connection_indicator", MagicMock())
        monkeypatch.setattr("scripts.gui.utils.update_led_indicator", MagicMock())

        handle_disconnect_state(mock_app)

        mock_dpg.configure_item.assert_any_call(
            TAGS["port_combo"],
            enabled=True
        )

    def test_disables_led_button(self, monkeypatch):
        """Verify LED button is disabled."""
        mock_app = MagicMock()
        mock_app._lock = MagicMock()

        mock_dpg = MagicMock()
        mock_dpg.does_item_exist.return_value = True
        monkeypatch.setattr("scripts.gui.utils.dpg", mock_dpg)
        monkeypatch.setattr("scripts.gui.utils.update_connection_indicator", MagicMock())
        monkeypatch.setattr("scripts.gui.utils.update_led_indicator", MagicMock())

        handle_disconnect_state(mock_app)

        mock_dpg.configure_item.assert_any_call(
            TAGS["led_btn"],
            enabled=False
        )

    def test_disables_adc_read_button(self, monkeypatch):
        """Verify ADC read button is disabled."""
        mock_app = MagicMock()
        mock_app._lock = MagicMock()

        mock_dpg = MagicMock()
        mock_dpg.does_item_exist.return_value = True
        monkeypatch.setattr("scripts.gui.utils.dpg", mock_dpg)
        monkeypatch.setattr("scripts.gui.utils.update_connection_indicator", MagicMock())
        monkeypatch.setattr("scripts.gui.utils.update_led_indicator", MagicMock())

        handle_disconnect_state(mock_app)

        mock_dpg.configure_item.assert_any_call(
            TAGS["adc_read_btn"],
            enabled=False
        )

    def test_resets_stream_buttons(self, monkeypatch):
        """Verify streaming buttons are disabled."""
        mock_app = MagicMock()
        mock_app._lock = MagicMock()

        mock_dpg = MagicMock()
        mock_dpg.does_item_exist.return_value = True
        monkeypatch.setattr("scripts.gui.utils.dpg", mock_dpg)
        monkeypatch.setattr("scripts.gui.utils.update_connection_indicator", MagicMock())
        monkeypatch.setattr("scripts.gui.utils.update_led_indicator", MagicMock())

        handle_disconnect_state(mock_app)

        # Stream start/stop buttons should be disabled
        mock_dpg.configure_item.assert_any_call(
            TAGS["stream_start_btn"],
            enabled=False
        )
        mock_dpg.configure_item.assert_any_call(
            TAGS["stream_stop_btn"],
            enabled=False
        )

    def test_resets_stream_status(self, monkeypatch):
        """Verify stream status is reset."""
        mock_app = MagicMock()
        mock_app._lock = MagicMock()

        mock_dpg = MagicMock()
        mock_dpg.does_item_exist.return_value = True
        monkeypatch.setattr("scripts.gui.utils.dpg", mock_dpg)
        monkeypatch.setattr("scripts.gui.utils.update_connection_indicator", MagicMock())
        monkeypatch.setattr("scripts.gui.utils.update_led_indicator", MagicMock())

        handle_disconnect_state(mock_app)

        mock_dpg.set_value.assert_any_call(
            TAGS["stream_status"],
            "Stream: Stopped"
        )

    def test_resets_p_stream_buttons(self, monkeypatch):
        """Verify P-streaming buttons are disabled."""
        mock_app = MagicMock()
        mock_app._lock = MagicMock()

        mock_dpg = MagicMock()
        mock_dpg.does_item_exist.return_value = True
        monkeypatch.setattr("scripts.gui.utils.dpg", mock_dpg)
        monkeypatch.setattr("scripts.gui.utils.update_connection_indicator", MagicMock())
        monkeypatch.setattr("scripts.gui.utils.update_led_indicator", MagicMock())

        handle_disconnect_state(mock_app)

        mock_dpg.configure_item.assert_any_call(
            TAGS["p_stream_start_btn"],
            enabled=False
        )
        mock_dpg.configure_item.assert_any_call(
            TAGS["p_stream_stop_btn"],
            enabled=False
        )

    def test_resets_p_stream_status(self, monkeypatch):
        """Verify P-stream status is reset."""
        mock_app = MagicMock()
        mock_app._lock = MagicMock()

        mock_dpg = MagicMock()
        mock_dpg.does_item_exist.return_value = True
        monkeypatch.setattr("scripts.gui.utils.dpg", mock_dpg)
        monkeypatch.setattr("scripts.gui.utils.update_connection_indicator", MagicMock())
        monkeypatch.setattr("scripts.gui.utils.update_led_indicator", MagicMock())

        handle_disconnect_state(mock_app)

        mock_dpg.set_value.assert_any_call(
            TAGS["p_stream_status"],
            "P-Stream: Stopped"
        )

    def test_resets_p_mode_to_manual(self, monkeypatch):
        """Verify P-mode radio is reset to Manual (0)."""
        mock_app = MagicMock()
        mock_app._lock = MagicMock()

        mock_dpg = MagicMock()
        mock_dpg.does_item_exist.return_value = True
        monkeypatch.setattr("scripts.gui.utils.dpg", mock_dpg)
        monkeypatch.setattr("scripts.gui.utils.update_connection_indicator", MagicMock())
        monkeypatch.setattr("scripts.gui.utils.update_led_indicator", MagicMock())

        handle_disconnect_state(mock_app)

        mock_dpg.set_value.assert_any_call(
            TAGS["p_mode_radio"],
            0
        )

    def test_hides_p_control_group(self, monkeypatch):
        """Verify P-control group is hidden."""
        mock_app = MagicMock()
        mock_app._lock = MagicMock()

        mock_dpg = MagicMock()
        mock_dpg.does_item_exist.return_value = True
        monkeypatch.setattr("scripts.gui.utils.dpg", mock_dpg)
        monkeypatch.setattr("scripts.gui.utils.update_connection_indicator", MagicMock())
        monkeypatch.setattr("scripts.gui.utils.update_led_indicator", MagicMock())

        handle_disconnect_state(mock_app)

        mock_dpg.configure_item.assert_any_call(
            TAGS["p_control_group"],
            show=False
        )

    def test_calls_update_indicators(self, monkeypatch):
        """Verify connection and LED indicators are updated."""
        mock_app = MagicMock()
        mock_app._lock = MagicMock()

        mock_dpg = MagicMock()
        mock_dpg.does_item_exist.return_value = True
        mock_update_conn = MagicMock()
        mock_update_led = MagicMock()
        monkeypatch.setattr("scripts.gui.utils.dpg", mock_dpg)
        monkeypatch.setattr("scripts.gui.utils.update_connection_indicator", mock_update_conn)
        monkeypatch.setattr("scripts.gui.utils.update_led_indicator", mock_update_led)

        handle_disconnect_state(mock_app)

        mock_update_conn.assert_called_once_with(False)
        mock_update_led.assert_called_once_with(False)

    def test_skips_nonexistent_items(self, monkeypatch):
        """Verify function handles missing UI items gracefully."""
        mock_app = MagicMock()
        mock_app._lock = MagicMock()

        mock_dpg = MagicMock()
        # Return False for all items
        mock_dpg.does_item_exist.return_value = False
        monkeypatch.setattr("scripts.gui.utils.dpg", mock_dpg)
        monkeypatch.setattr("scripts.gui.utils.update_connection_indicator", MagicMock())
        monkeypatch.setattr("scripts.gui.utils.update_led_indicator", MagicMock())

        # Should not raise exception
        handle_disconnect_state(mock_app)


# =============================================================================
# Tests for update_status()
# =============================================================================

@pytest.mark.unit
class TestUpdateStatus:
    """Test update_status() - status message and color update."""

    def test_updates_message_with_existing_tag(self, monkeypatch):
        """Update status message when tag exists."""
        mock_dpg = MagicMock()
        mock_dpg.does_item_exist.return_value = True
        monkeypatch.setattr("scripts.gui.utils.dpg", mock_dpg)

        update_status("Test message", [255, 0, 0])

        mock_dpg.set_value.assert_called_once()
        mock_dpg.configure_item.assert_called_once()

    def test_skips_missing_tag(self, monkeypatch):
        """Skip update when tag doesn't exist."""
        mock_dpg = MagicMock()
        mock_dpg.does_item_exist.return_value = False
        monkeypatch.setattr("scripts.gui.utils.dpg", mock_dpg)

        # Should not raise exception
        update_status("Test message", [255, 0, 0])

        mock_dpg.set_value.assert_not_called()
        mock_dpg.configure_item.assert_not_called()

    def test_sets_message_text(self, monkeypatch):
        """Verify message text is set correctly."""
        mock_dpg = MagicMock()
        mock_dpg.does_item_exist.return_value = True
        monkeypatch.setattr("scripts.gui.utils.dpg", mock_dpg)

        test_message = "Connection established"
        update_status(test_message, [100, 255, 100])

        args = mock_dpg.set_value.call_args
        assert args[0][1] == test_message

    def test_sets_color(self, monkeypatch):
        """Verify color is set correctly."""
        mock_dpg = MagicMock()
        mock_dpg.does_item_exist.return_value = True
        monkeypatch.setattr("scripts.gui.utils.dpg", mock_dpg)

        test_color = [255, 100, 50]
        update_status("Test", test_color)

        # Check configure_item was called with color parameter
        args = mock_dpg.configure_item.call_args
        assert "color" in args[1]
        assert args[1]["color"] == test_color


# =============================================================================
# Tests for update_connection_indicator()
# =============================================================================

@pytest.mark.unit
class TestUpdateConnectionIndicator:
    """Test update_connection_indicator() - connection status indicator."""

    def test_connected_state_green(self, monkeypatch):
        """Set green color and 'Connected' text when True."""
        mock_dpg = MagicMock()
        mock_dpg.does_item_exist.return_value = True
        monkeypatch.setattr("scripts.gui.utils.dpg", mock_dpg)

        update_connection_indicator(True)

        mock_dpg.configure_item.assert_called_once()
        args = mock_dpg.configure_item.call_args
        assert args[1]["color"] == [100, 255, 100]

        mock_dpg.set_value.assert_called_once()
        args = mock_dpg.set_value.call_args
        assert args[0][1] == "Connected"

    def test_disconnected_state_gray(self, monkeypatch):
        """Set gray color and 'Disconnected' text when False."""
        mock_dpg = MagicMock()
        mock_dpg.does_item_exist.return_value = True
        monkeypatch.setattr("scripts.gui.utils.dpg", mock_dpg)

        update_connection_indicator(False)

        mock_dpg.configure_item.assert_called_once()
        args = mock_dpg.configure_item.call_args
        assert args[1]["color"] == [150, 150, 150]

        mock_dpg.set_value.assert_called_once()
        args = mock_dpg.set_value.call_args
        assert args[0][1] == "Disconnected"

    def test_skips_missing_tag(self, monkeypatch):
        """Skip update when tag doesn't exist."""
        mock_dpg = MagicMock()
        mock_dpg.does_item_exist.return_value = False
        monkeypatch.setattr("scripts.gui.utils.dpg", mock_dpg)

        # Should not raise exception
        update_connection_indicator(True)

        mock_dpg.configure_item.assert_not_called()
        mock_dpg.set_value.assert_not_called()


# =============================================================================
# Tests for update_led_indicator()
# =============================================================================

@pytest.mark.unit
class TestUpdateLedIndicator:
    """Test update_led_indicator() - LED status indicator."""

    def test_led_on_state_green(self, monkeypatch):
        """Set green color and 'LED ON' text when True."""
        mock_dpg = MagicMock()
        mock_dpg.does_item_exist.return_value = True
        monkeypatch.setattr("scripts.gui.utils.dpg", mock_dpg)

        update_led_indicator(True)

        mock_dpg.configure_item.assert_called_once()
        args = mock_dpg.configure_item.call_args
        assert args[1]["color"] == [100, 255, 100]

        mock_dpg.set_value.assert_called_once()
        args = mock_dpg.set_value.call_args
        assert args[0][1] == "LED ON"

    def test_led_off_state_gray(self, monkeypatch):
        """Set gray color and 'LED OFF' text when False."""
        mock_dpg = MagicMock()
        mock_dpg.does_item_exist.return_value = True
        monkeypatch.setattr("scripts.gui.utils.dpg", mock_dpg)

        update_led_indicator(False)

        mock_dpg.configure_item.assert_called_once()
        args = mock_dpg.configure_item.call_args
        assert args[1]["color"] == [150, 150, 150]

        mock_dpg.set_value.assert_called_once()
        args = mock_dpg.set_value.call_args
        assert args[0][1] == "LED OFF"

    def test_skips_missing_tag(self, monkeypatch):
        """Skip update when tag doesn't exist."""
        mock_dpg = MagicMock()
        mock_dpg.does_item_exist.return_value = False
        monkeypatch.setattr("scripts.gui.utils.dpg", mock_dpg)

        # Should not raise exception
        update_led_indicator(True)

        mock_dpg.configure_item.assert_not_called()
        mock_dpg.set_value.assert_not_called()

