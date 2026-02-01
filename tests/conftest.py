"""
Shared fixtures and configuration for pytest.

This file contains common fixtures used across multiple test files.
"""

import sys
import os
from pathlib import Path
import pytest

# Add project root to Python path for imports
project_root = Path(__file__).parent.parent
sys.path.insert(0, str(project_root))


# =============================================================================
# Test Constants
# =============================================================================

# ADC to current conversion tolerances
ADC_CURRENT_TOLERANCE = 0.01
PRECISION_THRESHOLD_PCT = 0.5
ROUNDTRIP_ERROR_THRESHOLD_PCT = 1.0


def pytest_configure(config):
    """Configure pytest with custom markers."""
    config.addinivalue_line(
        "markers", "unit: Unit tests (fast, isolated)"
    )
    config.addinivalue_line(
        "markers", "integration: Integration tests (slower, may use external resources)"
    )
    config.addinivalue_line(
        "markers", "slow: Slow tests (run with -m not slow to skip)"
    )


# =============================================================================
# Fixtures
# =============================================================================

@pytest.fixture
def sample_adc_values():
    """Provide sample ADC values for testing."""
    return [0, 100, 500, 1000, 1500, 2000, 2500, 3000, 3500, 4000, 4095]


@pytest.fixture
def sample_currents():
    """Provide sample current values (in amps) for testing."""
    return [0.0, 0.5, 1.0, 1.5, 2.0, 2.5, 3.0, 4.0, 5.0, 6.0]


@pytest.fixture
def valid_frame_builder():
    """Provide a function to build valid frames for testing."""
    def _build(cmd, value=None):
        """Build a valid frame with correct CRC."""
        if value is None:
            # 2-byte frame (command only)
            data = bytes([cmd])
        elif isinstance(value, int) and value > 255:
            # 4-byte frame (16-bit value)
            high = (value >> 8) & 0xFF
            low = value & 0xFF
            data = bytes([cmd, high, low])
        else:
            # 3-byte frame (8-bit value)
            data = bytes([cmd, value])
        return data + bytes([crc8(data)])

    # Import crc8 locally to avoid circular imports
    from scripts.protocol import crc8
    return _build


@pytest.fixture
def mock_dpg(monkeypatch):
    """
    Mock dearpygui.dearpygui module for GUI testing.

    Provides a minimal mock that can simulate item existence, values,
    and configuration without requiring the actual Dear PyGui library.
    """
    from unittest.mock import MagicMock

    class MockDPGContext:
        def __init__(self):
            self._items = {}
            self._values = {}
            self._config = {}

        def does_item_exist(self, tag):
            return tag in self._items

        def add_item(self, tag, **kwargs):
            self._items[tag] = kwargs
            self._values[tag] = kwargs.get('default_value', None)

        def get_value(self, tag):
            return self._values.get(tag)

        def set_value(self, tag, value):
            if tag in self._items:
                self._values[tag] = value

        def configure_item(self, tag, **kwargs):
            if tag in self._items:
                for key, value in kwargs.items():
                    self._config.setdefault(tag, {})[key] = value

        def set_item_label(self, tag, label):
            self.configure_item(tag, label=label)

    mock_dpg = MockDPGContext()

    # Monkeypatch the import
    import sys
    sys.modules['dearpygui'] = MagicMock()
    sys.modules['dearpygui.dearpygui'] = mock_dpg

    return mock_dpg


# =============================================================================
# GUI App and Serial Mocking Fixtures (Phase 2c)
# =============================================================================

@pytest.fixture
def mock_serial_connection():
    """Mock serial.Serial connection for testing handlers.

    Provides a controllable mock that:
    - Simulates open/close state
    - Tracks written data
    - Allows queuing responses to be read
    - Raises SerialException on errors
    """
    import queue

    class MockSerial:
        def __init__(self, port=None, baudrate=115200, timeout=1, write_timeout=1):
            self.port = port or "/dev/ttyACM0"
            self.baudrate = baudrate
            self.timeout = timeout
            self.write_timeout = write_timeout
            self.is_open = False
            self.written_data = []
            self._read_buffer = b''
            self._should_raise_on_write = False
            self._write_error_msg = ""

        def open(self):
            self.is_open = True

        def close(self):
            self.is_open = False

        def write(self, data):
            if not self.is_open:
                import serial
                raise serial.SerialException("Port not open")
            if self._should_raise_on_write:
                import serial
                raise serial.SerialException(self._write_error_msg)
            self.written_data.append(bytes(data))
            return len(data)

        def flush(self):
            pass

        def read(self, n=1):
            if not self.is_open or not self._read_buffer:
                return b''
            result = self._read_buffer[:n]
            self._read_buffer = self._read_buffer[n:]
            return result

        def queue_read_bytes(self, data):
            """Queue bytes to be read (for testing responses)."""
            self._read_buffer += bytes(data)

        def reset_input_buffer(self):
            self._read_buffer = b''

        def reset_output_buffer(self):
            self.written_data = []

        def simulate_write_error(self, message="Simulated write error"):
            """Make next write() call raise SerialException."""
            self._should_raise_on_write = True
            self._write_error_msg = message

        def clear_write_error(self):
            """Clear write error simulation."""
            self._should_raise_on_write = False
            self._write_error_msg = ""

    return MockSerial()


@pytest.fixture
def led_app():
    """Provide LEDControllerApp instance.

    The app is created fresh for each test and shutdown after.
    Worker threads are started but not connected to serial.
    """
    from scripts.gui.app import LEDControllerApp
    app = LEDControllerApp()
    yield app
    app.shutdown()


@pytest.fixture
def led_app_with_mock_serial(led_app, mock_serial_connection):
    """Provide app with mocked serial connection.

    Sets up the app with an open mock serial connection, ready for
    testing handler functions without actual hardware.

    Usage:
        app, mock_serial = led_app_with_mock_serial
        # Queue response bytes
        mock_serial.queue_read_bytes([0xFF])  # ACK
        # Run handler
        result = app._handle_send_led(True)
    """
    # Set up mock serial connection on the app
    led_app._serial_connection = mock_serial_connection
    mock_serial_connection.open()
    with led_app._lock:
        led_app._is_connected = True

    yield led_app, mock_serial_connection

    # Cleanup
    with led_app._lock:
        if mock_serial_connection.is_open:
            mock_serial_connection.close()
        led_app._serial_connection = None
        led_app._is_connected = False
        led_app._led_state = False
        led_app._pwm_duty = 0
        led_app._is_streaming = False
        led_app._p_streaming = False
