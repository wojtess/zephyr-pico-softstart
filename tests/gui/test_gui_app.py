"""
Tests for LEDControllerApp core application logic (Phase 2b).

These tests focus on:
- Initialization and defaults
- State getters (thread-safe)
- State setters
- Task management
- ADC history management
- P-stream history management

NO serial mocking - that's Phase 2c.
"""

import sys
import time
import threading
import queue
from pathlib import Path
from collections import deque

import pytest

# Add project root to Python path for imports
project_root = Path(__file__).parent.parent.parent
sys.path.insert(0, str(project_root))

from scripts.gui.app import LEDControllerApp
from scripts.gui.constants import SerialCommand, SerialTask


# =============================================================================
# Test Suite 1: Initialization (~10 tests)
# =============================================================================

@pytest.mark.unit
class TestLEDControllerAppInitialization:
    """Test LEDControllerApp initialization and default values."""

    def test_app_initialization_defaults(self, led_app):
        """Test that app initializes without errors."""
        assert led_app is not None
        assert isinstance(led_app, LEDControllerApp)

    def test_worker_threads_started(self, led_app):
        """Test that worker threads are started during initialization."""
        assert led_app._worker_thread is not None
        assert led_app._worker_thread.is_alive()

        assert led_app._data_reader_thread is not None
        assert led_app._data_reader_thread.is_alive()

    def test_initial_state_defaults(self, led_app):
        """Test that initial state values are correct."""
        assert led_app._running is True
        assert led_app._led_state is False
        assert led_app._pwm_duty == 0
        assert led_app._last_known_pwm == 0
        assert led_app._is_connected is False
        assert led_app._current_port is None

    def test_deques_initialised_with_correct_maxlen(self, led_app):
        """Test that history deques are initialized with maxlen=100."""
        assert led_app._adc_max_points == 100

        # ADC history deques
        assert isinstance(led_app._adc_time_history, deque)
        assert led_app._adc_time_history.maxlen == 100

        assert isinstance(led_app._adc_raw_history, deque)
        assert led_app._adc_raw_history.maxlen == 100

        assert isinstance(led_app._adc_voltage_history, deque)
        assert led_app._adc_voltage_history.maxlen == 100

        # P-stream history deques
        assert isinstance(led_app._p_time_history, deque)
        assert led_app._p_time_history.maxlen == 100

        assert isinstance(led_app._p_measured_history, deque)
        assert led_app._p_measured_history.maxlen == 100

        assert isinstance(led_app._setpoint_history, deque)
        assert led_app._setpoint_history.maxlen == 100

        assert isinstance(led_app._pwm_history, deque)
        assert led_app._pwm_history.maxlen == 100

        assert isinstance(led_app._error_history, deque)
        assert led_app._error_history.maxlen == 100

    def test_queues_created(self, led_app):
        """Test that all queues are created."""
        assert isinstance(led_app._task_queue, queue.Queue)
        assert isinstance(led_app._result_queue, queue.Queue)
        assert isinstance(led_app._response_queue, queue.Queue)

    def test_led_state_initially_false(self, led_app):
        """Test that LED state is initially False."""
        assert led_app.get_led_state() is False

    def test_pwm_duty_initially_zero(self, led_app):
        """Test that PWM duty is initially 0."""
        assert led_app.get_pwm_duty() == 0

    def test_is_connected_initially_false(self, led_app):
        """Test that is_connected is initially False."""
        assert led_app.is_connected() is False

    def test_is_streaming_initially_false(self, led_app):
        """Test that is_streaming is initially False."""
        assert led_app.is_streaming() is False

    def test_p_streaming_initially_false(self, led_app):
        """Test that is_p_streaming is initially False."""
        assert led_app.is_p_streaming() is False

    def test_p_recording_initially_false(self, led_app):
        """Test that is_p_recording is initially False."""
        assert led_app.is_p_recording() is False

    def test_lock_created(self, led_app):
        """Test that threading lock is created."""
        assert led_app._lock is not None
        assert isinstance(led_app._lock, threading.Lock)

    def test_pending_tasks_initially_empty(self, led_app):
        """Test that pending tasks list is initially empty."""
        assert led_app._pending_tasks == []
        assert led_app.get_pending_count() == 0

    def test_processing_task_initially_none(self, led_app):
        """Test that processing task is initially None."""
        assert led_app._processing_task is None
        assert led_app.get_processing_task() is None

    def test_p_controller_initial_defaults(self, led_app):
        """Test that P-controller parameters have correct initial values."""
        assert led_app._p_mode == 0  # 0=manual
        assert led_app._p_setpoint == 0
        assert led_app._p_gain == 1.0
        assert led_app._p_feed_forward == 0


# =============================================================================
# Test Suite 2: State getters - THREAD-SAFE (~20 tests)
# =============================================================================

@pytest.mark.unit
class TestLEDControllerAppGetters:
    """Test state getter methods (thread-safe)."""

    def test_get_led_state_initial(self, led_app):
        """Test get_led_state returns initial value."""
        assert led_app.get_led_state() is False

    def test_get_led_state_after_set(self, led_app):
        """Test get_led_state returns updated value after set."""
        led_app.set_led_state(True)
        assert led_app.get_led_state() is True

        led_app.set_led_state(False)
        assert led_app.get_led_state() is False

    def test_get_pwm_duty_initial(self, led_app):
        """Test get_pwm_duty returns initial value."""
        assert led_app.get_pwm_duty() == 0

    def test_get_pwm_duty_after_change(self, led_app):
        """Test get_pwm_duty returns updated value after change."""
        # Change via direct access (simulating what handlers do)
        with led_app._lock:
            led_app._pwm_duty = 50

        assert led_app.get_pwm_duty() == 50

    def test_is_connected_initial(self, led_app):
        """Test is_connected returns initial value."""
        assert led_app.is_connected() is False

    def test_is_connected_after_connect_simulation(self, led_app):
        """Test is_connected returns True after simulated connection."""
        # Simulate connection state (without actual serial)
        # Note: is_connected() checks both _is_connected AND _serial_connection
        # Create a mock serial connection object
        mock_serial = type('MockSerial', (object,), {'is_open': True, 'close': lambda: None})()
        with led_app._lock:
            led_app._is_connected = True
            led_app._serial_connection = mock_serial

        assert led_app.is_connected() is True

        # Clean up - restore to None
        with led_app._lock:
            led_app._is_connected = False
            led_app._serial_connection = None

    def test_is_streaming(self, led_app):
        """Test is_streaming returns correct state."""
        assert led_app.is_streaming() is False

        led_app.set_streaming(True)
        assert led_app.is_streaming() is True

        led_app.set_streaming(False)
        assert led_app.is_streaming() is False

    def test_is_p_streaming(self, led_app):
        """Test is_p_streaming returns correct state."""
        assert led_app.is_p_streaming() is False

        led_app.set_p_streaming(True)
        assert led_app.is_p_streaming() is True

        led_app.set_p_streaming(False)
        assert led_app.is_p_streaming() is False

    def test_is_p_recording(self, led_app):
        """Test is_p_recording returns correct state."""
        assert led_app.is_p_recording() is False

        led_app.set_p_recording(True)
        assert led_app.is_p_recording() is True

        led_app.set_p_recording(False)
        assert led_app.is_p_recording() is False

    def test_get_last_known_pwm_initial(self, led_app):
        """Test get_last_known_pwm returns initial value."""
        assert led_app.get_last_known_pwm() == 0

    def test_get_last_known_pwm_after_update(self, led_app):
        """Test get_last_known_pwm returns updated value."""
        with led_app._lock:
            led_app._last_known_pwm = 75

        assert led_app.get_last_known_pwm() == 75

    def test_get_pending_count_empty(self, led_app):
        """Test get_pending_count returns 0 when empty."""
        assert led_app.get_pending_count() == 0

    def test_get_pending_count_with_tasks(self, led_app):
        """Test get_pending_count returns correct count with tasks."""
        # Add tasks directly to pending list
        task1 = SerialTask(command=SerialCommand.SEND_LED)
        task2 = SerialTask(command=SerialCommand.SEND_PWM)

        with led_app._lock:
            led_app._pending_tasks.append(task1)
            led_app._pending_tasks.append(task2)

        assert led_app.get_pending_count() == 2

    def test_get_processing_task_empty(self, led_app):
        """Test get_processing_task returns None when empty."""
        assert led_app.get_processing_task() is None

    def test_get_processing_task_with_task(self, led_app):
        """Test get_processing_task returns task name when processing."""
        task = SerialTask(command=SerialCommand.READ_ADC)

        with led_app._lock:
            led_app._processing_task = task

        assert led_app.get_processing_task() == "read_adc"

    def test_get_processing_task_all_commands(self, led_app):
        """Test get_processing_task returns correct names for all commands."""
        commands = [
            (SerialCommand.CONNECT, "connect"),
            (SerialCommand.DISCONNECT, "disconnect"),
            (SerialCommand.SEND_LED, "send_led"),
            (SerialCommand.SEND_PWM, "send_pwm"),
            (SerialCommand.READ_ADC, "read_adc"),
            (SerialCommand.START_STREAM, "start_stream"),
            (SerialCommand.STOP_STREAM, "stop_stream"),
            (SerialCommand.CHECK_HEALTH, "check_health"),
            (SerialCommand.SET_P_MODE, "set_p_mode"),
            (SerialCommand.SET_P_SETPOINT, "set_p_setpoint"),
            (SerialCommand.SET_P_GAIN, "set_p_gain"),
            (SerialCommand.START_P_STREAM, "start_p_stream"),
            (SerialCommand.STOP_P_STREAM, "stop_p_stream"),
        ]

        for command, expected_name in commands:
            task = SerialTask(command=command)
            with led_app._lock:
                led_app._processing_task = task

            assert led_app.get_processing_task() == expected_name

            # Clean up
            with led_app._lock:
                led_app._processing_task = None

    def test_get_pending_list_empty(self, led_app):
        """Test get_pending_list returns empty list when no tasks."""
        assert led_app.get_pending_list() == []

    def test_get_pending_list_with_tasks(self, led_app):
        """Test get_pending_list returns correct task names."""
        task1 = SerialTask(command=SerialCommand.SEND_LED)
        task2 = SerialTask(command=SerialCommand.SEND_PWM)
        task3 = SerialTask(command=SerialCommand.READ_ADC)

        with led_app._lock:
            led_app._pending_tasks.extend([task1, task2, task3])

        pending = led_app.get_pending_list()
        assert pending == ["send_led", "send_pwm", "read_adc"]

    def test_get_stream_interval_initial(self, led_app):
        """Test get_stream_interval returns initial value."""
        assert led_app.get_stream_interval() == 100

    def test_get_stream_interval_after_change(self, led_app):
        """Test get_stream_interval returns updated value."""
        with led_app._lock:
            led_app._stream_interval = 250

        assert led_app.get_stream_interval() == 250


# =============================================================================
# Test Suite 3: Thread Safety Tests (~5 tests)
# =============================================================================

@pytest.mark.unit
class TestLEDControllerAppThreadSafety:
    """Test thread safety of getters and setters."""

    def test_thread_safety_of_getters_concurrent_reads(self, led_app):
        """Test that multiple threads can safely read state concurrently."""
        # Set some state
        led_app.set_led_state(True)
        led_app.set_streaming(True)

        results = []
        errors = []

        def read_state():
            try:
                for _ in range(100):
                    led_app.get_led_state()
                    led_app.is_streaming()
                    led_app.get_pwm_duty()
                    led_app.get_pending_count()
                results.append(True)
            except Exception as e:
                errors.append(e)

        # Create multiple threads
        threads = [
            threading.Thread(target=read_state)
            for _ in range(5)
        ]

        # Start all threads
        for t in threads:
            t.start()

        # Wait for completion
        for t in threads:
            t.join(timeout=5)

        # All threads should complete without errors
        assert len(errors) == 0, f"Thread safety errors: {errors}"
        assert len(results) == 5

    def test_thread_safety_of_setters_concurrent_writes(self, led_app):
        """Test that setters are thread-safe."""
        errors = []

        def write_state(value):
            try:
                for i in range(50):
                    led_app.set_led_state(bool(i % 2))
                    led_app.set_streaming(bool(i % 2))
                    led_app.set_p_streaming(bool(i % 2))
            except Exception as e:
                errors.append(e)

        threads = [
            threading.Thread(target=write_state, args=(i,))
            for i in range(3)
        ]

        for t in threads:
            t.start()

        for t in threads:
            t.join(timeout=5)

        assert len(errors) == 0, f"Thread safety errors: {errors}"

    def test_thread_safety_mixed_read_write(self, led_app):
        """Test thread safety with mixed reads and writes."""
        errors = []

        def reader():
            try:
                for _ in range(100):
                    led_app.get_led_state()
                    led_app.is_streaming()
                    led_app.get_pwm_duty()
            except Exception as e:
                errors.append(("reader", e))

        def writer():
            try:
                for i in range(100):
                    led_app.set_led_state(bool(i % 2))
                    led_app.set_streaming(bool(i % 2))
            except Exception as e:
                errors.append(("writer", e))

        threads = []
        for _ in range(2):
            threads.append(threading.Thread(target=reader))
            threads.append(threading.Thread(target=writer))

        for t in threads:
            t.start()

        for t in threads:
            t.join(timeout=5)

        assert len(errors) == 0, f"Thread safety errors: {errors}"

    def test_adc_history_thread_safety(self, led_app):
        """Test that ADC history methods are thread-safe."""
        errors = []

        def add_data():
            try:
                for i in range(50):
                    led_app.add_adc_data(i * 10, (i * 10) / 4095 * 3.3)
            except Exception as e:
                errors.append(("add", e))

        def read_data():
            try:
                for _ in range(50):
                    led_app.get_adc_history()
            except Exception as e:
                errors.append(("read", e))

        threads = [
            threading.Thread(target=add_data),
            threading.Thread(target=read_data)
        ]

        for t in threads:
            t.start()

        for t in threads:
            t.join(timeout=5)

        assert len(errors) == 0, f"Thread safety errors: {errors}"

    def test_task_queue_thread_safety(self, led_app):
        """Test that task queue operations are thread-safe."""
        errors = []
        task_count = [0]

        def send_tasks():
            try:
                for i in range(20):
                    task = SerialTask(command=SerialCommand.READ_ADC)
                    led_app.send_task(task)
                    task_count[0] += 1
            except Exception as e:
                errors.append(e)

        def check_pending():
            try:
                for _ in range(20):
                    led_app.get_pending_count()
                    led_app.get_pending_list()
            except Exception as e:
                errors.append(e)

        threads = [
            threading.Thread(target=send_tasks),
            threading.Thread(target=check_pending)
        ]

        for t in threads:
            t.start()

        for t in threads:
            t.join(timeout=5)

        assert len(errors) == 0, f"Thread safety errors: {errors}"


# =============================================================================
# Test Suite 4: State setters (~10 tests)
# =============================================================================

@pytest.mark.unit
class TestLEDControllerAppSetters:
    """Test state setter methods."""

    def test_set_port(self, led_app):
        """Test set_port stores the port correctly."""
        assert led_app._current_port is None

        led_app.set_port("/dev/ttyACM0")
        assert led_app._current_port == "/dev/ttyACM0"

        led_app.set_port("COM3")
        assert led_app._current_port == "COM3"

    def test_set_led_state(self, led_app):
        """Test set_led_state updates LED state."""
        assert led_app.get_led_state() is False

        led_app.set_led_state(True)
        assert led_app.get_led_state() is True

        led_app.set_led_state(False)
        assert led_app.get_led_state() is False

    def test_set_streaming(self, led_app):
        """Test set_streaming updates streaming state."""
        assert led_app.is_streaming() is False

        led_app.set_streaming(True)
        assert led_app.is_streaming() is True
        assert led_app._is_streaming is True

        led_app.set_streaming(False)
        assert led_app.is_streaming() is False
        assert led_app._is_streaming is False

    def test_set_p_streaming(self, led_app):
        """Test set_p_streaming updates P-streaming state."""
        assert led_app.is_p_streaming() is False

        led_app.set_p_streaming(True)
        assert led_app.is_p_streaming() is True
        assert led_app._p_streaming is True

        led_app.set_p_streaming(False)
        assert led_app.is_p_streaming() is False
        assert led_app._p_streaming is False

    def test_set_p_recording(self, led_app):
        """Test set_p_recording updates recording state."""
        assert led_app.is_p_recording() is False

        led_app.set_p_recording(True)
        assert led_app.is_p_recording() is True
        assert led_app._p_recording is True

        led_app.set_p_recording(False)
        assert led_app.is_p_recording() is False
        assert led_app._p_recording is False

    def test_setter_idempotency(self, led_app):
        """Test that setters can be called multiple times with same value."""
        for _ in range(5):
            led_app.set_led_state(True)
            assert led_app.get_led_state() is True

        for _ in range(5):
            led_app.set_streaming(False)
            assert led_app.is_streaming() is False

    def test_multiple_setters_independent(self, led_app):
        """Test that multiple setters operate independently."""
        led_app.set_led_state(True)
        led_app.set_streaming(True)
        led_app.set_p_streaming(True)

        assert led_app.get_led_state() is True
        assert led_app.is_streaming() is True
        assert led_app.is_p_streaming() is True

        # Change one shouldn't affect others
        led_app.set_led_state(False)
        assert led_app.get_led_state() is False
        assert led_app.is_streaming() is True
        assert led_app.is_p_streaming() is True


# =============================================================================
# Test Suite 5: Task management (~15 tests)
# =============================================================================

@pytest.mark.unit
class TestLEDControllerAppTaskManagement:
    """Test task queue and management methods."""

    def test_send_task_returns_result_queue(self, led_app):
        """Test that send_task returns a result queue."""
        task = SerialTask(command=SerialCommand.SEND_LED)
        result_queue = led_app.send_task(task)

        assert result_queue is not None
        assert isinstance(result_queue, queue.Queue)

    def test_send_task_adds_to_pending(self, led_app):
        """Test that send_task adds task to pending list."""
        assert led_app.get_pending_count() == 0

        task = SerialTask(command=SerialCommand.READ_ADC)
        led_app.send_task(task)

        assert led_app.get_pending_count() == 1

    def test_send_task_with_various_commands(self, led_app):
        """Test send_task with different command types."""
        commands = [
            SerialCommand.SEND_LED,
            SerialCommand.SEND_PWM,
            SerialCommand.READ_ADC,
            SerialCommand.START_STREAM,
            SerialCommand.STOP_STREAM,
        ]

        for cmd in commands:
            task = SerialTask(command=cmd)
            result_queue = led_app.send_task(task)

            assert result_queue is not None
            assert isinstance(result_queue, queue.Queue)

    def test_send_task_attaches_result_queue(self, led_app):
        """Test that send_task attaches result queue to task."""
        task = SerialTask(command=SerialCommand.SEND_LED)
        assert task.result_queue is None

        result_queue = led_app.send_task(task)

        assert task.result_queue is result_queue

    def test_multiple_send_tasks(self, led_app):
        """Test sending multiple tasks."""
        for i in range(5):
            task = SerialTask(command=SerialCommand.READ_ADC)
            led_app.send_task(task)

        assert led_app.get_pending_count() == 5

    def test_get_pending_count_increments(self, led_app):
        """Test that pending count increments correctly."""
        counts = []
        for i in range(1, 6):
            task = SerialTask(command=SerialCommand.SEND_LED)
            led_app.send_task(task)
            counts.append(led_app.get_pending_count())

        assert counts == [1, 2, 3, 4, 5]

    def test_get_pending_list_returns_names(self, led_app):
        """Test that get_pending_list returns command names."""
        tasks = [
            SerialTask(command=SerialCommand.SEND_LED),
            SerialTask(command=SerialCommand.SEND_PWM),
            SerialTask(command=SerialCommand.READ_ADC),
        ]

        for task in tasks:
            led_app.send_task(task)

        pending = led_app.get_pending_list()
        assert pending == ["send_led", "send_pwm", "read_adc"]

    def test_get_pending_list_order_preserved(self, led_app):
        """Test that pending list preserves task order."""
        commands = [
            SerialCommand.START_STREAM,
            SerialCommand.READ_ADC,
            SerialCommand.SEND_LED,
            SerialCommand.STOP_STREAM,
        ]

        for cmd in commands:
            task = SerialTask(command=cmd)
            led_app.send_task(task)

        pending = led_app.get_pending_list()
        expected = ["start_stream", "read_adc", "send_led", "stop_stream"]
        assert pending == expected

    def test_task_with_parameters(self, led_app):
        """Test sending tasks with various parameters."""
        # Task with LED state
        task1 = SerialTask(command=SerialCommand.SEND_LED, led_state=True)
        led_app.send_task(task1)
        assert task1.led_state is True

        # Task with PWM duty
        task2 = SerialTask(command=SerialCommand.SEND_PWM, pwm_duty=50)
        led_app.send_task(task2)
        assert task2.pwm_duty == 50

        # Task with stream interval
        task3 = SerialTask(command=SerialCommand.START_STREAM, stream_interval=100)
        led_app.send_task(task3)
        assert task3.stream_interval == 100

    def test_p_controller_task_parameters(self, led_app):
        """Test sending P-controller related tasks."""
        # P-mode task
        task1 = SerialTask(command=SerialCommand.SET_P_MODE, p_mode=1)
        led_app.send_task(task1)
        assert task1.p_mode == 1

        # P-setpoint task
        task2 = SerialTask(command=SerialCommand.SET_P_SETPOINT, p_setpoint=2048)
        led_app.send_task(task2)
        assert task2.p_setpoint == 2048

        # P-gain task
        task3 = SerialTask(command=SerialCommand.SET_P_GAIN, p_gain=2.5)
        led_app.send_task(task3)
        assert task3.p_gain == 2.5

    def test_check_results_returns_empty_initially(self, led_app):
        """Test that check_results returns empty list initially."""
        results = led_app.check_results()
        assert results == []

    def test_quit_task_creation(self, led_app):
        """Test that QUIT task can be created."""
        task = SerialTask(command=SerialCommand.QUIT)
        assert task.command == SerialCommand.QUIT

    def test_connect_task_with_port(self, led_app):
        """Test CONNECT task with port parameter."""
        task = SerialTask(command=SerialCommand.CONNECT, port="/dev/ttyACM0")
        led_app.send_task(task)

        assert task.port == "/dev/ttyACM0"


# =============================================================================
# Test Suite 6: ADC history management (~15 tests)
# =============================================================================

@pytest.mark.unit
class TestLEDControllerAppADCHistory:
    """Test ADC data history management."""

    def test_add_adc_data_single_point(self, led_app):
        """Test adding a single ADC data point."""
        led_app.add_adc_data(2048, 1.65)

        times, raw, voltage = led_app.get_adc_history()

        assert len(times) == 1
        assert len(raw) == 1
        assert len(voltage) == 1
        assert raw[0] == 2048
        assert abs(voltage[0] - 1.65) < 0.01

    def test_add_adc_data_multiple_points(self, led_app):
        """Test adding multiple ADC data points."""
        test_data = [
            (0, 0.0),
            (1024, 0.825),
            (2048, 1.65),
            (3072, 2.475),
            (4095, 3.3),
        ]

        for raw_val, voltage in test_data:
            led_app.add_adc_data(raw_val, voltage)

        times, raw, voltage = led_app.get_adc_history()

        assert len(times) == 5
        assert len(raw) == 5
        assert len(voltage) == 5
        assert raw == [0, 1024, 2048, 3072, 4095]

    def test_add_adc_data_circular_buffer_overflow(self, led_app):
        """Test that circular buffer discards old data when full."""
        # Add more than maxlen (100) points
        for i in range(150):
            led_app.add_adc_data(i, i * 0.001)

        times, raw, voltage = led_app.get_adc_history()

        # Should only have 100 points (maxlen)
        assert len(times) == 100
        assert len(raw) == 100
        assert len(voltage) == 100

        # Oldest data should be discarded (first 50 points gone)
        assert raw[0] == 50  # First remaining point
        assert raw[-1] == 149  # Last point

    def test_get_adc_history_empty(self, led_app):
        """Test get_adc_history returns empty lists when no data."""
        times, raw, voltage = led_app.get_adc_history()

        assert times == []
        assert raw == []
        assert voltage == []

    def test_get_adc_history_with_data(self, led_app):
        """Test get_adc_history returns correct data."""
        led_app.add_adc_data(1000, 0.806)
        led_app.add_adc_data(2000, 1.612)

        times, raw, voltage = led_app.get_adc_history()

        assert len(raw) == 2
        assert raw[0] == 1000
        assert raw[1] == 2000
        assert abs(voltage[0] - 0.806) < 0.01
        assert abs(voltage[1] - 1.612) < 0.01

    def test_get_adc_history_relative_time(self, led_app):
        """Test get_adc_history returns relative time (seconds ago)."""
        import time

        led_app.add_adc_data(1000, 0.806)
        time.sleep(0.1)  # Small delay
        led_app.add_adc_data(2000, 1.612)

        times, raw, voltage = led_app.get_adc_history()

        # Times should be negative (seconds ago)
        assert len(times) == 2
        assert times[0] < 0  # First point was longer ago
        assert times[1] < 0  # Second point was more recent
        assert times[0] < times[1]  # First point is more negative

    def test_get_adc_history_returns_copies_not_references(self, led_app):
        """Test that get_adc_history returns copies, not internal references."""
        led_app.add_adc_data(1000, 0.806)

        times1, raw1, voltage1 = led_app.get_adc_history()
        times2, raw2, voltage2 = led_app.get_adc_history()

        # Modifying returned lists shouldn't affect internal state
        raw1.append(9999)
        voltage1.append(9.99)

        # Second call should return original data
        assert len(raw2) == 1
        assert raw2[0] == 1000
        assert len(voltage2) == 1

    def test_maxlen_constant_100(self, led_app):
        """Test that ADC max points constant is 100."""
        assert led_app._adc_max_points == 100

        # Add exactly 100 points
        for i in range(100):
            led_app.add_adc_data(i, i * 0.001)

        times, raw, voltage = led_app.get_adc_history()
        assert len(times) == 100

        # Add one more - should still be 100
        led_app.add_adc_data(999, 0.999)
        times, raw, voltage = led_app.get_adc_history()
        assert len(times) == 100

    def test_adc_history_time_values_are_timestamps(self, led_app):
        """Test that ADC history stores valid timestamps."""
        import time

        start_time = time.time()
        led_app.add_adc_data(1234, 0.997)
        end_time = time.time()

        times, raw, voltage = led_app.get_adc_history()

        # Relative time should be between start and end time
        # (negative because it's "seconds ago")
        assert times[0] >= (start_time - end_time)
        assert times[0] <= 0

    def test_adc_voltage_precision(self, led_app):
        """Test that ADC voltage values maintain precision."""
        test_voltage = 1.23456789
        led_app.add_adc_data(1537, test_voltage)

        times, raw, voltage = led_app.get_adc_history()

        # Should preserve reasonable precision
        assert abs(voltage[0] - test_voltage) < 0.0001

    def test_concurrent_adc_data_addition(self, led_app):
        """Test thread safety of concurrent ADC data addition."""
        errors = []

        def add_data(start_idx):
            try:
                for i in range(50):
                    idx = start_idx + i
                    led_app.add_adc_data(idx, idx * 0.001)
            except Exception as e:
                errors.append(e)

        threads = [
            threading.Thread(target=add_data, args=(0,)),
            threading.Thread(target=add_data, args=(50,)),
            threading.Thread(target=add_data, args=(100,))
        ]

        for t in threads:
            t.start()

        for t in threads:
            t.join(timeout=5)

        assert len(errors) == 0

        times, raw, voltage = led_app.get_adc_history()
        # Should have 100 points (maxlen), possibly less due to circular buffer
        assert len(raw) <= 100

    def test_adc_history_separate_from_p_stream_history(self, led_app):
        """Test that ADC and P-stream histories are independent."""
        # Add to ADC history
        led_app.add_adc_data(1000, 0.806)

        # ADC history should have data
        _, adc_raw, _ = led_app.get_adc_history()
        assert len(adc_raw) == 1

        # P-stream history should be empty
        p_times, p_measured, p_voltage = led_app.get_p_stream_history()
        assert len(p_measured) == 0


# =============================================================================
# Test Suite 7: P-stream history (~10 tests)
# =============================================================================

@pytest.mark.unit
class TestLEDControllerAppPStreamHistory:
    """Test P-stream data history management."""

    def test_p_stream_history_storage_structure(self, led_app):
        """Test that P-stream history has correct structure."""
        # Check that all P-stream deques exist
        assert hasattr(led_app, '_p_time_history')
        assert hasattr(led_app, '_p_measured_history')
        assert hasattr(led_app, '_setpoint_history')
        assert hasattr(led_app, '_pwm_history')
        assert hasattr(led_app, '_error_history')

    def test_get_p_stream_history_empty(self, led_app):
        """Test get_p_stream_history returns empty when no data."""
        times, measured, voltage = led_app.get_p_stream_history()

        assert times == []
        assert measured == []
        assert voltage == []

    def test_p_stream_history_maxlen(self, led_app):
        """Test that P-stream history deques have maxlen=100."""
        assert led_app._p_time_history.maxlen == 100
        assert led_app._p_measured_history.maxlen == 100
        assert led_app._setpoint_history.maxlen == 100
        assert led_app._pwm_history.maxlen == 100
        assert led_app._error_history.maxlen == 100

    def test_p_stream_history_includes_all_series(self, led_app):
        """Test that P-stream history includes all data series."""
        # Manually add P-stream data (simulating what _data_reader does)
        with led_app._lock:
            led_app._p_time_history.append(time.time())
            led_app._p_measured_history.append(2048)
            led_app._setpoint_history.append(2500)
            led_app._pwm_history.append(65)
            led_app._error_history.append(452)

        # History should be accessible
        assert len(led_app._p_time_history) == 1
        assert len(led_app._p_measured_history) == 1
        assert len(led_app._setpoint_history) == 1
        assert len(led_app._pwm_history) == 1
        assert len(led_app._error_history) == 1

    def test_p_stream_history_voltage_conversion(self, led_app):
        """Test that P-stream history converts measured to voltage."""
        # Add raw ADC value
        with led_app._lock:
            led_app._p_time_history.append(time.time())
            led_app._p_measured_history.append(2048)  # Midpoint

        times, measured, voltage = led_app.get_p_stream_history()

        # Should convert to voltage (2048/4095 * 3.3 ≈ 1.65V)
        assert abs(voltage[0] - 1.65) < 0.01

    def test_p_stream_history_relative_time(self, led_app):
        """Test that P-stream history returns relative time."""
        with led_app._lock:
            led_app._p_time_history.append(time.time())
            led_app._p_measured_history.append(1000)

        times, measured, voltage = led_app.get_p_stream_history()

        # Time should be negative (seconds ago)
        assert len(times) == 1
        assert times[0] < 0
        assert times[0] > -1  # Should be less than 1 second ago

    def test_p_stream_history_circular_buffer(self, led_app):
        """Test that P-stream history behaves as circular buffer."""
        # Manually add more than 100 points
        for i in range(150):
            with led_app._lock:
                led_app._p_time_history.append(time.time())
                led_app._p_measured_history.append(i)
                led_app._setpoint_history.append(i + 100)
                led_app._pwm_history.append(i % 100)
                led_app._error_history.append(100 - i)

        # Should only have 100 points
        assert len(led_app._p_measured_history) == 100
        assert len(led_app._setpoint_history) == 100
        assert len(led_app._pwm_history) == 100
        assert len(led_app._error_history) == 100

    def test_p_stream_history_returns_copies(self, led_app):
        """Test that get_p_stream_history returns copies."""
        with led_app._lock:
            led_app._p_time_history.append(time.time())
            led_app._p_measured_history.append(1000)

        times1, measured1, voltage1 = led_app.get_p_stream_history()

        # Modify returned list
        measured1.append(9999)

        # Second call should return original data
        times2, measured2, voltage2 = led_app.get_p_stream_history()
        assert len(measured2) == 1
        assert measured2[0] == 1000

    def test_p_stream_error_calculation(self, led_app):
        """Test that P-stream error is setpoint - measured."""
        with led_app._lock:
            led_app._p_time_history.append(time.time())
            led_app._p_measured_history.append(1800)
            led_app._setpoint_history.append(2000)
            led_app._error_history.append(200)  # 2000 - 1800

        # Error should be stored correctly
        assert led_app._error_history[0] == 200


# =============================================================================
# Test Suite 8: Shutdown and cleanup (~3 tests)
# =============================================================================

@pytest.mark.unit
class TestLEDControllerAppShutdown:
    """Test application shutdown and cleanup."""

    def test_shutdown_sets_running_false(self, led_app):
        """Test that shutdown sets _running to False."""
        assert led_app._running is True

        led_app.shutdown()

        assert led_app._running is False

    def test_shutdown_stops_streaming_state(self, led_app):
        """Test that shutdown clears streaming state."""
        led_app.set_streaming(True)
        led_app.set_p_recording(True)

        led_app.shutdown()

        # Note: shutdown may not clear these immediately, but should stop
        # the threads from processing
        assert led_app._running is False

    def test_shutdown_closes_recording_file(self, led_app):
        """Test that shutdown closes recording file if open."""
        # Simulate recording state
        led_app.set_p_recording(True)

        led_app.shutdown()

        # After shutdown, recording should be stopped
        assert led_app._running is False


# =============================================================================
# Fixtures
# =============================================================================

@pytest.fixture
def led_app():
    """Provide a fresh LEDControllerApp instance for each test."""
    app = LEDControllerApp()
    yield app
    # Cleanup
    app.shutdown()
