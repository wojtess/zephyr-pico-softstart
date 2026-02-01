"""
Tests for GUI constants (SerialCommand, SerialTask, SerialResult, TAGS, P_MODE).

Phase 2a: GUI Constants Tests
Unit tests for constants.py data structures.
"""

import pytest
import queue
from dataclasses import FrozenInstanceError

from scripts.gui.constants import (
    SerialCommand,
    SerialTask,
    SerialResult,
    TAGS,
    P_MODE_MANUAL,
    P_MODE_AUTO,
    P_MODE_VALUES,
)


@pytest.mark.unit
class TestSerialCommandEnum:
    """Test SerialCommand enum values and properties."""

    def test_serial_command_values(self):
        """Verify all commands have correct string values."""
        expected_values = {
            SerialCommand.CONNECT: "connect",
            SerialCommand.DISCONNECT: "disconnect",
            SerialCommand.SEND_LED: "send_led",
            SerialCommand.SEND_PWM: "send_pwm",
            SerialCommand.READ_ADC: "read_adc",
            SerialCommand.START_STREAM: "start_stream",
            SerialCommand.STOP_STREAM: "stop_stream",
            SerialCommand.CHECK_HEALTH: "check_health",
            SerialCommand.SET_P_MODE: "set_p_mode",
            SerialCommand.SET_P_SETPOINT: "set_p_setpoint",
            SerialCommand.SET_P_GAIN: "set_p_gain",
            SerialCommand.SET_P_FEED_FORWARD: "set_p_feed_forward",
            SerialCommand.START_P_STREAM: "start_p_stream",
            SerialCommand.STOP_P_STREAM: "stop_p_stream",
            SerialCommand.GET_P_STATUS: "get_p_status",
            SerialCommand.START_P_RECORD: "start_p_record",
            SerialCommand.STOP_P_RECORD: "stop_p_record",
            SerialCommand.QUIT: "quit",
        }
        for command, expected_value in expected_values.items():
            assert command.value == expected_value

    def test_serial_command_uniqueness(self):
        """All command values should be unique."""
        values = [cmd.value for cmd in SerialCommand]
        assert len(values) == len(set(values))

    def test_all_expected_commands_exist(self):
        """Verify all expected commands are present."""
        expected_commands = [
            "CONNECT",
            "DISCONNECT",
            "SEND_LED",
            "SEND_PWM",
            "READ_ADC",
            "START_STREAM",
            "STOP_STREAM",
            "CHECK_HEALTH",
            "SET_P_MODE",
            "SET_P_SETPOINT",
            "SET_P_GAIN",
            "SET_P_FEED_FORWARD",
            "START_P_STREAM",
            "STOP_P_STREAM",
            "GET_P_STATUS",
            "START_P_RECORD",
            "STOP_P_RECORD",
            "QUIT",
        ]
        for cmd_name in expected_commands:
            assert hasattr(SerialCommand, cmd_name)

    def test_connect_command_value(self):
        """CONNECT command should have 'connect' value."""
        assert SerialCommand.CONNECT.value == "connect"

    def test_disconnect_command_value(self):
        """DISCONNECT command should have 'disconnect' value."""
        assert SerialCommand.DISCONNECT.value == "disconnect"

    def test_quit_command_value(self):
        """QUIT command should have 'quit' value."""
        assert SerialCommand.QUIT.value == "quit"

    def test_p_controller_commands_exist(self):
        """All P-controller related commands should exist."""
        p_commands = [
            SerialCommand.SET_P_MODE,
            SerialCommand.SET_P_SETPOINT,
            SerialCommand.SET_P_GAIN,
            SerialCommand.SET_P_FEED_FORWARD,
            SerialCommand.START_P_STREAM,
            SerialCommand.STOP_P_STREAM,
            SerialCommand.GET_P_STATUS,
            SerialCommand.START_P_RECORD,
            SerialCommand.STOP_P_RECORD,
        ]
        for cmd in p_commands:
            assert cmd in SerialCommand

    def test_streaming_commands_exist(self):
        """Streaming related commands should exist."""
        assert SerialCommand.START_STREAM in SerialCommand
        assert SerialCommand.STOP_STREAM in SerialCommand

    def test_enum_is_iterable(self):
        """SerialCommand should be iterable."""
        commands = list(SerialCommand)
        assert len(commands) > 0
        assert SerialCommand.QUIT in commands


@pytest.mark.unit
class TestSerialTaskDataclass:
    """Test SerialTask dataclass creation and properties."""

    def test_serial_task_creation_minimal(self):
        """Create SerialTask with only required command field."""
        task = SerialTask(command=SerialCommand.CONNECT)
        assert task.command == SerialCommand.CONNECT
        assert task.port is None
        assert task.led_state is None
        assert task.pwm_duty is None

    def test_serial_task_creation_with_result_queue(self):
        """Create SerialTask with result_queue."""
        result_queue = queue.Queue()
        task = SerialTask(
            command=SerialCommand.READ_ADC,
            result_queue=result_queue
        )
        assert task.result_queue is result_queue

    def test_serial_task_optional_fields_default_to_none(self):
        """All optional fields should default to None."""
        task = SerialTask(command=SerialCommand.SEND_LED)
        assert task.port is None
        assert task.led_state is None
        assert task.pwm_duty is None
        assert task.result_queue is None
        assert task.adc_interval is None
        assert task.stream_interval is None

    def test_serial_task_all_fields_settable(self):
        """All fields should be settable."""
        result_queue = queue.Queue()
        task = SerialTask(
            command=SerialCommand.SET_P_GAIN,
            port="/dev/ttyACM0",
            led_state=True,
            pwm_duty=50,
            result_queue=result_queue,
            adc_interval=0.5,
            stream_interval=100,
            p_mode=1,
            p_setpoint=2000,
            p_gain=1.5,
            p_feed_forward=100,
            p_stream_interval=50,
        )
        assert task.command == SerialCommand.SET_P_GAIN
        assert task.port == "/dev/ttyACM0"
        assert task.led_state is True
        assert task.pwm_duty == 50
        assert task.result_queue is result_queue
        assert task.adc_interval == 0.5
        assert task.stream_interval == 100
        assert task.p_mode == 1
        assert task.p_setpoint == 2000
        assert task.p_gain == 1.5
        assert task.p_feed_forward == 100
        assert task.p_stream_interval == 50

    def test_serial_task_with_adc_task(self):
        """Create SerialTask for ADC operations."""
        result_queue = queue.Queue()
        task = SerialTask(
            command=SerialCommand.READ_ADC,
            result_queue=result_queue
        )
        assert task.command == SerialCommand.READ_ADC
        assert task.result_queue is result_queue

    def test_serial_task_with_p_controller_task(self):
        """Create SerialTask for P-controller operations."""
        task = SerialTask(
            command=SerialCommand.SET_P_MODE,
            p_mode=1
        )
        assert task.command == SerialCommand.SET_P_MODE
        assert task.p_mode == 1

    def test_serial_task_with_stream_task(self):
        """Create SerialTask for streaming operations."""
        task = SerialTask(
            command=SerialCommand.START_STREAM,
            stream_interval=100
        )
        assert task.command == SerialCommand.START_STREAM
        assert task.stream_interval == 100

    def test_serial_task_connect_with_port(self):
        """Create CONNECT task with port."""
        task = SerialTask(
            command=SerialCommand.CONNECT,
            port="/dev/ttyACM0"
        )
        assert task.port == "/dev/ttyACM0"

    def test_serial_task_pwm_with_duty(self):
        """Create PWM task with duty cycle."""
        task = SerialTask(
            command=SerialCommand.SEND_PWM,
            pwm_duty=75
        )
        assert task.pwm_duty == 75

    def test_serial_task_led_with_state(self):
        """Create LED task with state."""
        task = SerialTask(
            command=SerialCommand.SEND_LED,
            led_state=True
        )
        assert task.led_state is True

    def test_serial_task_p_stream_with_interval(self):
        """Create P-stream task with interval."""
        task = SerialTask(
            command=SerialCommand.START_P_STREAM,
            p_stream_interval=50
        )
        assert task.p_stream_interval == 50

    def test_serial_task_quit_command(self):
        """Create QUIT task."""
        task = SerialTask(command=SerialCommand.QUIT)
        assert task.command == SerialCommand.QUIT

    def test_serial_task_pwm_duty_min_boundary(self):
        """PWM duty should accept 0 (minimum)."""
        task = SerialTask(
            command=SerialCommand.SEND_PWM,
            pwm_duty=0
        )
        assert task.pwm_duty == 0

    def test_serial_task_pwm_duty_max_boundary(self):
        """PWM duty should accept 100 (maximum)."""
        task = SerialTask(
            command=SerialCommand.SEND_PWM,
            pwm_duty=100
        )
        assert task.pwm_duty == 100

    def test_serial_task_adc_interval_small(self):
        """ADC interval should accept small float values."""
        task = SerialTask(
            command=SerialCommand.START_STREAM,
            adc_interval=0.1
        )
        assert task.adc_interval == 0.1

    def test_serial_task_stream_interval_boundary(self):
        """Stream interval should accept various millisecond values."""
        # Test with 1ms minimum
        task1 = SerialTask(
            command=SerialCommand.START_P_STREAM,
            p_stream_interval=1
        )
        assert task1.p_stream_interval == 1

        # Test with 1000ms (1 second)
        task2 = SerialTask(
            command=SerialCommand.START_P_STREAM,
            p_stream_interval=1000
        )
        assert task2.p_stream_interval == 1000


@pytest.mark.unit
class TestSerialResultDataclass:
    """Test SerialResult dataclass creation and properties."""

    def test_serial_result_creation_success(self):
        """Create successful SerialResult."""
        result = SerialResult(
            command=SerialCommand.CONNECT,
            success=True,
            message="Connected successfully"
        )
        assert result.command == SerialCommand.CONNECT
        assert result.success is True
        assert result.message == "Connected successfully"
        assert result.error is None

    def test_serial_result_creation_failure(self):
        """Create failed SerialResult with error."""
        result = SerialResult(
            command=SerialCommand.CONNECT,
            success=False,
            message="Connection failed",
            error="Port not found"
        )
        assert result.success is False
        assert result.error == "Port not found"

    def test_serial_result_with_adc_data(self):
        """Create SerialResult with ADC values."""
        result = SerialResult(
            command=SerialCommand.READ_ADC,
            success=True,
            message="ADC read",
            adc_value=2048,
            adc_voltage=1.65
        )
        assert result.adc_value == 2048
        assert result.adc_voltage == 1.65

    def test_serial_result_with_error(self):
        """Create SerialResult with error field."""
        result = SerialResult(
            command=SerialCommand.READ_ADC,
            success=False,
            message="Read failed",
            error="Timeout"
        )
        assert result.error == "Timeout"
        assert result.success is False

    def test_serial_result_default_values(self):
        """Optional fields should default to None."""
        result = SerialResult(
            command=SerialCommand.SEND_LED,
            success=True,
            message="LED state set"
        )
        assert result.led_state is None
        assert result.error is None
        assert result.adc_value is None
        assert result.adc_voltage is None

    def test_serial_result_with_led_state(self):
        """Create SerialResult with LED state."""
        result = SerialResult(
            command=SerialCommand.SEND_LED,
            success=True,
            message="LED turned on",
            led_state=True
        )
        assert result.led_state is True

    def test_serial_result_command_types(self):
        """SerialResult should work with all command types."""
        commands = [
            SerialCommand.CONNECT,
            SerialCommand.READ_ADC,
            SerialCommand.START_STREAM,
            SerialCommand.SET_P_GAIN,
        ]
        for cmd in commands:
            result = SerialResult(
                command=cmd,
                success=True,
                message="OK"
            )
            assert result.command == cmd

    def test_adc_value_range_min(self):
        """ADC value should accept 0 (minimum valid range)."""
        result = SerialResult(
            command=SerialCommand.READ_ADC,
            success=True,
            message="ADC min",
            adc_value=0,
            adc_voltage=0.0
        )
        assert result.adc_value == 0
        assert result.adc_voltage == 0.0

    def test_adc_value_range_max(self):
        """ADC value should accept 4095 (12-bit maximum)."""
        result = SerialResult(
            command=SerialCommand.READ_ADC,
            success=True,
            message="ADC max",
            adc_value=4095,
            adc_voltage=3.3
        )
        assert result.adc_value == 4095
        assert result.adc_voltage == 3.3

    def test_adc_voltage_mid_range(self):
        """ADC voltage should handle mid-range values."""
        result = SerialResult(
            command=SerialCommand.READ_ADC,
            success=True,
            message="ADC mid",
            adc_value=2048,
            adc_voltage=1.65
        )
        assert result.adc_value == 2048
        assert result.adc_voltage == 1.65


@pytest.mark.unit
class TestTagsConstants:
    """Test TAGS dictionary structure and content."""

    def test_tags_dict_structure(self):
        """TAGS should be a dictionary."""
        assert isinstance(TAGS, dict)

    def test_tag_count(self):
        """Should have exactly 68 tags."""
        assert len(TAGS) == 68, f"Expected 68 tags, found {len(TAGS)}"

    def test_all_expected_tags_present(self):
        """All 68 expected tags should be present."""
        expected_tags = [
            # Main window and connection (4)
            "primary_window",
            "port_combo",
            "refresh_btn",
            "connect_btn",
            # LED and status (5)
            "led_btn",
            "status_message",
            "conn_indicator",
            "led_indicator",
            "pwm_slider",
            # PWM labels and response (3)
            "pwm_label",
            "last_response",
            # ADC tags (7)
            "adc_read_btn",
            "adc_auto_checkbox",
            "adc_history_plot",
            "adc_value_raw",
            "adc_value_voltage",
            "adc_series_raw",
            "adc_series_voltage",
            # Streaming tags (4)
            "stream_interval_input",
            "stream_start_btn",
            "stream_stop_btn",
            "stream_status",
            # Task queue tags (3)
            "queue_label",
            "queue_count",
            "queue_processing",
            # P-Controller mode and controls (14)
            "p_mode_radio",
            "p_mode_radio_p",
            "p_control_group",
            "p_setpoint_input",
            "p_setpoint_slider",
            "p_setpoint_btn_m100",
            "p_setpoint_btn_m10",
            "p_setpoint_btn_p10",
            "p_setpoint_btn_p100",
            "p_gain_slider",
            "p_gain_input",
            "p_gain_label",
            "p_ff_slider",
            "p_ff_input",
            # P-Controller FF and output (3)
            "p_ff_label",
            "p_pwm_output",
            # P-Controller streaming (7)
            "p_stream_start_btn",
            "p_stream_stop_btn",
            "p_stream_interval",
            "p_stream_status",
            "p_stream_enable_chk",
            "p_record_enable_chk",
            "adc_stream_enable_chk",
            "p_stream_plot",
            # Plot series for P-stream (4)
            "p_series_setpoint",
            "p_series_measured",
            "p_series_pwm",
            "p_series_error",
            # P-Controller recording (3)
            "p_record_start_btn",
            "p_record_stop_btn",
            "p_record_status",
            # P-Controller analysis (11)
            "p_analyze_btn",
            "p_analysis_window",
            "p_analysis_plot",
            "p_analysis_stats",
            "p_analysis_table",
            "p_analysis_file_dialog",
            "p_analysis_export_btn",
            "p_analysis_export_format",
            "p_analysis_filter_method",
            "p_analysis_filter_window",
            "p_analysis_time_range",
            "p_analysis_derived_metrics",
        ]
        for tag in expected_tags:
            assert tag in TAGS, f"Missing tag: {tag}"
        # Verify count matches
        assert len(expected_tags) == 68, f"Expected 68 tags in list, found {len(expected_tags)}"

    def test_tag_values_are_strings(self):
        """All tag values should be strings."""
        for key, value in TAGS.items():
            assert isinstance(value, str), f"Tag {key} has non-string value: {value}"

    def test_tag_values_are_unique(self):
        """All tag values should be unique."""
        values = list(TAGS.values())
        assert len(values) == len(set(values)), "Duplicate tag values found"

    def test_tags_not_empty(self):
        """TAGS dictionary should not be empty."""
        assert len(TAGS) > 0


@pytest.mark.unit
class TestPModeConstants:
    """Test P-controller mode constants."""

    def test_p_mode_manual_value(self):
        """P_MODE_MANUAL should be 'M'."""
        assert P_MODE_MANUAL == "M"

    def test_p_mode_auto_value(self):
        """P_MODE_AUTO should be 'P'."""
        assert P_MODE_AUTO == "P"

    def test_p_mode_values_list(self):
        """P_MODE_VALUES should contain both modes."""
        assert P_MODE_MANUAL in P_MODE_VALUES
        assert P_MODE_AUTO in P_MODE_VALUES

    def test_p_mode_values_length(self):
        """P_MODE_VALUES should have exactly 2 modes."""
        assert len(P_MODE_VALUES) == 2

    def test_p_mode_values_are_strings(self):
        """All P_MODE_VALUES should be strings."""
        for mode in P_MODE_VALUES:
            assert isinstance(mode, str)

    def test_p_mode_manual_in_list(self):
        """P_MODE_MANUAL should be the first element."""
        assert P_MODE_VALUES[0] == P_MODE_MANUAL

    def test_p_mode_auto_in_list(self):
        """P_MODE_AUTO should be the second element."""
        assert P_MODE_VALUES[1] == P_MODE_AUTO

    def test_p_mode_values_immutability(self):
        """P_MODE_VALUES should be a tuple to prevent modification."""
        assert isinstance(P_MODE_VALUES, tuple), \
            "P_MODE_VALUES should be a tuple for immutability"
