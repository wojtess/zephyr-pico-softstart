"""
Constants and type definitions for RP2040 LED Control GUI.
"""

import queue
from typing import Optional
from dataclasses import dataclass
from enum import Enum


class SerialCommand(Enum):
    """Serial command types for worker thread."""
    CONNECT = "connect"
    DISCONNECT = "disconnect"
    SEND_LED = "send_led"
    SEND_PWM = "send_pwm"
    READ_ADC = "read_adc"
    START_STREAM = "start_stream"
    STOP_STREAM = "stop_stream"
    CHECK_HEALTH = "check_health"
    SET_P_MODE = "set_p_mode"
    SET_P_SETPOINT = "set_p_setpoint"
    SET_P_GAIN = "set_p_gain"
    SET_P_FEED_FORWARD = "set_p_feed_forward"
    START_P_STREAM = "start_p_stream"
    STOP_P_STREAM = "stop_p_stream"
    GET_P_STATUS = "get_p_status"
    START_P_RECORD = "start_p_record"
    STOP_P_RECORD = "stop_p_record"
    QUIT = "quit"


@dataclass
class SerialTask:
    """Task for serial worker thread."""
    command: SerialCommand
    port: Optional[str] = None
    led_state: Optional[bool] = None
    pwm_duty: Optional[int] = None
    result_queue: Optional[queue.Queue] = None
    adc_interval: Optional[float] = None  # For auto-read ADC (seconds)
    stream_interval: Optional[int] = None  # For streaming (milliseconds)
    p_mode: Optional[int] = None
    p_setpoint: Optional[int] = None
    p_gain: Optional[float] = None
    p_feed_forward: Optional[int] = None
    p_stream_interval: Optional[int] = None  # For P-streaming (milliseconds)


@dataclass
class SerialResult:
    """Result from serial worker thread."""
    command: SerialCommand
    success: bool
    message: str
    led_state: Optional[bool] = None
    error: Optional[str] = None
    adc_value: Optional[int] = None  # Raw ADC value (0-4095)
    adc_voltage: Optional[float] = None  # Voltage in V (0-3.3)


# Tag constants for better resource management
TAGS = {
    "primary_window": "win_primary",
    "port_combo": "combo_port",
    "refresh_btn": "btn_refresh",
    "connect_btn": "btn_connect",
    "led_btn": "btn_led",
    "status_message": "txt_status",
    "conn_indicator": "ind_conn",
    "led_indicator": "ind_led",
    "pwm_slider": "slider_pwm",
    "pwm_label": "txt_pwm_value",
    "last_response": "txt_response",
    # ADC tags
    "adc_read_btn": "btn_adc_read",
    "adc_auto_checkbox": "chk_adc_auto",
    "adc_history_plot": "plot_adc_history",
    "adc_value_raw": "txt_adc_raw",
    "adc_value_voltage": "txt_adc_voltage",
    "adc_series_raw": "series_adc_raw",
    "adc_series_voltage": "series_adc_voltage",
    # Streaming tags
    "stream_interval_input": "input_stream_interval",
    "stream_start_btn": "btn_stream_start",
    "stream_stop_btn": "btn_stream_stop",
    "stream_status": "txt_stream_status",
    # Task queue tags
    "queue_label": "txt_queue_label",
    "queue_count": "txt_queue_count",
    "queue_processing": "txt_queue_processing",
    # P-Controller mode and controls
    "p_mode_radio": "radio_p_mode",
    "p_control_group": "group_p_control",
    "p_setpoint_input": "input_p_setpoint",
    "p_gain_slider": "slider_p_gain",
    "p_gain_label": "txt_p_gain",
    "p_ff_slider": "slider_p_ff",
    "p_ff_label": "txt_p_ff",
    "p_pwm_output": "txt_p_pwm",
    # P-Controller streaming
    "p_stream_start_btn": "btn_p_stream_start",
    "p_stream_stop_btn": "btn_p_stream_stop",
    "p_stream_interval": "input_p_stream_interval",
    "p_stream_status": "txt_p_stream_status",
    "p_stream_plot": "plot_p_stream",
    # Plot series for P-stream
    "p_series_setpoint": "p_series_setpoint",
    "p_series_measured": "p_series_measured",
    "p_series_pwm": "p_series_pwm",
    "p_series_error": "p_series_error",
    # P-Controller recording
    "p_record_start_btn": "btn_p_record_start",
    "p_record_stop_btn": "btn_p_record_stop",
    "p_record_status": "txt_p_record_status",
    # P-Controller analysis
    "p_analyze_btn": "btn_p_analyze",
    "p_analysis_window": "win_p_analysis",
    "p_analysis_plot": "plot_p_analysis",
    "p_analysis_stats": "txt_p_analysis_stats",
    "p_analysis_table": "table_p_analysis",
    "p_analysis_file_dialog": "dialog_p_analysis_file",
    "p_analysis_export_btn": "btn_p_analysis_export",
    "p_analysis_export_format": "combo_p_analysis_export",
    "p_analysis_filter_method": "combo_p_analysis_filter",
    "p_analysis_filter_window": "input_p_analysis_window",
    "p_analysis_time_range": "input_p_analysis_time_range",
    "p_analysis_derived_metrics": "txt_p_analysis_derived_metrics"
}
