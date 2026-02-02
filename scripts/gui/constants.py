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
    SET_P_KI = "set_p_ki"
    SET_P_FEED_FORWARD = "set_p_feed_forward"
    START_P_STREAM = "start_p_stream"
    STOP_P_STREAM = "stop_p_stream"
    GET_P_STATUS = "get_p_status"
    SET_FILTER_ALPHA = "set_filter_alpha"
    SET_FILTER_MODE = "set_filter_mode"
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
    p_ki: Optional[float] = None
    p_feed_forward: Optional[int] = None
    p_stream_interval: Optional[int] = None  # For P-streaming (milliseconds)
    filter_alpha_num: Optional[int] = None  # Filter alpha numerator (1-255)
    filter_alpha_den: Optional[int] = None  # Filter alpha denominator (1-255)
    filter_mode: Optional[int] = None  # Filter mode (0=IIR, 1=OS, 2=OS+IIR)


@dataclass
class SerialResult:
    """Result from serial worker thread."""
    command: SerialCommand
    success: bool
    message: str
    led_state: Optional[bool] = None
    error: Optional[str] = None
    adc_value: Optional[int] = None  # Raw ADC value (0-4095)
    adc_amps: Optional[float] = None  # Current in A (0-1.0)


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
    "adc_value_amps": "txt_adc_amps",
    "adc_series_raw": "series_adc_raw",
    "adc_series_amps": "series_adc_amps",
    # Streaming tags
    "stream_interval_input": "input_stream_interval",
    "stream_start_btn": "btn_stream_start",
    "stream_stop_btn": "btn_stream_stop",
    "stream_status": "txt_stream_status",
    # Task queue tags
    "queue_label": "txt_queue_label",
    "queue_count": "txt_queue_count",
    "queue_processing": "txt_queue_processing",
    # PI-Controller mode and controls
    "p_mode_radio": "radio_p_mode",
    "p_mode_radio_p": "radio_p_mode_p",
    "p_control_group": "group_p_control",
    "p_setpoint_input": "input_p_setpoint",
    "p_setpoint_slider": "slider_p_setpoint",
    "p_setpoint_btn_m100": "btn_p_setpoint_m100",
    "p_setpoint_btn_m10": "btn_p_setpoint_m10",
    "p_setpoint_btn_p10": "btn_p_setpoint_p10",
    "p_setpoint_btn_p100": "btn_p_setpoint_p100",
    "p_gain_slider": "slider_p_gain",
    "p_gain_input": "input_p_gain",
    "p_gain_label": "txt_p_gain",
    "p_ki_slider": "slider_p_ki",
    "p_ki_input": "input_p_ki",
    "p_ki_label": "txt_p_ki",
    "p_ff_slider": "slider_p_ff",
    "p_ff_input": "input_p_ff",
    "p_ff_label": "txt_p_ff",
    "p_pwm_output": "txt_p_pwm",
    # PI-Controller streaming
    "p_stream_start_btn": "btn_p_stream_start",
    "p_stream_stop_btn": "btn_p_stream_stop",
    "p_stream_interval": "input_p_stream_interval",
    "p_stream_status": "txt_p_stream_status",
    "p_stream_enable_chk": "chk_p_stream_enable",
    "p_record_enable_chk": "chk_p_record_enable",
    "adc_stream_enable_chk": "chk_adc_stream_enable",
    "p_stream_plot": "plot_p_stream",
    # Plot series for P-stream
    "p_series_setpoint": "p_series_setpoint",
    "p_series_measured": "p_series_measured",
    "p_series_pwm": "p_series_pwm",
    "p_series_error": "p_series_error",
    # PI-Controller recording
    "p_record_start_btn": "btn_p_record_start",
    "p_record_stop_btn": "btn_p_record_stop",
    "p_record_status": "txt_p_record_status",
    # PI-Controller analysis
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
    "p_analysis_derived_metrics": "txt_p_analysis_derived_metrics",
    # Filter alpha controls
    "filter_alpha_input": "input_filter_alpha",
    "filter_alpha_label": "txt_filter_alpha",
    "filter_alpha_info": "txt_filter_alpha_info",
    # Filter mode controls
    "filter_mode_combo": "combo_filter_mode",
    "filter_mode_label": "txt_filter_mode",
    "filter_oversample_info": "txt_filter_oversample_info",
    # Autotune UI elements
    "autotune_status": "txt_autotune_status",
    "autotune_progress": "prog_autotune",
    "autotune_results": "txt_autotune_results",
    "autotune_params": "grp_autotune_params",
    "autotune_pwm_low": "input_autotune_pwm_low",
    "autotune_pwm_high": "input_autotune_pwm_high",
    "autotune_pwm_apply": "btn_autotune_apply",
    "autotune_k_display": "txt_autotune_k",
    "autotune_L_display": "txt_autotune_L",
    "autotune_T_display": "txt_autotune_T"
}

# PI-Controller mode values (must match radio button items)
P_MODE_MANUAL = "M"
P_MODE_AUTO = "P"
P_MODE_AUTOTUNE = "T"  # Automatic PI tuning mode

# Tuple of valid mode strings for validation (immutable)
P_MODE_VALUES = (P_MODE_MANUAL, P_MODE_AUTO, P_MODE_AUTOTUNE)


# =========================================================================
# AUTOTUNE CONSTANTS
# =========================================================================

# Autotune state constants
TUNE_STATE_IDLE = "idle"
TUNE_STATE_PREPARING = "preparing"
TUNE_STATE_STEP_UP = "step_up"
TUNE_STATE_STEADY_STATE = "steady_state"
TUNE_STATE_STEP_DOWN = "step_down"
TUNE_STATE_COMPLETE = "complete"
TUNE_STATE_ERROR = "error"

# Default tuning parameters
TUNE_DEFAULT_PWM_LOW = 20      # Starting PWM % for step test
TUNE_DEFAULT_PWM_HIGH = 50     # Step PWM % for step test
TUNE_MIN_DURATION = 5.0        # Minimum test duration (seconds)
TUNE_STEADY_THRESHOLD = 0.02   # 2% variation for steady state detection
TUNE_MAX_DURATION = 30.0       # Maximum test duration (seconds)
TUNE_STABILITY_SAMPLES = 50    # Samples to confirm steady state
