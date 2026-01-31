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
    CHECK_HEALTH = "check_health"
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
    "adc_plot": "plot_adc",
    "adc_value_raw": "txt_adc_raw",
    "adc_value_voltage": "txt_adc_voltage",
    "adc_series_raw": "series_adc_raw",
    "adc_series_voltage": "series_adc_voltage",
    # Task queue tags
    "queue_label": "txt_queue_label",
    "queue_count": "txt_queue_count",
    "queue_processing": "txt_queue_processing",
}
