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


@dataclass
class SerialResult:
    """Result from serial worker thread."""
    command: SerialCommand
    success: bool
    message: str
    led_state: Optional[bool] = None
    error: Optional[str] = None


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
}
