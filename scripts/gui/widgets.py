"""
UI widget creation functions for RP2040 LED Control GUI.
"""

import logging

import dearpygui.dearpygui as dpg

from .constants import TAGS
from .app import LEDControllerApp
from .callbacks import (
    on_refresh_clicked,
    on_port_selected,
    on_connect_clicked,
    on_led_toggle_clicked,
    on_pwm_slider_changed,
)
from .utils import find_ports_with_info, update_status

logger = logging.getLogger(__name__)


def create_main_window(app: LEDControllerApp) -> None:
    """Create the primary application window."""
    with dpg.window(
        label="RP2040 LED Control",
        tag=TAGS["primary_window"],
        no_close=True,
        no_collapse=True,
        no_move=True,
        no_resize=True,
    ):
        dpg.add_text("RP2040 LED Control", color=[100, 200, 255])
        dpg.add_separator()
        dpg.add_spacer(height=10)

        # Serial port selection
        dpg.add_text("Serial Port:")
        dpg.add_spacer(height=5)

        # Get initial ports
        initial_ports = find_ports_with_info()
        if initial_ports:
            display_list = [f"{p['description']} - {p['device']}" for p in initial_ports]
        else:
            display_list = ["No device detected"]

        dpg.add_combo(
            items=display_list,
            tag=TAGS["port_combo"],
            default_value=display_list[0] if display_list else "",
            width=-1,
            callback=on_port_selected,
            user_data=app,
        )

        # Connection buttons
        dpg.add_spacer(height=10)
        with dpg.group(horizontal=True):
            dpg.add_button(
                label="Refresh",
                tag=TAGS["refresh_btn"],
                callback=on_refresh_clicked,
                user_data=app,
                width=120,
            )

            dpg.add_spacer(width=10)

            dpg.add_button(
                label="Connect",
                tag=TAGS["connect_btn"],
                callback=on_connect_clicked,
                user_data=app,
                width=-1,
            )

        # Status indicators
        dpg.add_spacer(height=15)
        dpg.add_separator()
        dpg.add_spacer(height=10)

        with dpg.group(horizontal=True):
            dpg.add_text("Connection:", color=[200, 200, 200])
            dpg.add_spacer(width=10)
            dpg.add_text(tag=TAGS["conn_indicator"], default_value="Disconnected", color=[150, 150, 150])

        dpg.add_spacer(height=8)

        with dpg.group(horizontal=True):
            dpg.add_text("LED Status:", color=[200, 200, 200])
            dpg.add_spacer(width=10)
            dpg.add_text(tag=TAGS["led_indicator"], default_value="LED OFF", color=[150, 150, 150])

        # LED control
        dpg.add_spacer(height=15)
        dpg.add_separator()
        dpg.add_spacer(height=10)

        dpg.add_button(
            label="Turn LED ON",
            tag=TAGS["led_btn"],
            callback=on_led_toggle_clicked,
            user_data=app,
            width=-1,
            enabled=False,
        )

        # PWM control
        dpg.add_spacer(height=15)
        dpg.add_text("PWM Duty Cycle:", color=[200, 200, 200])

        dpg.add_spacer(height=5)

        with dpg.group(horizontal=True):
            dpg.add_slider_int(
                tag=TAGS["pwm_slider"],
                default_value=0,
                min_value=0,
                max_value=100,
                clamped=True,
                width=-1,
                callback=on_pwm_slider_changed,
                user_data=app,
            )

            dpg.add_spacer(width=10)

            dpg.add_text(
                tag=TAGS["pwm_label"],
                default_value="0%",
                color=[100, 200, 255],
            )

        # Status messages
        dpg.add_spacer(height=15)
        dpg.add_separator()
        dpg.add_spacer(height=10)

        dpg.add_text(
            tag=TAGS["status_message"],
            default_value="Select a serial port to connect",
            wrap=500,
        )

        dpg.add_spacer(height=8)
        dpg.add_text(
            tag=TAGS["last_response"],
            default_value="Last: None",
            color=[180, 180, 180],
        )

    # Set as primary window
    dpg.set_primary_window(TAGS["primary_window"], True)
