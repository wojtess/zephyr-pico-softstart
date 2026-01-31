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
    on_adc_read_clicked,
    on_stream_start_clicked,
    on_stream_stop_clicked,
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

        # ADC section
        dpg.add_spacer(height=15)
        dpg.add_separator()
        dpg.add_spacer(height=10)

        dpg.add_text("ADC (GPIO26 - 0-3.3V):", color=[200, 200, 200])

        dpg.add_spacer(height=5)

        # ADC value displays
        with dpg.group(horizontal=True):
            dpg.add_text("Raw:", color=[180, 180, 180])
            dpg.add_spacer(width=5)
            dpg.add_text(
                tag=TAGS["adc_value_raw"],
                default_value="--",
                color=[100, 200, 255],
            )

            dpg.add_spacer(width=20)

            dpg.add_text("Voltage:", color=[180, 180, 180])
            dpg.add_spacer(width=5)
            dpg.add_text(
                tag=TAGS["adc_value_voltage"],
                default_value="-- V",
                color=[100, 255, 100],
            )

        # ADC button
        dpg.add_spacer(height=8)

        with dpg.group(horizontal=True):
            dpg.add_button(
                label="Read ADC",
                tag=TAGS["adc_read_btn"],
                callback=on_adc_read_clicked,
                user_data=app,
                width=120,
                enabled=False,
            )

        # ADC Plot
        dpg.add_spacer(height=10)
        dpg.add_text("ADC History (last 100 samples):", color=[150, 150, 150])

        dpg.add_spacer(height=5)

        # Create plot with x-axis tag
        with dpg.plot(
            tag=TAGS["adc_plot"],
            height=200,
            width=-1,
            no_menus=True,
            no_box_select=True,
        ):
            # Create x and y axes
            x_axis = dpg.add_plot_axis(dpg.mvXAxis, tag="adc_plot_x_axis", label="Time")
            y_axis = dpg.add_plot_axis(dpg.mvYAxis, tag="adc_plot_y_axis", label="Value")

            # Add two series: raw value (0-4095) and voltage (0-3.3V) - parent must be y_axis
            dpg.add_line_series(
                [],
                [],
                tag=TAGS["adc_series_raw"],
                label="Raw (0-4095)",
                parent=y_axis,
            )
            dpg.add_line_series(
                [],
                [],
                tag=TAGS["adc_series_voltage"],
                label="Voltage (V)",
                parent=y_axis,
            )

        # ADC Streaming section
        dpg.add_spacer(height=15)
        dpg.add_separator()
        dpg.add_spacer(height=10)

        dpg.add_text("ADC Streaming:", color=[200, 200, 200])

        dpg.add_spacer(height=5)

        # Streaming interval input
        with dpg.group(horizontal=True):
            dpg.add_text("Interval (ms):", color=[180, 180, 180])
            dpg.add_spacer(width=5)
            dpg.add_input_int(
                tag=TAGS["stream_interval_input"],
                default_value=100,
                min_value=10,
                max_value=60000,
                width=100,
            )

        # Streaming buttons
        dpg.add_spacer(height=8)

        with dpg.group(horizontal=True):
            dpg.add_button(
                label="Start Stream",
                tag=TAGS["stream_start_btn"],
                callback=on_stream_start_clicked,
                user_data=app,
                width=120,
                enabled=False,
            )

            dpg.add_spacer(width=10)

            dpg.add_button(
                label="Stop Stream",
                tag=TAGS["stream_stop_btn"],
                callback=on_stream_stop_clicked,
                user_data=app,
                width=120,
                enabled=False,
            )

        # Streaming status
        dpg.add_spacer(height=5)

        dpg.add_text(
            tag=TAGS["stream_status"],
            default_value="Stream: Stopped",
            color=[150, 150, 150],
        )

        # Task Queue section
        dpg.add_spacer(height=15)
        dpg.add_separator()
        dpg.add_spacer(height=10)

        dpg.add_text("Task Queue:", color=[200, 200, 200])

        dpg.add_spacer(height=5)

        with dpg.group(horizontal=True):
            dpg.add_text("Pending:", color=[180, 180, 180])
            dpg.add_spacer(width=5)
            dpg.add_text(
                tag=TAGS["queue_count"],
                default_value="0",
                color=[100, 200, 255],
            )

        dpg.add_spacer(height=3)

        with dpg.group(horizontal=True):
            dpg.add_text("Processing:", color=[180, 180, 180])
            dpg.add_spacer(width=5)
            dpg.add_text(
                tag=TAGS["queue_processing"],
                default_value="None",
                color=[150, 150, 150],
            )

    # Set as primary window
    dpg.set_primary_window(TAGS["primary_window"], True)
