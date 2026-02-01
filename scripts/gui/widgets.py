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
    on_p_mode_changed,
    on_p_setpoint_changed,
    on_p_gain_changed,
    on_p_ff_changed,
    on_p_stream_start_clicked,
    on_p_stream_stop_clicked,
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

        # P-Controller Section
        dpg.add_spacer(height=15)
        dpg.add_separator()
        dpg.add_spacer(height=10)

        dpg.add_text("P-Controller:", color=[200, 200, 200])

        # Mode switch
        dpg.add_spacer(height=5)
        with dpg.group(horizontal=True):
            dpg.add_text("Mode:", color=[180, 180, 180])
            dpg.add_spacer(width=10)
            dpg.add_radio_button(
                items=["Manual", "P-Control"],
                tag=TAGS["p_mode_radio"],
                default_value=0,
                callback=on_p_mode_changed,
                user_data=app,
                horizontal=True
            )

        # P-Control controls (hidden by default)
        with dpg.group(tag=TAGS["p_control_group"], show=False):
            # Setpoint
            dpg.add_spacer(height=10)
            dpg.add_text("Setpoint (ADC 0-4095):", color=[180, 180, 180])
            dpg.add_input_int(
                tag=TAGS["p_setpoint_input"],
                default_value=2000,
                min_value=0,
                max_value=4095,
                width=150,
                callback=on_p_setpoint_changed,
                user_data=app
            )

            # P-Gain slider
            dpg.add_spacer(height=8)
            dpg.add_text("P-Gain:", color=[180, 180, 180])
            with dpg.group(horizontal=True):
                dpg.add_slider_float(
                    tag=TAGS["p_gain_slider"],
                    default_value=1.0,
                    min_value=0.0,
                    max_value=10.0,
                    clamped=True,
                    width=200,
                    callback=on_p_gain_changed,
                    user_data=app
                )
                dpg.add_text(
                    tag=TAGS["p_gain_label"],
                    default_value="1.00",
                    color=[100, 200, 255]
                )

            # Feed Forward slider
            dpg.add_spacer(height=8)
            dpg.add_text("Feed Forward (PWM %):", color=[180, 180, 180])
            with dpg.group(horizontal=True):
                dpg.add_slider_int(
                    tag=TAGS["p_ff_slider"],
                    default_value=0,
                    min_value=0,
                    max_value=100,
                    clamped=True,
                    width=200,
                    callback=on_p_ff_changed,
                    user_data=app
                )
                dpg.add_text(
                    tag=TAGS["p_ff_label"],
                    default_value="0%",
                    color=[100, 200, 255]
                )

            # Calculated PWM display
            dpg.add_spacer(height=8)
            with dpg.group(horizontal=True):
                dpg.add_text("PWM Output:", color=[180, 180, 180])
                dpg.add_spacer(width=5)
                dpg.add_text(
                    tag=TAGS["p_pwm_output"],
                    default_value="--%",
                    color=[100, 255, 100]
                )

            # Streaming controls
            dpg.add_spacer(height=10)
            dpg.add_separator()
            dpg.add_spacer(height=8)
            dpg.add_text("P-Stream:", color=[200, 200, 200])

            with dpg.group(horizontal=True):
                dpg.add_text("Interval (ms):", color=[180, 180, 180])
                dpg.add_spacer(width=5)
                dpg.add_input_int(
                    tag=TAGS["p_stream_interval"],
                    default_value=10,
                    min_value=1,
                    max_value=1000,
                    width=100
                )

            dpg.add_spacer(height=5)
            with dpg.group(horizontal=True):
                dpg.add_button(
                    label="Start",
                    tag=TAGS["p_stream_start_btn"],
                    callback=on_p_stream_start_clicked,
                    user_data=app,
                    width=100,
                    enabled=False
                )
                dpg.add_spacer(width=10)
                dpg.add_button(
                    label="Stop",
                    tag=TAGS["p_stream_stop_btn"],
                    callback=on_p_stream_stop_clicked,
                    user_data=app,
                    width=100,
                    enabled=False
                )

            # P-Stream status
            dpg.add_spacer(height=5)
            dpg.add_text(
                tag=TAGS["p_stream_status"],
                default_value="P-Stream: Stopped",
                color=[150, 150, 150],
            )

            # P-Stream Plot
            dpg.add_spacer(height=10)
            dpg.add_text("P-Stream Data (setpoint, measured, PWM):", color=[150, 150, 150])
            dpg.add_spacer(height=5)

            with dpg.plot(
                tag=TAGS["p_stream_plot"],
                height=200,
                width=-1,
                no_menus=True,
                no_box_select=True,
            ):
                x_axis = dpg.add_plot_axis(dpg.mvXAxis, tag="p_stream_x_axis", label="Time (s ago)")

                # Left Y axis (ADC values 0-4095)
                y_axis_adc = dpg.add_plot_axis(dpg.mvYAxis, tag="p_stream_y_adc", label="ADC Value")

                # Right Y axis (PWM 0-100%)
                y_axis_pwm = dpg.add_plot_axis(dpg.mvYAxis2, tag="p_stream_y_pwm", label="PWM (%)")

                # Four series for P-stream
                dpg.add_line_series(
                    [], [],
                    tag=TAGS["p_series_setpoint"],
                    label="Setpoint",
                    parent=y_axis_adc,
                )
                dpg.add_line_series(
                    [], [],
                    tag=TAGS["p_series_measured"],
                    label="Measured",
                    parent=y_axis_adc,
                )
                dpg.add_line_series(
                    [], [],
                    tag=TAGS["p_series_pwm"],
                    label="PWM (%)",
                    parent=y_axis_pwm,
                )
                dpg.add_line_series(
                    [], [],
                    tag=TAGS["p_series_error"],
                    label="Error",
                    parent=y_axis_adc,
                )

                dpg.add_plot_legend()

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

        # ADC History Plot
        dpg.add_spacer(height=10)
        dpg.add_text("ADC History (last 100 samples):", color=[150, 150, 150])

        dpg.add_spacer(height=5)

        # ADC History plot with raw value and voltage
        with dpg.plot(
            tag=TAGS["adc_history_plot"],
            height=200,
            width=-1,
            no_menus=True,
            no_box_select=True,
        ):
            x_axis = dpg.add_plot_axis(dpg.mvXAxis, tag="adc_history_x_axis", label="Time (s ago)")
            y_axis = dpg.add_plot_axis(dpg.mvYAxis, tag="adc_history_y_axis", label="Value")

            # Two series: raw (0-4095) and voltage (0-3.3V)
            dpg.add_line_series(
                [], [],
                tag=TAGS["adc_series_raw"],
                label="Raw (0-4095)",
                parent=y_axis,
            )
            dpg.add_line_series(
                [], [],
                tag=TAGS["adc_series_voltage"],
                label="Voltage (V)",
                parent=y_axis,
            )

            dpg.add_plot_legend()

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
