"""
UI widget creation functions for RP2040 LED Control GUI.
"""

import logging

import dearpygui.dearpygui as dpg

from .constants import TAGS, P_MODE_VALUES
from .app import LEDControllerApp
from .callbacks import (
    on_refresh_clicked,
    on_port_selected,
    on_connect_clicked,
    on_led_slider_changed,
    on_pwm_slider_changed,
    on_adc_read_clicked,
    on_stream_start_clicked,
    on_stream_stop_clicked,
    on_p_mode_changed,
    on_p_setpoint_changed,
    on_p_gain_changed,
    on_p_gain_input_changed,
    on_p_ki_changed,
    on_p_ki_input_changed,
    on_p_ff_changed,
    on_p_ff_input_changed,
    on_p_stream_enable_changed,
    on_p_record_enable_changed,
    on_adc_stream_enable_changed,
)
from .utils import find_ports_with_info, update_status

logger = logging.getLogger(__name__)


def toggle_p_stream_plot(sender, app_data, user_data):
    """Toggle P-Stream plot visibility."""
    if dpg.does_item_exist(TAGS["p_stream_plot"]):
        is_visible = dpg.is_item_shown(TAGS["p_stream_plot"])
        dpg.configure_item(TAGS["p_stream_plot"], show=not is_visible)
        if dpg.does_item_exist(TAGS["adc_history_plot"]):
            if not is_visible:
                dpg.configure_item(TAGS["adc_history_plot"], height=300)
            else:
                dpg.configure_item(TAGS["adc_history_plot"], height=650)


def toggle_adc_plot(sender, app_data, user_data):
    """Toggle ADC History plot visibility."""
    if dpg.does_item_exist(TAGS["adc_history_plot"]):
        is_visible = dpg.is_item_shown(TAGS["adc_history_plot"])
        dpg.configure_item(TAGS["adc_history_plot"], show=not is_visible)
        if dpg.does_item_exist(TAGS["p_stream_plot"]):
            if not is_visible:
                dpg.configure_item(TAGS["p_stream_plot"], height=300)
            else:
                dpg.configure_item(TAGS["p_stream_plot"], height=650)


def create_main_window(app: LEDControllerApp) -> None:
    """Create window - ALL CONTROLS VISIBLE + PLOTS (NO TABS)."""
    with dpg.window(
        label="RP2040 LED Control",
        tag=TAGS["primary_window"],
        no_close=True,
        no_collapse=True,
        no_move=True,
    ):
        # Title
        dpg.add_text("RP2040 LED Control", color=[100, 200, 255])
        dpg.add_separator()
        dpg.add_spacer(height=3)

        # TOP: Connection + Mode
        with dpg.group(horizontal=True):
            with dpg.group(width=220):
                initial_ports = find_ports_with_info()
                display_list = [f"{p['description'][:10]} - {p['device']}" for p in initial_ports] if initial_ports else ["No device"]
                dpg.add_combo(items=display_list, tag=TAGS["port_combo"], default_value=display_list[0] if display_list else "", width=-1, callback=on_port_selected, user_data=app)
                with dpg.group(horizontal=True):
                    dpg.add_button(label="Refresh", tag=TAGS["refresh_btn"], callback=on_refresh_clicked, user_data=app, width=85)
                    dpg.add_button(label="Connect", tag=TAGS["connect_btn"], callback=on_connect_clicked, user_data=app, width=-1)
            with dpg.group(width=80):
                with dpg.group(horizontal=True):
                    dpg.add_text("Conn:")
                    dpg.add_text(tag=TAGS["conn_indicator"], default_value="Off", color=[150, 150, 150])
                with dpg.group(horizontal=True):
                    dpg.add_text("LED:")
                    dpg.add_text(tag=TAGS["led_indicator"], default_value="OFF", color=[150, 150, 150])
            with dpg.group(width=70):
                dpg.add_text("Mode:")
                dpg.add_radio_button(items=P_MODE_VALUES, tag=TAGS["p_mode_radio"], default_value=0, callback=on_p_mode_changed, user_data=app, horizontal=True)

        dpg.add_spacer(height=5)

        # MAIN: Controls (left) + Plots (right) - BOTH ALWAYS VISIBLE
        with dpg.group(horizontal=True):

            # LEFT: ALL CONTROLS (320px, always visible)
            with dpg.group(width=320):

                # MANUAL
                dpg.add_text("--- MANUAL ---", color=[150, 150, 180])
                dpg.add_spacer(height=2)

                dpg.add_text("LED:")
                dpg.add_checkbox(label="ON", tag=TAGS["led_btn"], default_value=False, callback=on_led_slider_changed, user_data=app, enabled=False)

                dpg.add_spacer(height=3)
                dpg.add_text("PWM %:")
                dpg.add_slider_int(tag=TAGS["pwm_slider"], default_value=0, min_value=0, max_value=100, clamped=True, width=-1, callback=on_pwm_slider_changed, user_data=app)
                dpg.add_text(tag=TAGS["pwm_label"], default_value="0%", color=[100, 200, 255])

                dpg.add_spacer(height=3)
                with dpg.group(horizontal=True):
                    dpg.add_text("Raw:", color=[150, 150, 150])
                    dpg.add_text(tag=TAGS["adc_value_raw"], default_value="--", color=[100, 200, 255])
                    dpg.add_spacer(width=5)
                    dpg.add_text("Amp:", color=[150, 150, 150])
                    dpg.add_text(tag=TAGS["adc_value_amps"], default_value="-- A", color=[100, 255, 100])
                dpg.add_button(label="Read ADC", tag=TAGS["adc_read_btn"], callback=on_adc_read_clicked, user_data=app, width=-1, enabled=False)

                dpg.add_spacer(height=2)
                dpg.add_text("ADC Stream:")
                dpg.add_checkbox(label="Enable", tag=TAGS["adc_stream_enable_chk"], default_value=False, callback=on_adc_stream_enable_changed, user_data=app)

                dpg.add_spacer(height=8)

                # P-CONTROLLER
                dpg.add_text("--- P-CTRL ---", color=[150, 150, 180])
                dpg.add_spacer(height=2)

                dpg.add_text("Setpoint (0-4095):")
                dpg.add_input_int(tag=TAGS["p_setpoint_input"], default_value=2000, min_value=0, max_value=4095, width=-1, callback=on_p_setpoint_changed, user_data=app)
                dpg.add_slider_int(tag=TAGS["p_setpoint_slider"], default_value=2000, min_value=0, max_value=4095, clamped=True, width=-1, callback=on_p_setpoint_changed, user_data=app)

                dpg.add_spacer(height=3)

                dpg.add_text("Gain:")
                dpg.add_input_float(tag=TAGS["p_gain_input"], default_value=1.0, min_value=0.0, max_value=10.0, width=-1, format="%.2f", callback=on_p_gain_input_changed, user_data=app)
                dpg.add_slider_float(tag=TAGS["p_gain_slider"], default_value=1.0, min_value=0.0, max_value=10.0, clamped=True, width=-1, callback=on_p_gain_changed, user_data=app)
                dpg.add_text(tag=TAGS["p_gain_label"], default_value="1.00")

                dpg.add_spacer(height=3)

                dpg.add_text("Ki:")
                dpg.add_input_float(tag=TAGS["p_ki_input"], default_value=0.0, min_value=0.0, max_value=10.0, width=-1, format="%.2f", callback=on_p_ki_input_changed, user_data=app)
                dpg.add_slider_float(tag=TAGS["p_ki_slider"], default_value=0.0, min_value=0.0, max_value=10.0, clamped=True, width=-1, callback=on_p_ki_changed, user_data=app)
                dpg.add_text(tag=TAGS["p_ki_label"], default_value="0.00")

                dpg.add_spacer(height=3)

                dpg.add_text("Feed-Forward (PWM %):")
                dpg.add_input_int(tag=TAGS["p_ff_input"], default_value=0, min_value=0, max_value=100, width=-1, callback=on_p_ff_input_changed, user_data=app)
                dpg.add_slider_int(tag=TAGS["p_ff_slider"], default_value=0, min_value=0, max_value=100, clamped=True, width=-1, callback=on_p_ff_changed, user_data=app)

                dpg.add_spacer(height=3)
                with dpg.group(horizontal=True):
                    dpg.add_text("PWM out:")
                    dpg.add_text(tag=TAGS["p_pwm_output"], default_value="--", color=[100, 255, 100])

                dpg.add_spacer(height=8)

                # STREAMING
                dpg.add_text("--- STREAM ---", color=[150, 150, 180])
                dpg.add_spacer(height=2)

                with dpg.group(horizontal=True):
                    dpg.add_text("ms:")
                    dpg.add_input_int(tag=TAGS["p_stream_interval"], default_value=10, min_value=1, max_value=1000, width=60)

                dpg.add_spacer(height=2)
                with dpg.group(horizontal=True):
                    dpg.add_text("P-Stream:")
                    dpg.add_checkbox(label="Enable", tag=TAGS["p_stream_enable_chk"], default_value=False, callback=on_p_stream_enable_changed, user_data=app)
                dpg.add_text(tag=TAGS["p_stream_status"], default_value="Stopped", color=[150, 150, 150])

                dpg.add_spacer(height=5)
                with dpg.group(horizontal=True):
                    dpg.add_text("Record:")
                    dpg.add_checkbox(label="Rec", tag=TAGS["p_record_enable_chk"], default_value=False, callback=on_p_record_enable_changed, user_data=app)
                dpg.add_text(tag=TAGS["p_record_status"], default_value="Stopped", color=[150, 150, 150])

                dpg.add_spacer(height=8)

                # STATUS
                dpg.add_text("--- STATUS ---", color=[150, 150, 180])
                dpg.add_spacer(height=2)
                dpg.add_text(tag=TAGS["queue_count"], default_value="Q: 0", color=[100, 200, 255])
                dpg.add_text(tag=TAGS["queue_processing"], default_value="--", color=[150, 150, 150])

            # RIGHT: PLOTS (always visible, takes remaining space)
            with dpg.group():
                dpg.add_spacer(width=8)

                # Plot toggle buttons
                with dpg.group(horizontal=True):
                    dpg.add_button(label="P-Stream", callback=toggle_p_stream_plot, width=80, user_data=app)
                    dpg.add_spacer(width=3)
                    dpg.add_button(label="ADC", callback=toggle_adc_plot, width=80, user_data=app)

                dpg.add_spacer(height=3)

                # P-Stream Plot
                with dpg.plot(
                    tag=TAGS["p_stream_plot"],
                    height=300,
                    width=-1,
                    show=True,
                    no_menus=True,
                    no_box_select=True,
                ):
                    x_axis = dpg.add_plot_axis(dpg.mvXAxis, label="Time (s)")
                    y_axis_adc = dpg.add_plot_axis(dpg.mvYAxis, label="ADC Value")
                    y_axis_pwm = dpg.add_plot_axis(dpg.mvYAxis2, label="PWM %")

                    dpg.add_line_series([], [], tag=TAGS["p_series_setpoint"], label="Setpoint", parent=y_axis_adc)
                    dpg.add_line_series([], [], tag=TAGS["p_series_measured"], label="Measured", parent=y_axis_adc)
                    dpg.add_line_series([], [], tag=TAGS["p_series_pwm"], label="PWM", parent=y_axis_pwm)
                    dpg.add_line_series([], [], tag=TAGS["p_series_error"], label="Error", parent=y_axis_adc)
                    dpg.add_plot_legend()

                # ADC History Plot
                dpg.add_spacer(height=10)
                with dpg.plot(
                    tag=TAGS["adc_history_plot"],
                    height=300,
                    width=-1,
                    show=True,
                    no_menus=True,
                    no_box_select=True,
                ):
                    x_axis = dpg.add_plot_axis(dpg.mvXAxis, label="Time (s)")
                    y_axis = dpg.add_plot_axis(dpg.mvYAxis, label="Value")

                    dpg.add_line_series([], [], tag=TAGS["adc_series_raw"], label="Raw (0-4095)", parent=y_axis)
                    dpg.add_line_series([], [], tag=TAGS["adc_series_amps"], label="Current (A)", parent=y_axis)
                    dpg.add_plot_legend()

                # Status bar
                dpg.add_spacer(height=10)
                dpg.add_separator()
                dpg.add_spacer(height=5)

                dpg.add_text(
                    tag=TAGS["status_message"],
                    default_value="Select port to connect",
                    wrap=500,
                )
                dpg.add_text(
                    tag=TAGS["last_response"],
                    default_value="Last: None",
                    color=[180, 180, 180],
                )

    # Set as primary window
    dpg.set_primary_window(TAGS["primary_window"], True)
