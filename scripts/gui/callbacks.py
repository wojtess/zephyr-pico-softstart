"""
Callback functions for RP2040 LED Control GUI.
"""

import queue
import threading
import logging
import time
from typing import Optional

import serial
import dearpygui.dearpygui as dpg

from .constants import TAGS, SerialCommand, SerialTask, P_MODE_MANUAL, P_MODE_AUTO
from .app import LEDControllerApp
from .utils import (
    find_ports_with_info,
    update_status,
    update_connection_indicator,
    update_led_indicator,
    update_last_response,
    get_selected_port,
    handle_disconnect_state,
    update_queue_display,
)

logger = logging.getLogger(__name__)


def on_refresh_clicked(sender, app_data, user_data: LEDControllerApp) -> None:
    """Callback for refresh button click."""
    # Disable button
    if dpg.does_item_exist(sender):
        dpg.configure_item(sender, enabled=False)

    update_status("Scanning for devices...", [200, 200, 100])
    dpg.split_frame()

    # Get ports
    ports = find_ports_with_info()

    # Update dropdown
    if dpg.does_item_exist(TAGS["port_combo"]):
        if ports:
            display_list = [f"{p['description']} - {p['device']}" for p in ports]
            dpg.configure_item(TAGS["port_combo"], items=display_list, enabled=True)

            if len(ports) == 1:
                dpg.set_value(TAGS["port_combo"], display_list[0])
                update_status(f"Found 1 device: {ports[0]['device']}", [100, 200, 100])
            else:
                update_status(f"Found {len(ports)} devices", [100, 200, 100])
        else:
            dpg.configure_item(TAGS["port_combo"], items=["No device found"], enabled=True)
            update_status("No device found. Connect RP2040 via USB.", [255, 150, 50])

    # Re-enable button
    if dpg.does_item_exist(sender):
        dpg.configure_item(sender, enabled=True)


def on_port_selected(sender, app_data, user_data: LEDControllerApp) -> None:
    """Callback for port selection dropdown."""
    selected = dpg.get_value(sender)
    logger.info(f"Port selected: {selected}")

    # Parse device from selection
    if " - " in selected:
        device = selected.split(" - ")[1]
        logger.info(f"Device parsed: {device}")
        # Store selected device
        user_data.set_port(device)


def on_connect_clicked(sender, app_data, user_data: LEDControllerApp) -> None:
    """Callback for connect button click."""
    port = get_selected_port()
    if not port:
        update_status("No port selected", [255, 150, 50])
        return

    logger.info(f"Connect button clicked, port: {port}")

    # Send connect command to worker thread
    task = SerialTask(command=SerialCommand.CONNECT, port=port)
    result_queue = user_data.send_task(task)

    # Check result in background thread
    def check_connect():
        try:
            result = result_queue.get(timeout=5)

            if result.success:
                update_status("Connected!", [100, 200, 100])
                update_last_response("Connected to device")

                # Update UI to connected state
                update_connection_indicator(True)

                # Enable LED and ADC buttons
                if dpg.does_item_exist(TAGS["led_btn"]):
                    dpg.configure_item(TAGS["led_btn"], enabled=True)
                if dpg.does_item_exist(TAGS["adc_read_btn"]):
                    dpg.configure_item(TAGS["adc_read_btn"], enabled=True)
            else:
                update_status(f"Connection failed: {result.message}", [255, 100, 100])
                update_last_response(f"Failed: {result.error or 'Unknown'}")

        except queue.Empty:
            update_status("Connection timeout", [255, 150, 50])
            update_last_response("Timeout")

    threading.Thread(target=check_connect, daemon=True).start()


def on_led_toggle_clicked(sender, app_data, user_data: LEDControllerApp) -> None:
    """Callback for LED toggle button."""
    # Disable button
    if dpg.does_item_exist(sender):
        dpg.configure_item(sender, enabled=False)

    new_state = not user_data.get_led_state()

    update_status(f"Turning LED {'ON' if new_state else 'OFF'}...", [200, 200, 100])
    dpg.split_frame()

    task = SerialTask(command=SerialCommand.SEND_LED, led_state=new_state)
    result_queue = user_data.send_task(task)

    # Check result in separate thread
    def check_led():
        try:
            result = result_queue.get(timeout=2)

            # Re-enable button
            if dpg.does_item_exist(sender):
                dpg.configure_item(sender, enabled=True)

            if result.success:
                user_data.set_led_state(new_state)
                update_status(f"LED is {'ON' if new_state else 'OFF'}", [100, 200, 100])
                update_last_response(f"ACK: LED={'ON' if new_state else 'OFF'}")
                update_led_indicator(new_state)

                # Update LED button text if it exists
                if dpg.does_item_exist(TAGS["led_btn"]):
                    dpg.set_item_label(TAGS["led_btn"], "Turn LED OFF" if new_state else "Turn LED ON")
            else:
                update_status(f"LED error: {result.message}", [255, 100, 100])
                update_last_response(f"NACK: {result.error or 'Unknown'}")

                # Check if disconnected
                if result.error == "Disconnected" or result.error == "Timeout":
                    handle_disconnect_state(user_data)

        except queue.Empty:
            update_status("LED timeout", [255, 150, 50])
            update_last_response("Timeout")

            # Re-enable button
            if dpg.does_item_exist(sender):
                dpg.configure_item(sender, enabled=True)

    threading.Thread(target=check_led, daemon=True).start()


def on_pwm_slider_changed(sender, app_data, user_data: LEDControllerApp) -> None:
    """Handle PWM slider change."""
    duty = app_data  # 0-100
    logger.info(f"PWM slider changed to {duty}%")

    # Update label
    if dpg.does_item_exist(TAGS["pwm_label"]):
        dpg.set_value(TAGS["pwm_label"], f"{duty}%")

    # Send command to device
    task = SerialTask(command=SerialCommand.SEND_PWM, pwm_duty=duty)
    result_queue = user_data.send_task(task)

    # Check result in background thread
    def check_pwm():
        try:
            result = result_queue.get(timeout=2)

            if result.success:
                update_status(f"PWM set to {duty}%", [100, 200, 100])
                update_last_response(f"ACK: PWM={duty}%")
            else:
                if result.error == "Disconnected" or result.error == "Timeout":
                    handle_disconnect_state(user_data)

                update_status(f"PWM error: {result.message}", [255, 100, 100])
                update_last_response(f"NACK: {result.error or 'Unknown'}")

        except queue.Empty:
            update_status("PWM change timeout", [255, 150, 50])
            update_last_response("Timeout")

    threading.Thread(target=check_pwm, daemon=True).start()


def on_adc_read_clicked(sender, app_data, user_data: LEDControllerApp) -> None:
    """Handle single ADC read button click."""
    logger.info("ADC read button clicked")

    # Send read command to device
    task = SerialTask(command=SerialCommand.READ_ADC)
    result_queue = user_data.send_task(task)

    # Check result in background thread
    def check_adc():
        try:
            result = result_queue.get(timeout=2)

            if result.success:
                # Update ADC value displays
                if result.adc_value is not None:
                    raw = result.adc_value
                    voltage = (raw / 4095.0) * 3.3
                    if dpg.does_item_exist(TAGS["adc_value_raw"]):
                        dpg.set_value(TAGS["adc_value_raw"], f"{raw}")
                    if dpg.does_item_exist(TAGS["adc_value_voltage"]):
                        dpg.set_value(TAGS["adc_value_voltage"], f"{voltage:.3f} V")

                update_status(f"ADC read: {result.adc_value}", [100, 200, 100])
                update_last_response(f"ACK: ADC={result.adc_value}")
            else:
                if result.error == "Disconnected" or result.error == "Timeout":
                    handle_disconnect_state(user_data)

                update_status(f"ADC read error: {result.message}", [255, 100, 100])
                update_last_response(f"NACK: {result.error or 'Unknown'}")

        except queue.Empty:
            update_status("ADC read timeout", [255, 150, 50])
            update_last_response("Timeout")

    threading.Thread(target=check_adc, daemon=True).start()


def on_stream_start_clicked(sender, app_data, user_data: LEDControllerApp) -> None:
    """Callback for stream start button click."""
    logger.info("Stream start button clicked")

    # Disable start button, enable stop button
    if dpg.does_item_exist(TAGS["stream_start_btn"]):
        dpg.configure_item(TAGS["stream_start_btn"], enabled=False)
    if dpg.does_item_exist(TAGS["stream_stop_btn"]):
        dpg.configure_item(TAGS["stream_stop_btn"], enabled=True)

    update_status("Starting ADC stream...", [200, 200, 100])

    # Send start command to device
    task = SerialTask(command=SerialCommand.START_STREAM, stream_interval=100)  # default 100ms
    result_queue = user_data.send_task(task)

    # Check result in background thread
    def check_start():
        try:
            result = result_queue.get(timeout=2)

            if result.success:
                update_status("ADC streaming started", [100, 200, 100])
                update_last_response("ACK: Streaming started")

                # Update streaming state in app
                user_data.set_streaming(True)
            else:
                # Check if disconnected
                if result.error == "Disconnected" or result.error == "Timeout":
                    handle_disconnect_state(user_data)

                update_status(f"Stream start error: {result.message}", [255, 100, 100])
                update_last_response(f"NACK: {result.error or 'Unknown'}")

                # Re-enable start button, disable stop button
                if dpg.does_item_exist(TAGS["stream_start_btn"]):
                    dpg.configure_item(TAGS["stream_start_btn"], enabled=True)
                if dpg.does_item_exist(TAGS["stream_stop_btn"]):
                    dpg.configure_item(TAGS["stream_stop_btn"], enabled=False)

        except queue.Empty:
            update_status("Stream start timeout", [255, 150, 50])
            update_last_response("Timeout")

            # Re-enable start button, disable stop button
            if dpg.does_item_exist(TAGS["stream_start_btn"]):
                dpg.configure_item(TAGS["stream_start_btn"], enabled=True)
            if dpg.does_item_exist(TAGS["stream_stop_btn"]):
                dpg.configure_item(TAGS["stream_stop_btn"], enabled=False)

    threading.Thread(target=check_start, daemon=True).start()


def on_stream_stop_clicked(sender, app_data, user_data: LEDControllerApp) -> None:
    """Callback for stream stop button click."""
    logger.info("Stream stop button clicked")

    # Disable stop button
    if dpg.does_item_exist(TAGS["stream_stop_btn"]):
        dpg.configure_item(TAGS["stream_stop_btn"], enabled=False)

    update_status("Stopping ADC stream...", [200, 200, 100])

    # Send stop command to device
    task = SerialTask(command=SerialCommand.STOP_STREAM)
    result_queue = user_data.send_task(task)

    # Check result in background thread
    def check_stop():
        try:
            result = result_queue.get(timeout=2)

            if result.success:
                update_status("ADC stream stopped", [100, 200, 100])
                update_last_response("ACK: Stream stopped")

                # Update streaming state in app
                user_data.set_streaming(False)

                # Clear the button, disable stop button
                if dpg.does_item_exist(TAGS["stream_stop_btn"]):
                    dpg.configure_item(TAGS["stream_stop_btn"], enabled=False)
            else:
                # Check if disconnected
                if result.error == "Disconnected" or result.error == "Timeout":
                    handle_disconnect_state(user_data)

                update_status(f"Stream stop error: {result.message}", [255, 100, 100])
                update_last_response(f"NACK: {result.error or 'Unknown'}")

                # Re-enable stop button
                if dpg.does_item_exist(TAGS["stream_stop_btn"]):
                    dpg.does_item_exist(TAGS["stream_stop_btn"], enabled=True)

        except queue.Empty:
            update_status("Stream stop timeout", [255, 150, 50])
            update_last_response("Timeout")

            # Re-enable stop button
            if dpg.does_item_exist(TAGS["stream_stop_btn"]):
                dpg.configure_item(TAGS["stream_stop_btn"], enabled=True)

    threading.Thread(target=check_stop, daemon=True).start()


def on_p_mode_changed(sender, app_data, user_data: LEDControllerApp) -> None:
    """Handle mode switch (Manual <-> PI-Control)."""
    # app_data is the selected item name (string) from radio_button items
    # Use constants P_MODE_MANUAL ("M") and P_MODE_AUTO ("P")
    if app_data == P_MODE_MANUAL:
        mode = 0
    elif app_data == P_MODE_AUTO:
        mode = 1
    else:
        logger.error(f"Invalid mode string: {app_data}")
        return

    logger.info(f"PI-Mode changed to {app_data} ({mode})")

    # Send command to device
    task = SerialTask(command=SerialCommand.SET_P_MODE, p_mode=mode)
    result_queue = user_data.send_task(task)

    # Check result in background thread
    def check_mode():
        try:
            result = result_queue.get(timeout=2)

            if result.success:
                mode_name = "Manual" if mode == 0 else "PI-Control"
                update_status(f"Mode set to {mode_name}", [100, 200, 100])
                update_last_response(f"ACK: {mode_name} mode")

                # Enable/disable PWM slider based on mode
                # In PI-Control mode, PWM is calculated by firmware, not manual slider
                if dpg.does_item_exist(TAGS["pwm_slider"]):
                    if mode == 1:  # PI-Control mode
                        dpg.configure_item(TAGS["pwm_slider"], enabled=False)
                    else:  # Manual mode
                        dpg.configure_item(TAGS["pwm_slider"], enabled=True)

                # When switching to AUTO mode, sync gain and feed_forward to firmware
                # This ensures firmware values match GUI slider positions
                if mode == 1:  # PI-Control (AUTO) mode
                    sync_p_params_to_firmware(user_data)

            else:
                # Check if disconnected
                if result.error == "Disconnected" or result.error == "Timeout":
                    handle_disconnect_state(user_data)

                update_status(f"Mode error: {result.message}", [255, 100, 100])
                update_last_response(f"NACK: {result.error or 'Unknown'}")
                # Revert radio button
                if dpg.does_item_exist(TAGS["p_mode_radio"]):
                    dpg.set_value(TAGS["p_mode_radio"], "Manual")

        except queue.Empty:
            update_status("Mode change timeout", [255, 150, 50])
            update_last_response("Timeout")
            # Revert radio button
            if dpg.does_item_exist(TAGS["p_mode_radio"]):
                dpg.set_value(TAGS["p_mode_radio"], "Manual")

    threading.Thread(target=check_mode, daemon=True).start()


def on_p_setpoint_changed(sender, app_data, user_data: LEDControllerApp) -> None:
    """Handle setpoint input/slider change."""
    setpoint = app_data  # app_data contains the new setpoint value
    logger.info(f"PI-Setpoint changed to {setpoint}")

    # Sync slider and input
    if sender == TAGS["p_setpoint_slider"]:
        # Slider changed - update input
        if dpg.does_item_exist(TAGS["p_setpoint_input"]):
            dpg.set_value(TAGS["p_setpoint_input"], setpoint)
    elif sender == TAGS["p_setpoint_input"]:
        # Input changed - update slider
        if dpg.does_item_exist(TAGS["p_setpoint_slider"]):
            dpg.set_value(TAGS["p_setpoint_slider"], setpoint)

    # Send command to device
    task = SerialTask(command=SerialCommand.SET_P_SETPOINT, p_setpoint=setpoint)
    result_queue = user_data.send_task(task)

    # Check result in background thread
    def check_setpoint():
        try:
            result = result_queue.get(timeout=2)

            if result.success:
                update_status(f"Setpoint set to {setpoint}", [100, 200, 100])
                update_last_response(f"ACK: SP={setpoint}")
            else:
                # Check if disconnected
                if result.error == "Disconnected" or result.error == "Timeout":
                    handle_disconnect_state(user_data)

                update_status(f"Setpoint error: {result.message}", [255, 100, 100])
                update_last_response(f"NACK: {result.error or 'Unknown'}")

        except queue.Empty:
            update_status("Setpoint change timeout", [255, 150, 50])
            update_last_response("Timeout")

    threading.Thread(target=check_setpoint, daemon=True).start()


def on_p_setpoint_step_clicked(sender, app_data, user_data: tuple) -> None:
    """Handle setpoint step button click (-100, -1, +1, +100)."""
    app, step = user_data  # Unpack (app, step_value)
    logger.info(f"PI-Setpoint step by {step}")

    # Get current value, apply step, clamp to valid range
    current = dpg.get_value(TAGS["p_setpoint_input"])
    new_val = max(0, min(4095, current + step))

    # Update input field
    dpg.set_value(TAGS["p_setpoint_input"], new_val)

    # Trigger the main callback to send to firmware
    on_p_setpoint_changed(sender, new_val, app)


def on_p_gain_changed(sender, app_data, user_data: LEDControllerApp) -> None:
    """Handle P-gain slider change."""
    gain = app_data  # app_data contains the new gain value
    logger.info(f"PI-Gain slider changed to {gain:.2f}")

    # Update label
    if dpg.does_item_exist(TAGS["p_gain_label"]):
        dpg.set_value(TAGS["p_gain_label"], f"{gain:.2f}")

    # Update input field to match slider
    if dpg.does_item_exist(TAGS["p_gain_input"]):
        dpg.set_value(TAGS["p_gain_input"], gain)

    # Send command to device
    task = SerialTask(command=SerialCommand.SET_P_GAIN, p_gain=gain)
    result_queue = user_data.send_task(task)

    # Check result in background thread
    def check_gain():
        try:
            result = result_queue.get(timeout=2)

            if result.success:
                update_status(f"Gain set to {gain:.2f}", [100, 200, 100])
                update_last_response(f"ACK: Gain={gain:.2f}")
            else:
                if result.error == "Disconnected" or result.error == "Timeout":
                    handle_disconnect_state(user_data)

                update_status(f"Gain error: {result.message}", [255, 100, 100])
                update_last_response(f"NACK: {result.error or 'Unknown'}")

        except queue.Empty:
            update_status("Gain change timeout", [255, 150, 50])
            update_last_response("Timeout")

    threading.Thread(target=check_gain, daemon=True).start()


def on_p_gain_input_changed(sender, app_data, user_data: LEDControllerApp) -> None:
    """Handle P-gain input field change."""
    gain = max(0.0, min(10.0, app_data))  # Clamp
    logger.info(f"PI-Gain input changed to {gain:.2f}")

    # Update slider
    if dpg.does_item_exist(TAGS["p_gain_slider"]):
        dpg.set_value(TAGS["p_gain_slider"], gain)

    # Send command to device
    task = SerialTask(command=SerialCommand.SET_P_GAIN, p_gain=gain)
    result_queue = user_data.send_task(task)

    # Check result in background thread
    def check_gain():
        try:
            result = result_queue.get(timeout=2)

            if result.success:
                update_status(f"Gain set to {gain:.2f}", [100, 200, 100])
                update_last_response(f"ACK: Gain={gain:.2f}")
            else:
                if result.error == "Disconnected" or result.error == "Timeout":
                    handle_disconnect_state(user_data)

                update_status(f"Gain error: {result.message}", [255, 100, 100])
                update_last_response(f"NACK: {result.error or 'Unknown'}")

        except queue.Empty:
            update_status("Gain change timeout", [255, 150, 50])
            update_last_response("Timeout")

    threading.Thread(target=check_gain, daemon=True).start()


def on_p_ki_changed(sender, app_data, user_data: LEDControllerApp) -> None:
    """Handle PI-Ki slider change."""
    ki = app_data  # app_data contains the new Ki value
    logger.info(f"PI-Ki slider changed to {ki:.2f}")

    # Update label
    if dpg.does_item_exist(TAGS["p_ki_label"]):
        dpg.set_value(TAGS["p_ki_label"], f"{ki:.2f}")

    # Update input field to match slider
    if dpg.does_item_exist(TAGS["p_ki_input"]):
        dpg.set_value(TAGS["p_ki_input"], ki)

    # Send command to device
    task = SerialTask(command=SerialCommand.SET_P_KI, p_ki=ki)
    result_queue = user_data.send_task(task)

    # Check result in background thread
    def check_ki():
        try:
            result = result_queue.get(timeout=2)

            if result.success:
                update_status(f"Ki set to {ki:.2f}", [100, 200, 100])
                update_last_response(f"ACK: Ki={ki:.2f}")
            else:
                if result.error == "Disconnected" or result.error == "Timeout":
                    handle_disconnect_state(user_data)

                update_status(f"Ki error: {result.message}", [255, 100, 100])
                update_last_response(f"NACK: {result.error or 'Unknown'}")

        except queue.Empty:
            update_status("Ki change timeout", [255, 150, 50])
            update_last_response("Timeout")

    threading.Thread(target=check_ki, daemon=True).start()


def on_p_ki_input_changed(sender, app_data, user_data: LEDControllerApp) -> None:
    """Handle PI-Ki input field change."""
    ki = max(0.0, min(10.0, app_data))  # Clamp
    logger.info(f"PI-Ki input changed to {ki:.2f}")

    # Update slider
    if dpg.does_item_exist(TAGS["p_ki_slider"]):
        dpg.set_value(TAGS["p_ki_slider"], ki)

    # Send command to device
    task = SerialTask(command=SerialCommand.SET_P_KI, p_ki=ki)
    result_queue = user_data.send_task(task)

    # Check result in background thread
    def check_ki():
        try:
            result = result_queue.get(timeout=2)

            if result.success:
                update_status(f"Ki set to {ki:.2f}", [100, 200, 100])
                update_last_response(f"ACK: Ki={ki:.2f}")
            else:
                if result.error == "Disconnected" or result.error == "Timeout":
                    handle_disconnect_state(user_data)

                update_status(f"Ki error: {result.message}", [255, 100, 100])
                update_last_response(f"NACK: {result.error or 'Unknown'}")

        except queue.Empty:
            update_status("Ki change timeout", [255, 150, 50])
            update_last_response("Timeout")

    threading.Thread(target=check_ki, daemon=True).start()


def on_p_ff_changed(sender, app_data, user_data: LEDControllerApp) -> None:
    """Handle P feed-forward slider change."""
    ff = app_data  # 0-100 PWM percent
    logger.info(f"PI-FeedForward slider changed to {ff}%")

    # Update input field to match slider
    if dpg.does_item_exist(TAGS["p_ff_input"]):
        dpg.set_value(TAGS["p_ff_input"], ff)

    # Send command to device
    task = SerialTask(command=SerialCommand.SET_P_FEED_FORWARD, p_feed_forward=ff)
    result_queue = user_data.send_task(task)

    # Check result in background thread
    def check_ff():
        try:
            result = result_queue.get(timeout=2)

            if result.success:
                update_status(f"Feed-Forward set to {ff}%", [100, 200, 100])
                update_last_response(f"ACK: FF={ff}%")
            else:
                if result.error == "Disconnected" or result.error == "Timeout":
                    handle_disconnect_state(user_data)

                update_status(f"Feed-Forward error: {result.message}", [255, 100, 100])
                update_last_response(f"NACK: {result.error or 'Unknown'}")

        except queue.Empty:
            update_status("Feed-Forward change timeout", [255, 150, 50])
            update_last_response("Timeout")

    threading.Thread(target=check_ff, daemon=True).start()


def on_p_ff_input_changed(sender, app_data, user_data: LEDControllerApp) -> None:
    """Handle P feed-forward input field change."""
    ff = max(0, min(100, app_data))  # Clamp 0-100

    # Update slider
    if dpg.does_item_exist(TAGS["p_ff_slider"]):
        dpg.set_value(TAGS["p_ff_slider"], ff)

    # Send command to device
    task = SerialTask(command=SerialCommand.SET_P_FEED_FORWARD, p_feed_forward=ff)
    result_queue = user_data.send_task(task)

    # Check result in background thread
    def check_ff():
        try:
            result = result_queue.get(timeout=2)

            if result.success:
                update_status(f"Feed-Forward set to {ff}%", [100, 200, 100])
                update_last_response(f"ACK: FF={ff}%")
            else:
                if result.error == "Disconnected" or result.error == "Timeout":
                    handle_disconnect_state(user_data)

                update_status(f"Feed-Forward error: {result.message}", [255, 100, 100])
                update_last_response(f"NACK: {result.error or 'Unknown'}")

        except queue.Empty:
            update_status("FeedForward change timeout", [255, 150, 50])
            update_last_response("Timeout")

    threading.Thread(target=check_ff, daemon=True).start()


def sync_p_params_to_firmware(app: LEDControllerApp) -> None:
    """
    Sync current PI-controller parameters (gain, ki, feed_forward) from GUI to firmware.

    Called when switching to AUTO mode to ensure firmware values match GUI sliders.
    Without this, firmware would use gain=0, ki=0 and feed_forward=0 (defaults), causing
    PWM output to always be 0 regardless of error.
    """
    # Get current gain value from slider
    gain = 1.0  # default
    if dpg.exists(TAGS["p_gain_slider"]):
        gain = dpg.get_value(TAGS["p_gain_slider"])

    # Get current Ki value from slider
    ki = 0.0  # default
    if dpg.exists(TAGS["p_ki_slider"]):
        ki = dpg.get_value(TAGS["p_ki_slider"])

    # Get current feed-forward value from slider
    ff = 0  # default
    if dpg.exists(TAGS["p_ff_slider"]):
        ff = dpg.get_value(TAGS["p_ff_slider"])

    logger.info(f"Syncing PI-params to firmware: gain={gain:.2f}, ki={ki:.2f}, ff={ff}%")

    # Send gain command
    task_gain = SerialTask(command=SerialCommand.SET_P_GAIN, p_gain=gain)
    result_gain = app.send_task(task_gain)

    # Send Ki command
    task_ki = SerialTask(command=SerialCommand.SET_P_KI, p_ki=ki)
    result_ki = app.send_task(task_ki)

    # Send feed-forward command
    task_ff = SerialTask(command=SerialCommand.SET_P_FEED_FORWARD, p_feed_forward=ff)
    result_ff = app.send_task(task_ff)

    # Check all results
    for result, label, value in [(result_gain, "Gain", gain), (result_ki, "Ki", ki), (result_ff, "FF", ff)]:
        try:
            result = result.get(timeout=2)
            if result and result.success:
                logger.info(f"Synced {label}: {value}")
            else:
                logger.warning(f"Failed to sync {label}: {result.error if result else 'timeout'}")
        except queue.Empty:
            logger.warning(f"Timeout syncing {label}")


def on_p_stream_start_clicked(sender, app_data, user_data: LEDControllerApp) -> None:
    """Handle P-stream start button click."""
    logger.info("P-stream start button clicked")

    # Disable start button, enable stop button
    if dpg.does_item_exist(TAGS["p_stream_start_btn"]):
        dpg.configure_item(TAGS["p_stream_start_btn"], enabled=False)
    if dpg.does_item_exist(TAGS["p_stream_stop_btn"]):
        dpg.configure_item(TAGS["p_stream_stop_btn"], enabled=True)

    # Get interval from input
    interval_ms = 100  # default
    if dpg.does_item_exist(TAGS["p_stream_interval"]):
        interval_ms = dpg.get_value(TAGS["p_stream_interval"])

    update_status(f"Starting P-stream ({interval_ms}ms)...", [200, 200, 100])

    # Send command to device
    task = SerialTask(command=SerialCommand.START_P_STREAM, p_stream_interval=interval_ms)
    result_queue = user_data.send_task(task)

    # Check result in background thread
    def check_start():
        try:
            result = result_queue.get(timeout=2)

            if result.success:
                user_data.set_p_streaming(True)
                update_status("P-streaming active", [100, 200, 100])
                update_last_response(f"ACK: P-stream started ({interval_ms}ms)")

                # Enable recording buttons
                if dpg.does_item_exist(TAGS["p_record_start_btn"]):
                    dpg.does_item_exist(TAGS["p_record_start_btn"], enabled=True)
                if dpg.does_item_exist(TAGS["p_record_stop_btn"]):
                    dpg.does_item_exist(TAGS["p_record_stop_btn"], enabled=True)

            else:
                if result.error == "Disconnected" or result.error == "Timeout":
                    handle_disconnect_state(user_data)

                update_status(f"P-stream start error: {result.message}", [255, 100, 100])
                update_last_response(f"NACK: {result.error or 'Unknown'}")

                # Re-enable start button, disable stop button
                if dpg.does_item_exist(TAGS["p_stream_start_btn"]):
                    dpg.configure_item(TAGS["p_stream_start_btn"], enabled=True)
                if dpg.does_item_exist(TAGS["p_stream_stop_btn"]):
                    dpg.configure_item(TAGS["p_stream_stop_btn"], enabled=False)

                # Update checkbox state
                if dpg.does_item_exist(TAGS["p_stream_enable_chk"]):
                    dpg.set_value(TAGS["p_stream_enable_chk"], False)

        except queue.Empty:
            update_status("P-stream start timeout", [255, 150, 50])
            update_last_response("Timeout")

            # Re-enable start button, disable stop button
            if dpg.does_item_exist(TAGS["p_stream_start_btn"]):
                dpg.configure_item(TAGS["p_stream_start_btn"], enabled=True)
            if dpg.does_item_exist(TAGS["p_stream_stop_btn"]):
                dpg.configure_item(TAGS["p_stream_stop_btn"], enabled=False)

            # Update checkbox state
            if dpg.does_item_exist(TAGS["p_stream_enable_chk"]):
                dpg.set_value(TAGS["p_stream_enable_chk"], False)

    threading.Thread(target=check_start, daemon=True).start()


def on_p_stream_stop_clicked(sender, app_data, user_data: LEDControllerApp) -> None:
    """Handle P-stream stop button click."""
    logger.info("P-stream stop button clicked")

    # Disable stop button
    if dpg.does_item_exist(TAGS["p_stream_stop_btn"]):
        dpg.configure_item(TAGS["p_stream_stop_btn"], enabled=False)

    update_status("Stopping P-stream...", [200, 200, 100])

    # Send command to device
    task = SerialTask(command=SerialCommand.STOP_P_STREAM)
    result_queue = user_data.send_task(task)

    # Check result in background thread
    def check_stop():
        try:
            result = result_queue.get(timeout=2)

            if result.success:
                user_data.set_p_streaming(False)
                update_status("P-stream stopped", [100, 200, 100])
                update_last_response("ACK: P-stream stopped")

                # Disable recording buttons
                if dpg.does_item_exist(TAGS["p_record_start_btn"]):
                    dpg.configure_item(TAGS["p_record_start_btn"], enabled=False)
                if dpg.does_item_exist(TAGS["p_record_stop_btn"]):
                    dpg.configure_item(TAGS["p_record_stop_btn"], enabled=False)

                # Update checkbox state
                if dpg.does_item_exist(TAGS["p_stream_enable_chk"]):
                    dpg.set_value(TAGS["p_stream_enable_chk"], False)

                # Update status
                if dpg.does_item_exist(TAGS["p_stream_status"]):
                    dpg.set_value(TAGS["p_stream_status"], "PI-Stream: Stopped")
                    dpg.configure_item(TAGS["p_stream_status"], color=[150, 150, 150])

            else:
                if result.error == "Disconnected" or result.error == "Timeout":
                    handle_disconnect_state(user_data)

                update_status(f"P-stream stop error: {result.message}", [255, 100, 100])
                update_last_response(f"NACK: {result.error or 'Unknown'}")

                # Re-enable stop button
                if dpg.does_item_exist(TAGS["p_stream_stop_btn"]):
                    dpg.configure_item(TAGS["p_stream_stop_btn"], enabled=True)

        except queue.Empty:
            update_status("P-stream stop timeout", [255, 150, 50])
            update_last_response("Timeout")

            # Re-enable stop button
            if dpg.does_item_exist(TAGS["p_stream_stop_btn"]):
                dpg.configure_item(TAGS["p_stream_stop_btn"], enabled=True)

    threading.Thread(target=check_stop, daemon=True).start()


def on_p_record_start_clicked(sender, app_data, user_data: LEDControllerApp) -> None:
    """Handle P-stream recording start button click."""
    logger.info("P-record start button clicked")

    # Disable start button, enable stop button
    if dpg.does_item_exist(TAGS["p_record_start_btn"]):
        dpg.does_item_exist(TAGS["p_record_start_btn"], enabled=False)
    if dpg.does_item_exist(TAGS["p_record_stop_btn"]):
        dpg.configure_item(TAGS["p_record_stop_btn"], enabled=True)

    update_status("Starting P-stream recording...", [200, 200, 100])

    # Send command to device
    task = SerialTask(command=SerialCommand.START_P_RECORD)
    result_queue = user_data.send_task(task)

    # Check result in background thread
    def check_record():
        try:
            result = result_queue.get(timeout=2)

            if result.success:
                user_data.set_p_recording(True)
                update_status("Recording P-stream data...", [100, 200, 100])
                update_last_response("ACK: Recording started")

                # Update status
                if dpg.does_item_exist(TAGS["p_record_status"]):
                    dpg.set_value(TAGS["p_record_status"], "Recording...")
                    dpg.configure_item(TAGS["p_record_status"], color=[100, 255, 100])

                # Update checkbox state
                if dpg.does_item_exist(TAGS["p_record_enable_chk"]):
                    dpg.set_value(TAGS["p_record_enable_chk"], True)

            else:
                if result.error == "Disconnected" or result.error == "Timeout":
                    handle_disconnect_state(user_data)

                update_status(f"Record start error: {result.message}", [255, 100, 100])
                update_last_response(f"NACK: {result.error or 'Unknown'}")

                # Re-enable start button, disable stop button
                if dpg.does_item_exist(TAGS["p_record_start_btn"]):
                    dpg.configure_item(TAGS["p_record_start_btn"], enabled=True)
                if dpg.does_item_exist(TAGS["p_record_stop_btn"]):
                    dpg.configure_item(TAGS["p_record_stop_btn"], enabled=False)

        except queue.Empty:
            update_status("Record start timeout", [255, 150, 50])
            update_last_response("Timeout")

            # Re-enable start button, disable stop button
            if dpg.does_item_exist(TAGS["p_record_start_btn"]):
                dpg.configure_item(TAGS["p_record_start_btn"], enabled=True)
            if dpg.does_item_exist(TAGS["p_record_stop_btn"]):
                dpg.configure_item(TAGS["p_record_stop_btn"], enabled=False)

    threading.Thread(target=check_record, daemon=True).start()


def on_p_record_stop_clicked(sender, app_data, user_data: LEDControllerApp) -> None:
    """Handle P-stream recording stop button click."""
    logger.info("P-record stop button clicked")

    # Disable stop button
    if dpg.does_item_exist(TAGS["p_record_stop_btn"]):
        dpg.configure_item(TAGS["p_record_stop_btn"], enabled=False)

    update_status("Stopping P-stream recording...", [200, 200, 100])

    # Send command to device
    task = SerialTask(command=SerialCommand.STOP_P_RECORD)
    result_queue = user_data.send_task(task)

    # Check result in background thread
    def check_record():
        try:
            result = result_queue.get(timeout=2)

            if result.success:
                user_data.set_p_recording(False)
                update_status("Recording stopped", [100, 200, 100])
                update_last_response("ACK: Recording stopped")

                # Flush recorded data to file
                count = user_data.flush_recorded_data()
                if count > 0:
                    update_status(f"Recording stopped, {count} samples saved", [100, 200, 100])
                    logger.info(f"Flushed {count} samples to recording file")

                # Update status
                if dpg.does_item_exist(TAGS["p_record_status"]):
                    dpg.set_value(TAGS["p_record_status"], "Stopped")
                    dpg.configure_item(TAGS["p_record_status"], color=[150, 150, 150])

                # Update checkbox state
                if dpg.does_item_exist(TAGS["p_record_enable_chk"]):
                    dpg.set_value(TAGS["p_record_enable_chk"], False)

            else:
                if result.error == "Disconnected" or result.error == "Timeout":
                    handle_disconnect_state(user_data)

                update_status(f"Record stop error: {result.message}", [255, 100, 100])
                update_last_response(f"NACK: {result.error or 'Unknown'}")

                # Re-enable stop button
                if dpg.does_item_exist(TAGS["p_record_stop_btn"]):
                    dpg.configure_item(TAGS["p_record_stop_btn"], enabled=True)

        except queue.Empty:
            update_status("Record stop timeout", [255, 150, 50])
            update_last_response("Timeout")

            # Re-enable stop button
            if dpg.does_item_exist(TAGS["p_record_stop_btn"]):
                dpg.configure_item(TAGS["p_record_stop_btn"], enabled=True)

    threading.Thread(target=check_record, daemon=True).start()


def on_led_slider_changed(sender, app_data, user_data: LEDControllerApp) -> None:
    """Callback for LED slider (0=OFF, 1=ON)."""
    state = bool(app_data)  # 0 = OFF, 1 = ON
    logger.info(f"LED slider changed to {'ON' if state else 'OFF'}")

    update_status(f"LED {'ON' if state else 'OFF'}...", [200, 200, 100])

    # Send command to device
    task = SerialTask(command=SerialCommand.SEND_LED, led_state=state)
    result_queue = user_data.send_task(task)

    # Check result in background thread
    def check_led():
        try:
            result = result_queue.get(timeout=2)
            if result.success:
                user_data.set_led_state(state)
                update_status(f"LED {'ON' if state else 'OFF'}", [100, 200, 100])
                update_last_response(f"ACK: LED={'ON' if state else 'OFF'}")
                update_led_indicator(state)
            else:
                if result.error == "Disconnected" or result.error == "Timeout":
                    handle_disconnect_state(user_data)
                update_status(f"LED error: {result.message}", [255, 100, 100])
                update_last_response(f"NACK: {result.error or 'Unknown'}")
                # Revert checkbox
                if dpg.does_item_exist(TAGS["led_btn"]):
                    dpg.set_value(TAGS["led_btn"], False)
        except queue.Empty:
            update_status("LED timeout", [255, 150, 50])
            update_last_response("Timeout")
            if dpg.does_item_exist(TAGS["led_btn"]):
                dpg.set_value(TAGS["led_btn"], False)

    threading.Thread(target=check_led, daemon=True).start()


def on_p_stream_enable_changed(sender, app_data, user_data: LEDControllerApp) -> None:
    """Callback for PI-Stream enable checkbox."""
    enabled = bool(app_data)
    logger.info(f"PI-Stream enable changed to {enabled}")

    if enabled:
        # Start streaming
        on_p_stream_start_clicked(sender, app_data, user_data)
    else:
        # Stop streaming
        on_p_stream_stop_clicked(sender, app_data, user_data)


def on_p_record_enable_changed(sender, app_data, user_data: LEDControllerApp) -> None:
    """Callback for Record enable checkbox."""
    enabled = bool(app_data)
    logger.info(f"Record enable changed to {enabled}")

    if enabled:
        # Start recording
        on_p_record_start_clicked(sender, app_data, user_data)
    else:
        # Stop recording
        on_p_record_stop_clicked(sender, app_data, user_data)


def on_adc_stream_enable_changed(sender, app_data, user_data: LEDControllerApp) -> None:
    """Callback for ADC Stream enable checkbox."""
    enabled = bool(app_data)
    logger.info(f"ADC Stream enable changed to {enabled}")

    if enabled:
        # Start ADC streaming
        on_stream_start_clicked(sender, app_data, user_data)
    else:
        # Stop ADC streaming
        on_stream_stop_clicked(sender, app_data, user_data)
