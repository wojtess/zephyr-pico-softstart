"""
Callback functions for RP2040 LED Control GUI.
"""

import queue
import threading
import logging
from typing import Optional

import serial
import dearpygui.dearpygui as dpg

from .constants import TAGS, SerialCommand, SerialTask
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
                update_status(f"Found {len(ports)} devices. Select a port to connect.", [150, 180, 150])
                dpg.set_value(TAGS["port_combo"], display_list[0])
        else:
            dpg.configure_item(TAGS["port_combo"], items=["No device detected"], enabled=False)
            update_status(
                "No RP2040 device found. Connect device via USB and press Refresh.",
                [255, 150, 50]
            )

    # Re-enable button
    if dpg.does_item_exist(sender):
        dpg.configure_item(sender, enabled=True)


def on_port_selected(sender, app_data, user_data: LEDControllerApp) -> None:
    """Callback when user selects a port from dropdown."""
    selected_port = get_selected_port()
    if selected_port and not user_data.is_connected():
        update_status(f"Selected: {selected_port}. Press Connect.", [100, 180, 200])


def on_connect_clicked(sender, app_data, user_data: LEDControllerApp) -> None:
    """Callback for connect/disconnect button."""
    if serial is None:
        update_status("pyserial not installed!", [255, 100, 100])
        return

    if user_data.is_connected():
        # Disconnect
        update_status("Disconnecting...", [200, 200, 100])
        dpg.split_frame()

        task = SerialTask(command=SerialCommand.DISCONNECT)
        result_queue = user_data.send_task(task)

        # Check result after short delay
        def check_disconnect():
            try:
                result = result_queue.get(timeout=2)
                if result.success:
                    handle_disconnect_state(user_data)
                    update_status("Disconnected", [150, 180, 150])
                else:
                    update_status(f"Disconnect error: {result.message}", [255, 100, 100])
            except queue.Empty:
                update_status("Disconnect timeout", [255, 150, 50])

        threading.Thread(target=check_disconnect, daemon=True).start()

    else:
        # Connect
        port = get_selected_port()
        if not port:
            update_status("No port selected!", [255, 100, 100])
            return

        # Disable button
        if dpg.does_item_exist(sender):
            dpg.configure_item(sender, enabled=False)

        update_status(f"Connecting to {port}...", [200, 200, 100])
        dpg.split_frame()

        task = SerialTask(command=SerialCommand.CONNECT, port=port)
        result_queue = user_data.send_task(task)

        # Check result in separate thread
        def check_connect():
            try:
                result = result_queue.get(timeout=5)

                # Re-enable button
                if dpg.does_item_exist(sender):
                    dpg.configure_item(sender, enabled=True)

                if result.success:
                    # Update UI
                    if dpg.does_item_exist(TAGS["connect_btn"]):
                        dpg.set_item_label(TAGS["connect_btn"], "Disconnect")

                    if dpg.does_item_exist(TAGS["port_combo"]):
                        dpg.configure_item(TAGS["port_combo"], enabled=False)

                    if dpg.does_item_exist(TAGS["refresh_btn"]):
                        dpg.configure_item(TAGS["refresh_btn"], enabled=False)

                    if dpg.does_item_exist(TAGS["led_btn"]):
                        dpg.configure_item(TAGS["led_btn"], enabled=True)

                    if dpg.does_item_exist(TAGS["adc_read_btn"]):
                        dpg.configure_item(TAGS["adc_read_btn"], enabled=True)

                    update_connection_indicator(True)
                    update_status(result.message, [100, 200, 100])

                    # Start health monitoring after a delay (2 seconds to let connection stabilize)
                    import time
                    def schedule_first_health_check():
                        time.sleep(2.0)
                        user_data._last_health_check = time.time()
                        user_data.send_task(SerialTask(command=SerialCommand.CHECK_HEALTH))
                        logger.info("First health check scheduled")
                    threading.Thread(target=schedule_first_health_check, daemon=True).start()

                else:
                    update_status(f"Connection failed: {result.message}", [255, 100, 100])

            except queue.Empty:
                if dpg.does_item_exist(sender):
                    dpg.configure_item(sender, enabled=True)
                update_status("Connection timeout", [255, 150, 50])

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
                # Update button label
                label = "Turn LED OFF" if result.led_state else "Turn LED ON"
                if dpg.does_item_exist(sender):
                    dpg.set_item_label(sender, label)

                update_led_indicator(result.led_state)
                update_status(result.message, [100, 200, 100])
                update_last_response("ACK (0xFF)")

            else:
                # Check if disconnected
                if result.error == "Disconnected" or result.error == "Timeout":
                    handle_disconnect_state(user_data)

                update_status(f"Error: {result.message}", [255, 100, 100])
                update_last_response(f"NACK/Error: {result.error or 'Unknown'}")

        except queue.Empty:
            if dpg.does_item_exist(sender):
                dpg.configure_item(sender, enabled=True)
            update_status("Command timeout", [255, 150, 50])
            update_last_response("Timeout")

    threading.Thread(target=check_led, daemon=True).start()


def on_pwm_slider_changed(sender, app_data, user_data: LEDControllerApp) -> None:
    """Callback for PWM slider value changes."""
    duty = app_data  # app_data contains the new slider value
    logger.info(f"PWM slider changed to {duty}%")

    # Update label
    if dpg.does_item_exist(TAGS["pwm_label"]):
        dpg.set_value(TAGS["pwm_label"], f"{duty}%")

    # Send PWM command
    task = SerialTask(command=SerialCommand.SEND_PWM, pwm_duty=duty)
    result_queue = user_data.send_task(task)

    # Check result in background thread
    def check_pwm():
        try:
            result = result_queue.get(timeout=2)

            if result.success:
                update_status(f"PWM set to {duty}%", [100, 200, 100])
                update_last_response("ACK (0xFF)")
            else:
                # Check if disconnected
                if result.error == "Disconnected" or result.error == "Timeout":
                    handle_disconnect_state(user_data)

                update_status(f"PWM Error: {result.message}", [255, 100, 100])
                update_last_response(f"NACK: {result.error or 'Unknown'}")
                # Revert slider to last known good value
                if dpg.does_item_exist(TAGS["pwm_slider"]):
                    dpg.set_value(TAGS["pwm_slider"], user_data.get_pwm_duty())
                    if dpg.does_item_exist(TAGS["pwm_label"]):
                        dpg.set_value(TAGS["pwm_label"], f"{user_data.get_pwm_duty()}%")

        except queue.Empty:
            update_status("PWM command timeout", [255, 150, 50])
            update_last_response("Timeout")
            # Revert slider to last known good value
            if dpg.does_item_exist(TAGS["pwm_slider"]):
                dpg.set_value(TAGS["pwm_slider"], user_data.get_pwm_duty())
                if dpg.does_item_exist(TAGS["pwm_label"]):
                    dpg.set_value(TAGS["pwm_label"], f"{user_data.get_pwm_duty()}%")

    threading.Thread(target=check_pwm, daemon=True).start()


def on_adc_read_clicked(sender, app_data, user_data: LEDControllerApp) -> None:
    """Callback for ADC single read button."""
    # Disable button
    if dpg.does_item_exist(sender):
        dpg.configure_item(sender, enabled=False)

    update_status("Reading ADC...", [200, 200, 100])
    dpg.split_frame()

    task = SerialTask(command=SerialCommand.READ_ADC)
    result_queue = user_data.send_task(task)

    # Check result in separate thread
    def check_adc():
        try:
            result = result_queue.get(timeout=2)

            # Re-enable button
            if dpg.does_item_exist(sender):
                dpg.configure_item(sender, enabled=True)

            if result.success:
                update_status(result.message, [100, 200, 100])
                update_last_response(f"ADC: {result.adc_value} ({result.adc_voltage:.3f}V)")

                # Update value displays
                if dpg.does_item_exist(TAGS["adc_value_raw"]):
                    dpg.set_value(TAGS["adc_value_raw"], f"{result.adc_value}")
                if dpg.does_item_exist(TAGS["adc_value_voltage"]):
                    dpg.set_value(TAGS["adc_value_voltage"], f"{result.adc_voltage:.3f} V")

                # Add data to app history and update plot
                user_data.add_adc_data(result.adc_value, result.adc_voltage)

                # Update plot if exists
                if dpg.does_item_exist(TAGS["adc_plot"]):
                    x_axis, raw_series, volt_series = user_data.get_adc_history()

                    # Update plot series with set_value
                    if dpg.does_item_exist(TAGS["adc_series_raw"]):
                        dpg.set_value(TAGS["adc_series_raw"], [x_axis, raw_series])
                    if dpg.does_item_exist(TAGS["adc_series_voltage"]):
                        dpg.set_value(TAGS["adc_series_voltage"], [x_axis, volt_series])

            else:
                # Check if disconnected
                if result.error == "Disconnected" or result.error == "Timeout":
                    handle_disconnect_state(user_data)

                update_status(f"ADC Error: {result.message}", [255, 100, 100])
                update_last_response(f"ADC Error: {result.error or 'Unknown'}")

        except queue.Empty:
            if dpg.does_item_exist(sender):
                dpg.configure_item(sender, enabled=True)
            update_status("ADC read timeout", [255, 150, 50])
            update_last_response("Timeout")

    threading.Thread(target=check_adc, daemon=True).start()


def on_frame_callback(sender, app_data, user_data: LEDControllerApp) -> None:
    """Called every frame to check for health check results."""
    # Note: PWM slider now uses direct callback instead of polling

    # Check results from worker thread
    results = user_data.check_results()

    for result in results:
        if result.command == SerialCommand.CHECK_HEALTH:
            if not result.success:
                # Connection lost
                handle_disconnect_state(user_data)
                update_status(f"Connection lost: {result.message}", [255, 150, 50])

                # Stop health checks
                continue

            # Schedule next health check with rate limiting (max 1 per second)
            import time
            current_time = time.time()
            if current_time - user_data._last_health_check >= user_data._health_check_interval:
                user_data._last_health_check = current_time
                user_data.send_task(SerialTask(command=SerialCommand.CHECK_HEALTH))

    # Update queue display
    update_queue_display(user_data)
