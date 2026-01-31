"""
Utility functions for RP2040 LED Control GUI.
"""

import glob
import logging
from typing import List, Dict, Optional

import dearpygui.dearpygui as dpg

from .constants import TAGS
from .app import LEDControllerApp

logger = logging.getLogger(__name__)


def find_ports_with_info() -> List[Dict[str, str]]:
    """Auto-detect available serial ports with device information."""
    ports_info = []

    # Linux
    for port in glob.glob("/dev/ttyACM*"):
        ports_info.append({"device": port, "description": "ACM Device"})
    for port in glob.glob("/dev/ttyUSB*"):
        ports_info.append({"device": port, "description": "USB Serial"})

    # macOS
    for port in glob.glob("/dev/cu.usbmodem*"):
        ports_info.append({"device": port, "description": "USB Modem"})
    for port in glob.glob("/dev/cu.usbserial*"):
        ports_info.append({"device": port, "description": "USB Serial"})

    # Windows
    try:
        import serial.tools.list_ports
        for port in serial.tools.list_ports.comports():
            if "ACM" in port.description or "USB" in port.description:
                ports_info.append({
                    "device": port.device,
                    "description": port.description or "USB Device"
                })
    except (ImportError, OSError):
        pass

    return ports_info


def update_status(message: str, color: List[int]) -> None:
    """Update the status message with text and color."""
    if dpg.does_item_exist(TAGS["status_message"]):
        dpg.set_value(TAGS["status_message"], message)
        dpg.configure_item(TAGS["status_message"], color=color)


def update_connection_indicator(connected: bool) -> None:
    """Update the connection status indicator."""
    if dpg.does_item_exist(TAGS["conn_indicator"]):
        if connected:
            dpg.configure_item(TAGS["conn_indicator"], color=[100, 255, 100])  # Green
            dpg.set_value(TAGS["conn_indicator"], "Connected")
        else:
            dpg.configure_item(TAGS["conn_indicator"], color=[150, 150, 150])  # Gray
            dpg.set_value(TAGS["conn_indicator"], "Disconnected")


def update_led_indicator(on: bool) -> None:
    """Update the LED status indicator."""
    if dpg.does_item_exist(TAGS["led_indicator"]):
        if on:
            dpg.configure_item(TAGS["led_indicator"], color=[100, 255, 100])  # Green
            dpg.set_value(TAGS["led_indicator"], "LED ON")
        else:
            dpg.configure_item(TAGS["led_indicator"], color=[150, 150, 150])  # Gray
            dpg.set_value(TAGS["led_indicator"], "LED OFF")


def update_last_response(message: str) -> None:
    """Update the last response display."""
    if dpg.does_item_exist(TAGS["last_response"]):
        dpg.set_value(TAGS["last_response"], f"Last: {message}")


def get_selected_port() -> Optional[str]:
    """Get the currently selected port device path."""
    if not dpg.does_item_exist(TAGS["port_combo"]):
        return None

    try:
        selected = dpg.get_value(TAGS["port_combo"])
        if selected and "No device" not in selected:
            parts = selected.split(" - ")
            if len(parts) == 2:
                return parts[1]
    except Exception:
        logger.exception("Error getting selected port")

    return None


def handle_disconnect_state(app: LEDControllerApp) -> None:
    """Handle unexpected disconnect - reset UI state."""
    if dpg.does_item_exist(TAGS["connect_btn"]):
        dpg.set_item_label(TAGS["connect_btn"], "Connect")

    if dpg.does_item_exist(TAGS["port_combo"]):
        dpg.configure_item(TAGS["port_combo"], enabled=True)

    if dpg.does_item_exist(TAGS["refresh_btn"]):
        dpg.configure_item(TAGS["refresh_btn"], enabled=True)

    if dpg.does_item_exist(TAGS["led_btn"]):
        dpg.configure_item(TAGS["led_btn"], enabled=False)

    if dpg.does_item_exist(TAGS["adc_read_btn"]):
        dpg.configure_item(TAGS["adc_read_btn"], enabled=False)

    update_connection_indicator(False)
    update_led_indicator(False)


def update_queue_display(app: LEDControllerApp) -> None:
    """Update task queue display in GUI."""
    pending_count = app.get_pending_count()
    processing_task = app.get_processing_task()

    if dpg.does_item_exist(TAGS["queue_count"]):
        dpg.set_value(TAGS["queue_count"], str(pending_count))

    if dpg.does_item_exist(TAGS["queue_processing"]):
        if processing_task:
            dpg.set_value(TAGS["queue_processing"], processing_task)
            dpg.configure_item(TAGS["queue_processing"], color=[100, 255, 100])
        else:
            dpg.set_value(TAGS["queue_processing"], "None")
            dpg.configure_item(TAGS["queue_processing"], color=[150, 150, 150])
