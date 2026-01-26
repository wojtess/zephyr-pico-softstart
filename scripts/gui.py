#!/usr/bin/env python3
"""
RP2040 LED Control - Dear PyGui Application

Complete application with serial connection, protocol implementation,
LED toggle button, and visual status indicators.

Features:
- Auto-detect serial ports (Linux/macOS/Windows)
- Dropdown port selection with device descriptions
- Connect/Disconnect button with state management
- LED ON/OFF toggle with protocol communication
- Visual status indicators (connection, LED state)
- Status message area with color coding
"""

import sys
import glob
import time
from typing import List, Optional, Dict, Any

try:
    import serial
except ImportError:
    print("WARNING: pyserial not installed. Install with: pip install pyserial")
    serial = None

try:
    import dearpygui.dearpygui as dpg
except ImportError as e:
    print("ERROR: Dear PyGui is not installed.")
    print("Install it with: ./venv/bin/pip install dearpygui")
    sys.exit(1)

# Import protocol module
from protocol import (
    CMD_SET_LED,
    RESP_ACK,
    RESP_NACK,
    build_frame,
    parse_response,
    get_error_name,
)


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
    "last_response": "txt_response",
}

# Global state
serial_connection: Optional[serial.Serial] = None
led_state: bool = False  # Track LED state locally


def find_ports_with_info() -> List[Dict[str, str]]:
    """Auto-detect available serial ports with device information.

    Returns:
        List of dicts with 'device' and 'description' keys
    """
    ports_info = []

    # Linux - get basic info
    for port in glob.glob("/dev/ttyACM*"):
        ports_info.append({"device": port, "description": "ACM Device"})
    for port in glob.glob("/dev/ttyUSB*"):
        ports_info.append({"device": port, "description": "USB Serial"})

    # macOS
    for port in glob.glob("/dev/cu.usbmodem*"):
        ports_info.append({"device": port, "description": "USB Modem"})
    for port in glob.glob("/dev/cu.usbserial*"):
        ports_info.append({"device": port, "description": "USB Serial"})

    # Windows - try to get more detailed info
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


def refresh_port_list() -> List[Dict[str, str]]:
    """Refresh the list of available serial ports.

    Returns:
        List of dicts with 'device' and 'description' keys
    """
    ports = find_ports_with_info()

    if ports:
        print(f"Found {len(ports)} port(s): {[p['device'] for p in ports]}")
    else:
        print("No serial ports found")

    return ports


def update_port_dropdown() -> bool:
    """Update the port dropdown with current port list.

    Returns:
        True if ports were found, False otherwise
    """
    # Verify widget exists before operations
    if not dpg.does_item_exist(TAGS["port_combo"]):
        print(f"WARNING: Port combo widget {TAGS['port_combo']} does not exist")
        return False

    if not dpg.does_item_exist(TAGS["status_message"]):
        print(f"WARNING: Status message widget {TAGS['status_message']} does not exist")
        return False

    ports = refresh_port_list()

    if ports:
        # Create display list with descriptions
        display_list = [f"{p['description']} - {p['device']}" for p in ports]
        dpg.configure_item(TAGS["port_combo"], items=display_list, enabled=True)

        # Auto-select if only one port
        if len(ports) == 1:
            dpg.set_value(TAGS["port_combo"], display_list[0])
            update_status(f"✓ Found 1 device: {ports[0]['device']}", [100, 200, 100])
        else:
            update_status(f"✓ Found {len(ports)} devices. Select a port to connect.", [150, 180, 150])
            dpg.set_value(TAGS["port_combo"], display_list[0])

        return True
    else:
        # No ports found - disable combo
        dpg.configure_item(TAGS["port_combo"], items=["No device detected"], enabled=False)
        update_status(
            "⚠ No RP2040 device found. Please:\n"
            "   • Connect your RP2040 via USB\n"
            "   • Press the 'Refresh' button\n"
            "   • Check USB cable connection",
            [255, 150, 50]
        )
        return False


def update_status(message: str, color: List[int]) -> None:
    """Update the status message with text and color.

    Args:
        message: Status message text
        color: RGB color list [r, g, b]
    """
    if dpg.does_item_exist(TAGS["status_message"]):
        dpg.set_value(TAGS["status_message"], message)
        dpg.configure_item(TAGS["status_message"], color=color)


def update_connection_indicator(connected: bool) -> None:
    """Update the connection status indicator.

    Args:
        connected: True if connected, False otherwise
    """
    if dpg.does_item_exist(TAGS["conn_indicator"]):
        if connected:
            dpg.configure_item(TAGS["conn_indicator"], color=[100, 255, 100])  # Green
            dpg.set_value(TAGS["conn_indicator"], "● Connected")
        else:
            dpg.configure_item(TAGS["conn_indicator"], color=[150, 150, 150])  # Gray
            dpg.set_value(TAGS["conn_indicator"], "○ Disconnected")


def update_led_indicator(on: bool) -> None:
    """Update the LED status indicator.

    Args:
        on: True if LED is ON, False if OFF
    """
    if dpg.does_item_exist(TAGS["led_indicator"]):
        if on:
            dpg.configure_item(TAGS["led_indicator"], color=[100, 255, 100])  # Green
            dpg.set_value(TAGS["led_indicator"], "● LED ON")
        else:
            dpg.configure_item(TAGS["led_indicator"], color=[150, 150, 150])  # Gray
            dpg.set_value(TAGS["led_indicator"], "○ LED OFF")


def update_last_response(message: str) -> None:
    """Update the last response display.

    Args:
        message: Response message to display
    """
    if dpg.does_item_exist(TAGS["last_response"]):
        dpg.set_value(TAGS["last_response"], f"Last response: {message}")


def get_selected_port() -> Optional[str]:
    """Get the currently selected port device path.

    Returns:
        Port device path (e.g., "/dev/ttyACM0") or None
    """
    if not dpg.does_item_exist(TAGS["port_combo"]):
        return None

    try:
        selected = dpg.get_value(TAGS["port_combo"])
        if selected and "No device" not in selected:
            # Extract device path from "Description - /dev/path"
            parts = selected.split(" - ")
            if len(parts) == 2:
                return parts[1]
    except Exception:
        pass

    return None


def is_connected() -> bool:
    """Check if currently connected to a serial port.

    Returns:
        True if connected, False otherwise
    """
    global serial_connection
    return serial_connection is not None and serial_connection.is_open


def send_led_command(state: bool) -> bool:
    """Send LED control command to device.

    Args:
        state: True to turn LED ON, False to turn OFF

    Returns:
        True if command succeeded (ACK received), False otherwise
    """
    global serial_connection

    if not is_connected():
        update_status("❌ Not connected!", [255, 100, 100])
        return False

    value = 1 if state else 0
    frame = build_frame(CMD_SET_LED, value)

    try:
        # Send command
        serial_connection.write(frame)
        serial_connection.flush()

        # Read response
        resp = serial_connection.read(1)

        if not resp:
            update_status("❌ No response from device", [255, 100, 100])
            update_last_response("Timeout")
            return False

        resp_type, err_code = parse_response(resp)

        if resp_type == RESP_ACK:
            update_status(f"✓ LED turned {'ON' if state else 'OFF'}", [100, 200, 100])
            update_last_response("ACK (0xFF)")
            return True
        else:  # RESP_NACK
            error_name = get_error_name(err_code)
            update_status(f"❌ Command failed: {error_name}", [255, 100, 100])
            update_last_response(f"NACK (0x{resp_type:02X}): {error_name}")
            return False

    except serial.SerialException as e:
        update_status(f"❌ Serial error: {e}", [255, 100, 100])
        update_last_response(f"Error: {e}")
        return False

    except Exception as e:
        update_status(f"❌ Unexpected error: {e}", [255, 100, 100])
        update_last_response(f"Error: {e}")
        return False


def on_refresh_clicked(sender, app_data, user_data) -> None:
    """Callback for refresh button click with visual feedback.

    Args:
        sender: Button widget ID
        app_data: Button data (usually None for buttons)
        user_data: Optional user data passed from button
    """
    # Disable button and show feedback
    if dpg.does_item_exist(sender):
        dpg.configure_item(sender, enabled=False)

    update_status("🔄 Scanning for devices...", [200, 200, 100])

    # Force UI update
    dpg.split_frame()

    # Do refresh
    ports_found = update_port_dropdown()

    # Re-enable button and show result
    if dpg.does_item_exist(sender):
        dpg.configure_item(sender, enabled=True)

    if ports_found:
        update_status("✓ Refresh complete", [100, 200, 100])


def on_port_selected(sender, app_data, user_data) -> None:
    """Callback when user selects a port from dropdown.

    Args:
        sender: Combo widget ID
        app_data: Selected value (display string)
        user_data: Optional user data
    """
    selected_port = get_selected_port()
    if selected_port:
        print(f"Port selected: {selected_port}")
        if not is_connected():
            update_status(f"Selected: {selected_port}. Press Connect.", [100, 180, 200])


def on_connect_clicked(sender, app_data, user_data) -> None:
    """Callback for connect/disconnect button.

    Args:
        sender: Button widget ID
        app_data: Button data (usually None for buttons)
        user_data: Optional user data passed from button
    """
    global serial_connection

    if serial is None:
        update_status("❌ pyserial not installed!", [255, 100, 100])
        return

    if is_connected():
        # Disconnect
        try:
            serial_connection.close()
            serial_connection = None

            # Update UI
            if dpg.does_item_exist(TAGS["connect_btn"]):
                dpg.set_item_label(TAGS["connect_btn"], "Connect")

            if dpg.does_item_exist(TAGS["port_combo"]):
                dpg.configure_item(TAGS["port_combo"], enabled=True)

            if dpg.does_item_exist(TAGS["refresh_btn"]):
                dpg.configure_item(TAGS["refresh_btn"], enabled=True)

            if dpg.does_item_exist(TAGS["led_btn"]):
                dpg.configure_item(TAGS["led_btn"], enabled=False)

            update_connection_indicator(False)
            update_status("✓ Disconnected", [150, 180, 150])
            print("Disconnected from serial port")

        except Exception as e:
            update_status(f"❌ Disconnect error: {e}", [255, 100, 100])
            print(f"Disconnect error: {e}")
    else:
        # Connect
        port = get_selected_port()
        if not port:
            update_status("❌ No port selected!", [255, 100, 100])
            return

        # Disable button and show feedback
        if dpg.does_item_exist(sender):
            dpg.configure_item(sender, enabled=False)

        update_status(f"🔄 Connecting to {port}...", [200, 200, 100])
        dpg.split_frame()

        try:
            # Attempt connection
            serial_connection = serial.Serial(
                port=port,
                baudrate=115200,
                timeout=1,
                write_timeout=1
            )

            # Flush any initial data
            serial_connection.reset_input_buffer()
            serial_connection.reset_output_buffer()

            # Wait for boot messages
            time.sleep(0.5)

            # Update UI on success
            if dpg.does_item_exist(TAGS["connect_btn"]):
                dpg.set_item_label(TAGS["connect_btn"], "Disconnect")
                dpg.configure_item(TAGS["connect_btn"], enabled=True)

            if dpg.does_item_exist(TAGS["port_combo"]):
                dpg.configure_item(TAGS["port_combo"], enabled=False)

            if dpg.does_item_exist(TAGS["refresh_btn"]):
                dpg.configure_item(TAGS["refresh_btn"], enabled=False)

            if dpg.does_item_exist(TAGS["led_btn"]):
                dpg.configure_item(TAGS["led_btn"], enabled=True)

            update_connection_indicator(True)
            update_status(f"✓ Connected to {port}", [100, 200, 100])
            print(f"Connected to {port}")

        except serial.SerialException as e:
            serial_connection = None

            # Re-enable button
            if dpg.does_item_exist(sender):
                dpg.configure_item(sender, enabled=True)

            update_status(f"❌ Connection failed: {e}", [255, 100, 100])
            print(f"Connection error: {e}")

        except Exception as e:
            serial_connection = None

            # Re-enable button
            if dpg.does_item_exist(sender):
                dpg.configure_item(sender, enabled=True)

            update_status(f"❌ Unexpected error: {e}", [255, 100, 100])
            print(f"Unexpected error: {e}")


def on_led_toggle_clicked(sender, app_data, user_data) -> None:
    """Callback for LED toggle button.

    Args:
        sender: Button widget ID
        app_data: Button data (usually None for buttons)
        user_data: Optional user data passed from button
    """
    global led_state

    # Toggle state
    new_state = not led_state

    # Send command
    if send_led_command(new_state):
        led_state = new_state
        update_led_indicator(led_state)

        # Update button label
        if dpg.does_item_exist(TAGS["led_btn"]):
            label = "Turn LED OFF" if led_state else "Turn LED ON"
            dpg.set_item_label(TAGS["led_btn"], label)


def main() -> int:
    """Main application with proper lifecycle management.

    Returns:
        Exit code (0 for success, non-zero for errors)
    """
    global serial_connection, led_state

    # Initialize Dear PyGui context
    dpg.create_context()

    try:
        # Configure viewport
        dpg.create_viewport(
            title="RP2040 LED Control",
            width=550,
            height=500,
            clear_color=[30, 30, 30, 255],
            decorated=True,
        )

        # Create primary window (full viewport)
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

            # Serial port selection group
            dpg.add_text("Serial Port:")
            dpg.add_spacer(height=5)

            # Get initial ports
            initial_ports = find_ports_with_info()
            if initial_ports:
                display_list = [f"{p['description']} - {p['device']}" for p in initial_ports]
            else:
                display_list = ["No device detected"]

            # Port selection dropdown
            dpg.add_combo(
                items=display_list,
                tag=TAGS["port_combo"],
                default_value=display_list[0] if display_list else "",
                width=-1,
                callback=on_port_selected,
            )

            # Connection buttons row
            dpg.add_spacer(height=10)
            with dpg.group(horizontal=True):
                dpg.add_button(
                    label="Refresh",
                    tag=TAGS["refresh_btn"],
                    callback=on_refresh_clicked,
                    width=120,
                )

                dpg.add_spacer(width=10)

                dpg.add_button(
                    label="Connect",
                    tag=TAGS["connect_btn"],
                    callback=on_connect_clicked,
                    width=-1,
                )

            # Status indicators
            dpg.add_spacer(height=15)
            dpg.add_separator()
            dpg.add_spacer(height=10)

            with dpg.group(horizontal=True):
                dpg.add_text("Connection:", color=[200, 200, 200])
                dpg.add_spacer(width=10)
                dpg.add_text(tag=TAGS["conn_indicator"], default_value="○ Disconnected", color=[150, 150, 150])

            dpg.add_spacer(height=8)

            with dpg.group(horizontal=True):
                dpg.add_text("LED Status:", color=[200, 200, 200])
                dpg.add_spacer(width=10)
                dpg.add_text(tag=TAGS["led_indicator"], default_value="○ LED OFF", color=[150, 150, 150])

            # LED control button
            dpg.add_spacer(height=15)
            dpg.add_separator()
            dpg.add_spacer(height=10)

            dpg.add_button(
                label="Turn LED ON",
                tag=TAGS["led_btn"],
                callback=on_led_toggle_clicked,
                width=-1,
                enabled=False,
            )

            # Status message area
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
                default_value="Last response: None",
                color=[180, 180, 180],
            )

        # Set as primary window (fills viewport)
        dpg.set_primary_window(TAGS["primary_window"], True)

        # Setup and show viewport
        dpg.setup_dearpygui()
        dpg.show_viewport()

        # Initial port scan and update
        update_port_dropdown()

        # Main loop
        dpg.start_dearpygui()

        # Cleanup: close serial connection if open
        if serial_connection and serial_connection.is_open:
            serial_connection.close()
            print("Serial connection closed on exit")

    except Exception as e:
        print(f"ERROR: {e}")
        import traceback
        traceback.print_exc()

        # Cleanup on error
        if serial_connection and serial_connection.is_open:
            serial_connection.close()

        return 1

    finally:
        # Always cleanup context
        dpg.destroy_context()

    return 0


if __name__ == "__main__":
    sys.exit(main())
