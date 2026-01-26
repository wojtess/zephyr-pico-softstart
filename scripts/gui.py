#!/usr/bin/env python3
"""
RP2040 LED Control - Dear PyGui Application

Step 2: Serial port detection and dropdown selection.

Features:
- Auto-detect serial ports (Linux/macOS/Windows)
- Dropdown port selection
- Refresh button to rescan ports
"""

import sys
import glob
from typing import List, Optional

try:
    import dearpygui.dearpygui as dpg
except ImportError as e:
    print("ERROR: Dear PyGui is not installed.")
    print("Install it with: ./venv/bin/pip install dearpygui")
    sys.exit(1)


# Tag constants for better resource management
TAGS = {
    "primary_window": "win_primary",
    "port_combo": "combo_port",
    "refresh_btn": "btn_refresh",
}


def find_ports() -> List[str]:
    """Auto-detect available serial ports.

    Returns:
        List of port paths (e.g., ["/dev/ttyACM0", "/dev/ttyACM1"])
    """
    ports = []

    # Linux
    ports.extend(glob.glob("/dev/ttyACM*"))
    ports.extend(glob.glob("/dev/ttyUSB*"))

    # macOS
    ports.extend(glob.glob("/dev/cu.usbmodem*"))
    ports.extend(glob.glob("/dev/cu.usbserial*"))

    # Windows (try if available)
    try:
        import serial.tools.list_ports
        for port in serial.tools.list_ports.comports():
            if "ACM" in port.description or "USB" in port.description:
                ports.append(port.device)
    except ImportError:
        pass

    return sorted(set(ports))  # Remove duplicates and sort


def refresh_port_list() -> List[str]:
    """Refresh the list of available serial ports.

    Returns:
        List of port paths
    """
    ports = find_ports()

    if ports:
        print(f"Found {len(ports)} port(s): {ports}")
    else:
        print("No serial ports found")

    return ports


def update_port_dropdown():
    """Update the port dropdown with current port list."""
    ports = refresh_port_list()

    # Get current selection
    try:
        current_selection = dpg.get_value(TAGS["port_combo"])
    except:
        current_selection = None

    # Configure combo with new list
    if ports:
        dpg.configure_item(TAGS["port_combo"], items=ports)
        # Restore selection if still valid, otherwise select first
        if current_selection in ports:
            dpg.set_value(TAGS["port_combo"], current_selection)
        elif ports:
            dpg.set_value(TAGS["port_combo"], ports[0])
    else:
        dpg.configure_item(TAGS["port_combo"], items=["No ports found"])


def on_refresh_clicked(sender, app_data, user_data):
    """Callback for refresh button click."""
    update_port_dropdown()


def main() -> int:
    """Main application with proper lifecycle management.

    Returns:
        Exit code (0 for success, non-zero for errors)
    """
    # Initialize Dear PyGui context
    dpg.create_context()

    try:
        # Configure viewport
        dpg.create_viewport(
            title="RP2040 LED Control",
            width=400,
            height=300,
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

            # Serial port selection
            dpg.add_text("Serial Port:")
            initial_ports = find_ports()
            port_list = initial_ports if initial_ports else ["No ports found"]
            dpg.add_combo(
                items=port_list,
                tag=TAGS["port_combo"],
                default_value=port_list[0] if port_list else "",
                width=300,
            )

            # Refresh button
            dpg.add_spacer(height=5)
            dpg.add_button(
                label="Refresh Ports",
                tag=TAGS["refresh_btn"],
                callback=on_refresh_clicked,
                width=150,
            )

            dpg.add_spacer(height=20)
            dpg.add_text("Connection controls coming soon...")

        # Set as primary window (fills viewport)
        dpg.set_primary_window(TAGS["primary_window"], True)

        # Setup and show viewport
        dpg.setup_dearpygui()
        dpg.show_viewport()

        # Main loop
        dpg.start_dearpygui()

    except Exception as e:
        print(f"ERROR: {e}")
        import traceback
        traceback.print_exc()
        return 1

    finally:
        # Always cleanup context
        dpg.destroy_context()

    return 0


if __name__ == "__main__":
    sys.exit(main())
