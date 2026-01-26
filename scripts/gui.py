#!/usr/bin/env python3
"""
RP2040 LED Control - Dear PyGui Application

Step 2: Serial port detection with improved UX and error handling.

Features:
- Auto-detect serial ports (Linux/macOS/Windows)
- Dropdown port selection with device descriptions
- Refresh button with visual feedback
- Status message area
- Auto-select if only one port found
"""

import sys
import glob
from typing import List, Optional, Dict, Any

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
    "status_message": "txt_status",
}


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
        update_status(f"Selected: {selected_port}. Ready to connect.", [100, 180, 200])


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
            width=500,
            height=350,
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

            # Refresh button row
            dpg.add_spacer(height=10)
            with dpg.group(horizontal=True):
                dpg.add_button(
                    label="Refresh Ports",
                    tag=TAGS["refresh_btn"],
                    callback=on_refresh_clicked,
                    width=-1,
                )

            # Status message area
            dpg.add_spacer(height=15)
            dpg.add_text(
                tag=TAGS["status_message"],
                default_value="Select a serial port to connect",
                wrap=450,
            )

            dpg.add_spacer(height=15)
            dpg.add_text("Connection controls coming soon...", color=[150, 150, 150])

        # Set as primary window (fills viewport)
        dpg.set_primary_window(TAGS["primary_window"], True)

        # Setup and show viewport
        dpg.setup_dearpygui()
        dpg.show_viewport()

        # Initial port scan and update
        update_port_dropdown()

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
