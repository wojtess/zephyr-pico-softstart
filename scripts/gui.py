#!/usr/bin/env python3
"""
RP2040 LED Control - Dear PyGui Application

Step 1: Basic window setup with improved structure and error handling.
"""

import sys

try:
    import dearpygui.dearpygui as dpg
except ImportError as e:
    print("ERROR: Dear PyGui is not installed.")
    print("Install it with: ./venv/bin/pip install dearpygui")
    sys.exit(1)


# Tag constants for better resource management
TAGS = {
    "primary_window": "win_primary",
}


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
            dpg.add_text("GUI controls coming soon...")

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
