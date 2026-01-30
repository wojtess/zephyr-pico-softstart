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
- Thread-safe serial operations
- Connection health monitoring
"""

import sys
import os
from pathlib import Path

# Add parent directory to path for imports when running directly
sys.path.insert(0, str(Path(__file__).parent.parent))

import logging

# Configure logging
logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(levelname)s - %(message)s')
logger = logging.getLogger(__name__)

try:
    import dearpygui.dearpygui as dpg
except ImportError as e:
    print("ERROR: Dear PyGui is not installed.")
    print("Install it with: ./venv/bin/pip install dearpygui")
    sys.exit(1)

# Use absolute imports to work both as module and direct script
from gui.app import LEDControllerApp
from gui.constants import TAGS
from gui.widgets import create_main_window
from gui.callbacks import on_frame_callback
from gui.utils import find_ports_with_info, update_status


def main() -> int:
    """Main application entry point."""
    # Initialize app
    app = LEDControllerApp()

    # Initialize Dear PyGui context
    dpg.create_context()

    try:
        # Configure viewport
        dpg.create_viewport(
            title="RP2040 LED Control",
            width=550,
            height=600,
            clear_color=[30, 30, 30, 255],
            decorated=True,
        )

        # Create primary window with all widgets
        create_main_window(app)

        # Setup and show viewport
        dpg.setup_dearpygui()
        dpg.show_viewport()

        # Register frame callback for result checking (runs every frame)
        dpg.set_frame_callback(-1, on_frame_callback, user_data=app)

        # Initial port scan
        ports = find_ports_with_info()
        if ports:
            update_status(f"Found {len(ports)} device(s). Ready to connect.", [100, 200, 100])
        else:
            update_status("No device found. Connect RP2040 via USB.", [255, 150, 50])

        # Main loop
        dpg.start_dearpygui()

    except Exception as e:
        logger.error(f"Application error: {e}", exc_info=True)
        return 1

    finally:
        # Shutdown
        app.shutdown()
        dpg.destroy_context()

    return 0


if __name__ == "__main__":
    sys.exit(main())
