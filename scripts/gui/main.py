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
logging.basicConfig(level=logging.DEBUG, format='%(asctime)s - %(levelname)s - %(message)s')
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
from gui.utils import find_ports_with_info, update_status


def main() -> int:
    """Main application entry point."""
    # Initialize app
    app = LEDControllerApp()

    # Initialize Dear PyGui context
    dpg.create_context()

    try:
        # Configure viewport (larger for controls + plots)
        dpg.create_viewport(
            title="RP2040 LED Control",
            width=1400,
            height=900,
            clear_color=[30, 30, 30, 255],
            decorated=True,
        )

        # Create primary window with all widgets
        create_main_window(app)

        # Setup and show viewport
        dpg.setup_dearpygui()
        dpg.show_viewport()

        # Start plot refresh thread (updates plot when streaming)
        import threading
        def plot_refresh_worker():
            import time
            while dpg.is_dearpygui_running():
                try:
                    # Update ADC History plot (when ADC streaming is active)
                    if app.is_streaming():
                        x_axis, raw_series, volt_series = app.get_adc_history()

                        # Update ADC history plot
                        if dpg.does_item_exist(TAGS["adc_history_plot"]):
                            if dpg.does_item_exist(TAGS["adc_series_raw"]):
                                dpg.set_value(TAGS["adc_series_raw"], [x_axis, raw_series])
                            if dpg.does_item_exist(TAGS["adc_series_voltage"]):
                                dpg.set_value(TAGS["adc_series_voltage"], [x_axis, volt_series])

                        # Update value displays with latest data
                        if raw_series:
                            latest_raw = raw_series[-1]
                            latest_voltage = volt_series[-1]
                            if dpg.does_item_exist(TAGS["adc_value_raw"]):
                                dpg.set_value(TAGS["adc_value_raw"], f"{latest_raw}")
                            if dpg.does_item_exist(TAGS["adc_value_voltage"]):
                                dpg.set_value(TAGS["adc_value_voltage"], f"{latest_voltage:.3f} V")

                    # Update P-Stream plot (when P-streaming is active)
                    if app.is_p_streaming():
                        x_axis, raw_series, volt_series = app.get_p_stream_history()
                        setpoint_series = list(app._setpoint_history)
                        pwm_series = list(app._pwm_history)
                        error_series = list(app._error_history)

                        # Update P-stream plot
                        if dpg.does_item_exist(TAGS["p_stream_plot"]):
                            if dpg.does_item_exist(TAGS["p_series_setpoint"]):
                                dpg.set_value(TAGS["p_series_setpoint"], [x_axis, setpoint_series])
                            if dpg.does_item_exist(TAGS["p_series_measured"]):
                                dpg.set_value(TAGS["p_series_measured"], [x_axis, raw_series])
                            if dpg.does_item_exist(TAGS["p_series_pwm"]):
                                dpg.set_value(TAGS["p_series_pwm"], [x_axis, pwm_series])
                            if dpg.does_item_exist(TAGS["p_series_error"]):
                                dpg.set_value(TAGS["p_series_error"], [x_axis, error_series])

                        # Flush recording data periodically (when recording is active)
                        if app.is_p_recording():
                            count = app.flush_recorded_data()
                            if count > 0:
                                logger.debug(f"Flushed {count} samples to recording")

                    # Update PWM output display ALWAYS (independent of streaming state)
                    # This shows the last known PWM value from any source (manual or P-stream)
                    last_pwm = app.get_last_known_pwm()
                    if dpg.does_item_exist(TAGS["p_pwm_output"]):
                        dpg.set_value(TAGS["p_pwm_output"], f"{last_pwm}%")

                    time.sleep(0.05)  # 20 FPS for plot updates
                except Exception:
                    break

        threading.Thread(target=plot_refresh_worker, daemon=True).start()

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
