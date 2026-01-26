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
import glob
import time
import threading
import queue
import logging
from typing import List, Optional, Dict
from dataclasses import dataclass
from enum import Enum

# Configure logging
logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(levelname)s - %(message)s')
logger = logging.getLogger(__name__)

try:
    import serial
except ImportError:
    print("WARNING: pyserial not installed. Install with: pip install pyserial")
    serial = None
    logger.error("pyserial not available")

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


# ==========================================================================
# CONSTANTS AND TYPES
# ==========================================================================

class SerialCommand(Enum):
    """Serial command types for worker thread."""
    CONNECT = "connect"
    DISCONNECT = "disconnect"
    SEND_LED = "send_led"
    CHECK_HEALTH = "check_health"
    QUIT = "quit"


@dataclass
class SerialTask:
    """Task for serial worker thread."""
    command: SerialCommand
    port: Optional[str] = None
    led_state: Optional[bool] = None
    result_queue: Optional[queue.Queue] = None


@dataclass
class SerialResult:
    """Result from serial worker thread."""
    command: SerialCommand
    success: bool
    message: str
    led_state: Optional[bool] = None
    error: Optional[str] = None


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


# ==========================================================================
# MAIN APPLICATION CLASS
# ==========================================================================

class LEDControllerApp:
    """Main LED Controller application with Dear PyGui GUI."""

    def __init__(self):
        """Initialize the application."""
        # Thread-safe state
        self._lock = threading.Lock()
        self._serial_connection: Optional[serial.Serial] = None
        self._led_state: bool = False
        self._is_connected: bool = False

        # Worker thread
        self._task_queue: queue.Queue = queue.Queue()
        self._result_queue: queue.Queue = queue.Queue()
        self._worker_thread: Optional[threading.Thread] = None
        self._running: bool = False

        # Start worker thread
        self._start_worker()

    def _start_worker(self) -> None:
        """Start the serial worker thread."""
        self._running = True
        self._worker_thread = threading.Thread(target=self._serial_worker, daemon=True)
        self._worker_thread.start()
        logger.info("Serial worker thread started")

    def _serial_worker(self) -> None:
        """Background thread for all serial operations."""
        logger.info("Serial worker running")

        while self._running:
            try:
                task = self._task_queue.get(timeout=0.1)

                if task.command == SerialCommand.QUIT:
                    logger.info("Worker received QUIT command")
                    break

                elif task.command == SerialCommand.CONNECT:
                    result = self._handle_connect(task.port)
                    if task.result_queue:
                        task.result_queue.put(result)

                elif task.command == SerialCommand.DISCONNECT:
                    result = self._handle_disconnect()
                    if task.result_queue:
                        task.result_queue.put(result)

                elif task.command == SerialCommand.SEND_LED:
                    result = self._handle_send_led(task.led_state)
                    if task.result_queue:
                        task.result_queue.put(result)

                elif task.command == SerialCommand.CHECK_HEALTH:
                    result = self._handle_health_check()
                    # Health check doesn't need result callback

                self._task_queue.task_done()

            except queue.Empty:
                continue
            except Exception as e:
                logger.error(f"Worker error: {e}", exc_info=True)

        logger.info("Serial worker stopped")

    def _handle_connect(self, port: str) -> SerialResult:
        """Handle connection in worker thread."""
        try:
            with self._lock:
                if self._serial_connection and self._serial_connection.is_open:
                    return SerialResult(
                        SerialCommand.CONNECT,
                        False,
                        "Already connected",
                        error="Already connected"
                    )

            logger.info(f"Connecting to {port}...")
            connection = serial.Serial(
                port=port,
                baudrate=115200,
                timeout=1,
                write_timeout=1
            )

            # Flush any initial data
            connection.reset_input_buffer()
            connection.reset_output_buffer()

            # Wait for device to be ready
            time.sleep(0.3)

            with self._lock:
                self._serial_connection = connection
                self._is_connected = True

            logger.info(f"Connected to {port}")
            return SerialResult(
                SerialCommand.CONNECT,
                True,
                f"Connected to {port}"
            )

        except serial.SerialException as e:
            error_msg = str(e)
            # Translate to user-friendly message
            if "Permission denied" in error_msg or "could not open port" in error_msg:
                user_msg = "Port busy. Close other programs (IDE, terminal) using this device."
            elif "does not exist" in error_msg or "could not open port" in error_msg:
                user_msg = "Device not found. Check USB connection."
            elif "Device or resource busy" in error_msg:
                user_msg = "Device is busy. Wait a moment and try again."
            else:
                user_msg = "Connection failed. Try disconnecting and reconnecting."

            logger.error(f"Connection error: {e}")
            return SerialResult(
                SerialCommand.CONNECT,
                False,
                user_msg,
                error=error_msg
            )

        except Exception as e:
            logger.error(f"Unexpected connection error: {e}", exc_info=True)
            return SerialResult(
                SerialCommand.CONNECT,
                False,
                "Unexpected error during connection",
                error=str(e)
            )

    def _handle_disconnect(self) -> SerialResult:
        """Handle disconnection in worker thread."""
        try:
            with self._lock:
                if self._serial_connection:
                    self._serial_connection.close()
                    self._serial_connection = None
                self._is_connected = False
                self._led_state = False

            logger.info("Disconnected")
            return SerialResult(
                SerialCommand.DISCONNECT,
                True,
                "Disconnected"
            )

        except Exception as e:
            logger.error(f"Disconnect error: {e}", exc_info=True)
            return SerialResult(
                SerialCommand.DISCONNECT,
                False,
                "Disconnect error",
                error=str(e)
            )

    def _handle_send_led(self, state: bool) -> SerialResult:
        """Handle LED command in worker thread."""
        try:
            with self._lock:
                if not self._serial_connection or not self._serial_connection.is_open:
                    return SerialResult(
                        SerialCommand.SEND_LED,
                        False,
                        "Not connected",
                        error="No connection"
                    )

                # Build and send frame
                value = 1 if state else 0
                frame = build_frame(CMD_SET_LED, value)

                self._serial_connection.write(frame)
                self._serial_connection.flush()

                # Read response (read up to 2 bytes for ACK or NACK+error)
                resp = self._serial_connection.read(2)

                if not resp:
                    self._is_connected = False
                    self._serial_connection = None
                    self._led_state = False
                    return SerialResult(
                        SerialCommand.SEND_LED,
                        False,
                        "Device not responding (timeout)",
                        error="Timeout",
                        led_state=None
                    )

                resp_type, err_code = parse_response(resp)

                if resp_type == RESP_ACK:
                    self._led_state = state
                    return SerialResult(
                        SerialCommand.SEND_LED,
                        True,
                        f"LED turned {'ON' if state else 'OFF'}",
                        led_state=state
                    )
                else:  # RESP_NACK
                    error_name = get_error_name(err_code)
                    logger.warning(f"NACK received: {error_name}")
                    return SerialResult(
                        SerialCommand.SEND_LED,
                        False,
                        f"Command failed: {error_name}",
                        error=error_name
                    )

        except serial.SerialException as e:
            error_msg = str(e)
            # Check for disconnection
            if "device disconnected" in error_msg.lower() or not self._serial_connection.is_open:
                with self._lock:
                    self._is_connected = False
                    self._serial_connection = None
                    self._led_state = False

                logger.warning("Device disconnected during command")
                return SerialResult(
                    SerialCommand.SEND_LED,
                    False,
                    "Device disconnected",
                    error="Disconnected"
                )

            logger.error(f"Serial error: {e}")
            return SerialResult(
                SerialCommand.SEND_LED,
                False,
                "Communication error",
                error=str(e)
            )

        except Exception as e:
            logger.error(f"Unexpected error sending LED command: {e}", exc_info=True)
            return SerialResult(
                SerialCommand.SEND_LED,
                False,
                "Unexpected error",
                error=str(e)
            )

    def _handle_health_check(self) -> SerialResult:
        """Check if connection is still alive."""
        try:
            with self._lock:
                if not self._serial_connection or not self._serial_connection.is_open:
                    if self._is_connected:
                        # Connection was lost
                        logger.warning("Connection lost detected during health check")
                        self._is_connected = False
                        self._serial_connection = None
                        self._led_state = False
                        return SerialResult(
                            SerialCommand.CHECK_HEALTH,
                            False,
                            "Connection lost",
                            error="Disconnected"
                        )
                    return SerialResult(SerialCommand.CHECK_HEALTH, True, "OK")

            # Try to read DTR line to check connection
            try:
                dtr = self._serial_connection.getCD()  # Check Carrier Detect
                if not dtr:
                    logger.warning("Carrier Detect lost - device disconnected")
                    self._is_connected = False
                    self._serial_connection.close()
                    self._serial_connection = None
                    self._led_state = False
                    return SerialResult(
                        SerialCommand.CHECK_HEALTH,
                        False,
                        "Connection lost",
                        error="No carrier"
                    )
            except:
                pass  # Not all ports support CD

            return SerialResult(SerialCommand.CHECK_HEALTH, True, "OK")

        except Exception as e:
            logger.error(f"Health check error: {e}")
            return SerialResult(
                SerialCommand.CHECK_HEALTH,
                False,
                "Health check failed",
                error=str(e)
            )

    def is_connected(self) -> bool:
        """Check if currently connected."""
        with self._lock:
            return self._is_connected and self._serial_connection is not None

    def get_led_state(self) -> bool:
        """Get current LED state."""
        with self._lock:
            return self._led_state

    def send_task(self, task: SerialTask) -> queue.Queue:
        """Send task to worker thread and return result queue."""
        result_queue = queue.Queue()
        task.result_queue = result_queue
        self._task_queue.put(task)
        return result_queue

    def check_results(self) -> List[SerialResult]:
        """Check for completed operations from worker thread."""
        results = []
        try:
            while True:
                result = self._result_queue.get_nowait()
                results.append(result)
        except queue.Empty:
            pass
        return results

    def shutdown(self) -> None:
        """Shutdown the application and worker thread."""
        logger.info("Shutting down...")
        self._running = False

        # Send quit command to worker
        try:
            self._task_queue.put(SerialTask(command=SerialCommand.QUIT), timeout=1)
        except:
            pass

        # Wait for worker to finish
        if self._worker_thread and self._worker_thread.is_alive():
            self._worker_thread.join(timeout=2)

        # Close serial connection
        with self._lock:
            if self._serial_connection and self._serial_connection.is_open:
                self._serial_connection.close()
            self._serial_connection = None
            self._is_connected = False

        logger.info("Shutdown complete")


# ==========================================================================
# HELPER FUNCTIONS
# ==========================================================================

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

    update_connection_indicator(False)
    update_led_indicator(False)


# ==========================================================================
# CALLBACKS
# ==========================================================================

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
    if selected_port and not app.is_connected():
        update_status(f"Selected: {selected_port}. Press Connect.", [100, 180, 200])


def on_connect_clicked(sender, app_data, user_data: LEDControllerApp) -> None:
    """Callback for connect/disconnect button."""
    if serial is None:
        update_status("pyserial not installed!", [255, 100, 100])
        return

    if app.is_connected():
        # Disconnect
        update_status("Disconnecting...", [200, 200, 100])
        dpg.split_frame()

        task = SerialTask(command=SerialCommand.DISCONNECT)
        result_queue = app.send_task(task)

        # Check result after short delay
        def check_disconnect():
            try:
                result = result_queue.get(timeout=2)
                if result.success:
                    handle_disconnect_state(app)
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
        result_queue = app.send_task(task)

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

                    update_connection_indicator(True)
                    update_status(result.message, [100, 200, 100])

                    # Start health monitoring
                    app.send_task(SerialTask(command=SerialCommand.CHECK_HEALTH))

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

    new_state = not app.get_led_state()

    update_status(f"Turning LED {'ON' if new_state else 'OFF'}...", [200, 200, 100])
    dpg.split_frame()

    task = SerialTask(command=SerialCommand.SEND_LED, led_state=new_state)
    result_queue = app.send_task(task)

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
                    handle_disconnect_state(app)

                update_status(f"Error: {result.message}", [255, 100, 100])
                update_last_response(f"NACK/Error: {result.error or 'Unknown'}")

        except queue.Empty:
            if dpg.does_item_exist(sender):
                dpg.configure_item(sender, enabled=True)
            update_status("Command timeout", [255, 150, 50])
            update_last_response("Timeout")

    threading.Thread(target=check_led, daemon=True).start()


def on_frame_callback(sender, app_data, user_data: LEDControllerApp) -> None:
    """Called every frame to check for completed operations."""
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

            # Schedule next health check
            user_data.send_task(SerialTask(command=SerialCommand.CHECK_HEALTH))


# ==========================================================================
# MAIN
# ==========================================================================

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
            height=500,
            clear_color=[30, 30, 30, 255],
            decorated=True,
        )

        # Create primary window
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
            dpg.add_spacer(height=5)

            # Get initial ports
            initial_ports = find_ports_with_info()
            if initial_ports:
                display_list = [f"{p['description']} - {p['device']}" for p in initial_ports]
            else:
                display_list = ["No device detected"]

            dpg.add_combo(
                items=display_list,
                tag=TAGS["port_combo"],
                default_value=display_list[0] if display_list else "",
                width=-1,
                callback=lambda s, d, a=app: on_port_selected(s, d, a),
            )

            # Connection buttons
            dpg.add_spacer(height=10)
            with dpg.group(horizontal=True):
                dpg.add_button(
                    label="Refresh",
                    tag=TAGS["refresh_btn"],
                    callback=lambda s, d, a=app: on_refresh_clicked(s, d, a),
                    width=120,
                )

                dpg.add_spacer(width=10)

                dpg.add_button(
                    label="Connect",
                    tag=TAGS["connect_btn"],
                    callback=lambda s, d, a=app: on_connect_clicked(s, d, a),
                    width=-1,
                )

            # Status indicators
            dpg.add_spacer(height=15)
            dpg.add_separator()
            dpg.add_spacer(height=10)

            with dpg.group(horizontal=True):
                dpg.add_text("Connection:", color=[200, 200, 200])
                dpg.add_spacer(width=10)
                dpg.add_text(tag=TAGS["conn_indicator"], default_value="Disconnected", color=[150, 150, 150])

            dpg.add_spacer(height=8)

            with dpg.group(horizontal=True):
                dpg.add_text("LED Status:", color=[200, 200, 200])
                dpg.add_spacer(width=10)
                dpg.add_text(tag=TAGS["led_indicator"], default_value="LED OFF", color=[150, 150, 150])

            # LED control
            dpg.add_spacer(height=15)
            dpg.add_separator()
            dpg.add_spacer(height=10)

            dpg.add_button(
                label="Turn LED ON",
                tag=TAGS["led_btn"],
                callback=lambda s, d, a=app: on_led_toggle_clicked(s, d, a),
                width=-1,
                enabled=False,
            )

            # Status messages
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
                default_value="Last: None",
                color=[180, 180, 180],
            )

        # Set as primary window
        dpg.set_primary_window(TAGS["primary_window"], True)

        # Setup and show viewport
        dpg.setup_dearpygui()
        dpg.show_viewport()

        # Register frame callback for result checking
        dpg.set_frame_callback(0, lambda s, d, a=app: on_frame_callback(s, d, a))

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
