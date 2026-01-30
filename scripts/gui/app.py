"""
LED Controller Application - Main application class with serial worker thread.
"""

import sys
import os
from pathlib import Path

# Add parent directory to path for protocol module import
sys.path.insert(0, str(Path(__file__).parent.parent))

import time
import queue
import threading
import logging
from typing import List, Optional

try:
    import serial
except ImportError:
    serial = None

from .constants import SerialCommand, SerialTask, SerialResult
from protocol import (
    CMD_SET_LED,
    CMD_SET_PWM,
    RESP_ACK,
    RESP_NACK,
    build_frame,
    parse_response,
    get_error_name,
)

logger = logging.getLogger(__name__)


class LEDControllerApp:
    """Main LED Controller application with Dear PyGui GUI."""

    def __init__(self):
        """Initialize the application."""
        # Thread-safe state
        self._lock = threading.Lock()
        self._serial_connection: Optional[serial.Serial] = None
        self._led_state: bool = False
        self._pwm_duty: int = 0  # 0-100%
        self._is_connected: bool = False

        # Worker thread
        self._task_queue: queue.Queue = queue.Queue()
        self._result_queue: queue.Queue = queue.Queue()
        self._worker_thread: Optional[threading.Thread] = None
        self._running: bool = False

        # Health check rate limiting (max 1 check per second)
        self._last_health_check: float = 0.0
        self._health_check_interval: float = 1.0  # seconds

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

                elif task.command == SerialCommand.SEND_PWM:
                    result = self._handle_send_pwm(task.pwm_duty)
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
        logger.info(f"SEND_LED: state={state}")
        try:
            with self._lock:
                if not self._serial_connection or not self._serial_connection.is_open:
                    logger.warning("SEND_LED: Not connected - serial_connection is None or not open")
                    return SerialResult(
                        SerialCommand.SEND_LED,
                        False,
                        "Not connected",
                        error="No connection"
                    )

                # Build and send frame
                value = 1 if state else 0
                frame = build_frame(CMD_SET_LED, value)
                logger.debug(f"SEND_LED: sending frame: {frame.hex()}")

                self._serial_connection.write(frame)
                self._serial_connection.flush()

                # Read response: 1 byte first (ACK or NACK), then read error code if NACK
                resp_type_byte = self._serial_connection.read(1)
                if not resp_type_byte:
                    logger.error("SEND_LED: No response from device")
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

                resp_type = resp_type_byte[0]
                err_code = 0

                # If NACK, read error code byte
                if resp_type == RESP_NACK:
                    err_byte = self._serial_connection.read(1)
                    if err_byte:
                        err_code = err_byte[0]

                logger.debug(f"SEND_LED: received response: type=0x{resp_type:02X}, err=0x{err_code:02X}")

                if resp_type == RESP_ACK:
                    self._led_state = state  # State update inside lock
                    logger.info(f"SEND_LED: ACK - LED {'ON' if state else 'OFF'}")
                    return SerialResult(
                        SerialCommand.SEND_LED,
                        True,
                        f"LED turned {'ON' if state else 'OFF'}",
                        led_state=state
                    )
                else:  # RESP_NACK
                    error_name = get_error_name(err_code)
                    logger.warning(f"SEND_LED: NACK - {error_name} (err_code={err_code})")
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

    def _handle_send_pwm(self, duty: int) -> SerialResult:
        """Handle PWM duty cycle command in worker thread."""
        logger.info(f"SEND_PWM: duty={duty}%")
        try:
            with self._lock:
                if not self._serial_connection or not self._serial_connection.is_open:
                    logger.warning("SEND_PWM: Not connected - serial_connection is None or not open")
                    return SerialResult(
                        SerialCommand.SEND_PWM,
                        False,
                        "Not connected",
                        error="No connection"
                    )

                # Validate duty cycle range
                if duty < 0 or duty > 100:
                    logger.warning(f"SEND_PWM: Invalid duty cycle {duty}")
                    return SerialResult(
                        SerialCommand.SEND_PWM,
                        False,
                        "Invalid duty cycle (must be 0-100)",
                        error="Invalid value"
                    )

                # Build and send frame
                frame = build_frame(CMD_SET_PWM, duty)
                logger.debug(f"SEND_PWM: sending frame: {frame.hex()}")

                self._serial_connection.write(frame)
                self._serial_connection.flush()

                # Read response: 1 byte first (ACK or NACK), then read error code if NACK
                resp_type_byte = self._serial_connection.read(1)
                if not resp_type_byte:
                    logger.error("SEND_PWM: No response from device")
                    self._is_connected = False
                    self._serial_connection = None
                    self._led_state = False
                    self._pwm_duty = 0
                    return SerialResult(
                        SerialCommand.SEND_PWM,
                        False,
                        "Device not responding (timeout)",
                        error="Timeout",
                        led_state=None
                    )

                resp_type = resp_type_byte[0]
                err_code = 0

                # If NACK, read error code byte
                if resp_type == RESP_NACK:
                    err_byte = self._serial_connection.read(1)
                    if err_byte:
                        err_code = err_byte[0]

                logger.debug(f"SEND_PWM: received response: type=0x{resp_type:02X}, err=0x{err_code:02X}")

                if resp_type == RESP_ACK:
                    # Update state atomically inside lock
                    self._pwm_duty = duty
                    self._led_state = duty > 0
                    logger.info(f"SEND_PWM: ACK - PWM set to {duty}%")
                    return SerialResult(
                        SerialCommand.SEND_PWM,
                        True,
                        f"PWM set to {duty}%",
                        led_state=duty > 0
                    )
                else:  # RESP_NACK
                    error_name = get_error_name(err_code)
                    logger.warning(f"SEND_PWM: NACK - {error_name} (err_code={err_code})")
                    return SerialResult(
                        SerialCommand.SEND_PWM,
                        False,
                        f"PWM failed: {error_name}",
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
                    self._pwm_duty = 0

                logger.warning("Device disconnected during PWM command")
                return SerialResult(
                    SerialCommand.SEND_PWM,
                    False,
                    "Device disconnected",
                    error="Disconnected"
                )

            logger.error(f"Serial error: {e}")
            return SerialResult(
                SerialCommand.SEND_PWM,
                False,
                "Communication error",
                error=str(e)
            )

        except Exception as e:
            logger.error(f"Unexpected error sending PWM command: {e}", exc_info=True)
            return SerialResult(
                SerialCommand.SEND_PWM,
                False,
                "Unexpected error",
                error=str(e)
            )

    def _handle_health_check(self) -> SerialResult:
        """Check if connection is still alive.

        Note: USB CDC ACM devices don't reliably implement CD (Carrier Detect).
        We use is_open check instead, which is sufficient for USB devices.
        """
        try:
            with self._lock:
                logger.debug("Health check: verifying connection state")

                if not self._serial_connection:
                    logger.debug("Health check: serial_connection is None")
                    return SerialResult(SerialCommand.CHECK_HEALTH, False,
                                       "Not connected", error="No connection")

                if not self._serial_connection.is_open:
                    logger.warning(f"Health check: serial port {self._serial_connection.port} is not open")
                    if self._is_connected:
                        logger.warning("Connection lost detected during health check")
                        self._is_connected = False
                        self._led_state = False
                    self._serial_connection = None
                    return SerialResult(
                        SerialCommand.CHECK_HEALTH,
                        False,
                        "Connection lost",
                        error="Port closed"
                    )

                # Connection is healthy
                logger.debug(f"Health check: connection OK on {self._serial_connection.port}")
                return SerialResult(SerialCommand.CHECK_HEALTH, True, "OK")

        except Exception as e:
            logger.error(f"Health check exception: {e}", exc_info=True)
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

    def get_pwm_duty(self) -> int:
        """Get current PWM duty cycle."""
        with self._lock:
            return self._pwm_duty

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
        except Exception:
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
