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
from collections import deque

try:
    import serial
except ImportError:
    serial = None

from .constants import SerialCommand, SerialTask, SerialResult
from protocol import (
    CMD_SET_LED,
    CMD_SET_PWM,
    CMD_READ_ADC,
    CMD_START_STREAM,
    CMD_STOP_STREAM,
    CMD_SET_MODE,
    CMD_SET_P_SETPOINT,
    CMD_SET_P_GAIN,
    CMD_START_P_STREAM,
    CMD_STOP_P_STREAM,
    CMD_SET_FEED_FORWARD,
    RESP_ACK,
    RESP_NACK,
    RESP_DEBUG,
    build_frame,
    build_adc_frame,
    build_stream_start_frame,
    build_stream_stop_frame,
    build_set_mode_frame,
    build_set_p_setpoint_frame,
    build_set_p_gain_frame,
    build_set_feed_forward_frame,
    build_p_stream_start_frame,
    build_p_stream_stop_frame,
    parse_adc_response,
    parse_stream_data,
    parse_p_stream_data,
    get_error_name,
    crc8,
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

        # ADC data history - using deque for circular buffer (scrolling plot)
        self._adc_max_points: int = 100
        self._adc_time_history = deque(maxlen=self._adc_max_points)
        self._adc_raw_history = deque(maxlen=self._adc_max_points)
        self._adc_voltage_history = deque(maxlen=self._adc_max_points)

        # ADC Streaming state
        self._is_streaming: bool = False
        self._stream_interval: int = 100  # milliseconds

        # P-Controller state
        self._p_mode: int = 0  # 0=manual, 1=p_control
        self._p_setpoint: int = 0
        self._p_gain: float = 1.0
        self._p_feed_forward: int = 0
        self._p_streaming: bool = False

        # History for P-controller plot (deques)
        self._setpoint_history = deque(maxlen=self._adc_max_points)
        self._pwm_history = deque(maxlen=self._adc_max_points)
        self._error_history = deque(maxlen=self._adc_max_points)

        # P-Controller recording state
        self._p_recording: bool = False
        self._p_record_start_time: float = 0.0
        self._p_record_file = None
        self._p_record_data: list = []  # Buffer data, write periodically

        # Command queue (visible in GUI) - pending tasks waiting to be processed
        self._pending_tasks: List[SerialTask] = []
        self._processing_task: Optional[SerialTask] = None

        # Response queue for command responses (filled by data reader)
        # Contains raw bytes - handlers parse them
        self._response_queue: queue.Queue = queue.Queue()

        # Start worker thread
        self._start_worker()

    def _start_worker(self) -> None:
        """Start the serial worker threads."""
        self._running = True

        # Thread 1: Task processor (handles commands)
        self._worker_thread = threading.Thread(target=self._task_processor, daemon=True)
        self._worker_thread.start()

        # Thread 2: Data reader (continuously reads and parses serial data)
        self._data_reader_thread = threading.Thread(target=self._data_reader, daemon=True)
        self._data_reader_thread.start()

        logger.info("Serial worker threads started")

    def _data_reader(self) -> None:
        """Continuously read and parse data from serial port.

        This is THE ONE AND ONLY data receiver. It parses all incoming data
        and routes it appropriately:
        - Streaming data (0x04 + ADC_H + ADC_L + CRC) -> add to plot
        - P-stream data (0x09 + SP_H + SP_L + MEAS_H + MEAS_L + PWM + CRC) -> add to plot
        - ALL other bytes -> pass to response queue for handlers
        """
        logger.info("Data reader thread running")

        # Parser state for streaming data
        state = "WAIT_MARKER"
        adc_h = 0
        adc_l = 0
        # P-stream parsing state
        p_sp_h = 0
        p_sp_l = 0
        p_meas_h = 0
        p_meas_l = 0
        p_pwm = 0

        while self._running:
            # Only read when connected
            with self._lock:
                if not self._serial_connection or not self._serial_connection.is_open:
                    time.sleep(0.1)
                    continue
                conn = self._serial_connection

            try:
                # Read one byte with timeout
                byte_data = conn.read(1)
                if not byte_data:
                    continue

                byte_val = byte_data[0]
                logger.debug(f"Data reader: got byte 0x{byte_val:02X}, state={state}")

                # State machine for parsing streaming data
                if state == "WAIT_MARKER":
                    if byte_val == CMD_START_STREAM:
                        # Streaming data packet starts - consume it
                        state = "ADC_H"
                    elif byte_val == CMD_START_P_STREAM:
                        # P-stream data packet starts - consume it
                        state = "P_SP_H"
                    elif byte_val == RESP_DEBUG:
                        # Debug packet - read and log code byte
                        code_byte = conn.read(1)
                        if code_byte:
                            code = code_byte[0]
                            debug_names = {
                                1: "TIMER_START", 2: "TIMER_CB", 3: "ADC_READ",
                                4: "ADC_ERROR", 5: "TX_PUT"
                            }
                            name = debug_names.get(code, f"UNKNOWN({code})")
                            logger.debug(f"Debug packet: {name} (0x{code:02X})")
                    else:
                        # Pass all other bytes to response queue
                        self._response_queue.put(byte_val)

                elif state == "ADC_H":
                    adc_h = byte_val
                    state = "ADC_L"

                elif state == "ADC_L":
                    adc_l = byte_val
                    state = "CRC"

                elif state == "CRC":
                    # Validate CRC and add to plot
                    calc_crc = crc8(bytes([CMD_START_STREAM, adc_h, adc_l]))
                    if byte_val == calc_crc:
                        adc_raw = (adc_h << 8) | adc_l
                        voltage = (adc_raw / 4095.0) * 3.3

                        # Add to history
                        with self._lock:
                            self._adc_time_history.append(time.time())
                            self._adc_raw_history.append(adc_raw)
                            self._adc_voltage_history.append(voltage)

                            # Trim to max points
                            if len(self._adc_time_history) > self._adc_max_points:
                                self._adc_time_history.pop(0)
                                self._adc_raw_history.pop(0)
                                self._adc_voltage_history.pop(0)

                        logger.debug(f"Stream data: raw={adc_raw}, voltage={voltage:.3f}V")
                    else:
                        logger.warning(f"CRC error in stream data: expected 0x{calc_crc:02X}, got 0x{byte_val:02X}")

                    # Reset state
                    state = "WAIT_MARKER"

                # P-stream parsing states
                elif state == "P_SP_H":
                    p_sp_h = byte_val
                    state = "P_SP_L"

                elif state == "P_SP_L":
                    p_sp_l = byte_val
                    state = "P_MEAS_H"

                elif state == "P_MEAS_H":
                    p_meas_h = byte_val
                    state = "P_MEAS_L"

                elif state == "P_MEAS_L":
                    p_meas_l = byte_val
                    state = "P_PWM"

                elif state == "P_PWM":
                    p_pwm = byte_val
                    state = "P_CRC"

                elif state == "P_CRC":
                    # Validate CRC and add to plot
                    calc_crc = crc8(bytes([CMD_START_P_STREAM, p_sp_h, p_sp_l, p_meas_h, p_meas_l, p_pwm]))
                    if byte_val == calc_crc:
                        setpoint = (p_sp_h << 8) | p_sp_l
                        measured = (p_meas_h << 8) | p_meas_l
                        pwm = p_pwm
                        error = setpoint - measured  # Calculate error

                        # Add to history
                        with self._lock:
                            self._adc_time_history.append(time.time())
                            self._adc_raw_history.append(measured)
                            self._adc_voltage_history.append((measured / 4095.0) * 3.3)
                            self._setpoint_history.append(setpoint)
                            self._pwm_history.append(pwm)
                            self._error_history.append(error)

                            # Trim to max points
                            if len(self._adc_time_history) > self._adc_max_points:
                                self._adc_time_history.pop(0)
                                self._adc_raw_history.pop(0)
                                self._adc_voltage_history.pop(0)
                                self._setpoint_history.pop(0)
                                self._pwm_history.pop(0)

                            # Record data if recording is active
                            if self._p_recording:
                                timestamp = time.time() - self._p_record_start_time
                                self._p_record_data.append([timestamp, setpoint, measured, pwm, error])

                        logger.debug(f"P-stream data: SP={setpoint}, Meas={measured}, PWM={pwm}%")
                    else:
                        logger.warning(f"CRC error in P-stream data: expected 0x{calc_crc:02X}, got 0x{byte_val:02X}")

                    # Reset state
                    state = "WAIT_MARKER"

            except serial.SerialException as e:
                logger.error(f"Serial error in data reader: {e}")
                # Connection lost
                with self._lock:
                    self._is_connected = False
                    self._serial_connection = None
                    self._is_streaming = False
                    self._p_streaming = False
                state = "WAIT_MARKER"
            except Exception as e:
                logger.error(f"Error in data reader: {e}", exc_info=True)
                state = "WAIT_MARKER"

        logger.info("Data reader thread stopped")

    def _task_processor(self) -> None:
        """Process queued tasks and send commands."""
        logger.info("Task processor thread running")

        while self._running:
            try:
                task = self._task_queue.get(timeout=0.1)

                # Set as processing task and remove from pending
                with self._lock:
                    if task in self._pending_tasks:
                        self._pending_tasks.remove(task)
                    self._processing_task = task

                # Execute command
                result = None
                if task.command == SerialCommand.QUIT:
                    logger.info("Worker received QUIT command")
                    break

                elif task.command == SerialCommand.CONNECT:
                    result = self._handle_connect(task.port)

                elif task.command == SerialCommand.DISCONNECT:
                    result = self._handle_disconnect()

                elif task.command == SerialCommand.SEND_LED:
                    result = self._handle_send_led(task.led_state)

                elif task.command == SerialCommand.SEND_PWM:
                    result = self._handle_send_pwm(task.pwm_duty)

                elif task.command == SerialCommand.READ_ADC:
                    result = self._handle_read_adc()

                elif task.command == SerialCommand.START_STREAM:
                    result = self._handle_start_stream(task.stream_interval)

                elif task.command == SerialCommand.STOP_STREAM:
                    result = self._handle_stop_stream()

                elif task.command == SerialCommand.CHECK_HEALTH:
                    result = self._handle_health_check()

                elif task.command == SerialCommand.SET_P_MODE:
                    result = self._handle_set_p_mode(task.p_mode)

                elif task.command == SerialCommand.SET_P_SETPOINT:
                    result = self._handle_set_p_setpoint(task.p_setpoint)

                elif task.command == SerialCommand.SET_P_GAIN:
                    result = self._handle_set_p_gain(task.p_gain)

                elif task.command == SerialCommand.SET_P_FEED_FORWARD:
                    result = self._handle_set_p_feed_forward(task.p_feed_forward)

                elif task.command == SerialCommand.START_P_STREAM:
                    result = self._handle_start_p_stream(task.p_stream_interval)

                elif task.command == SerialCommand.STOP_P_STREAM:
                    result = self._handle_stop_p_stream()

                elif task.command == SerialCommand.START_P_RECORD:
                    result = self._handle_start_p_record()

                elif task.command == SerialCommand.STOP_P_RECORD:
                    result = self._handle_stop_p_record()

                # After handler completes, check for leftover bytes in response queue
                # This detects command collisions, double responses, or other protocol errors
                leftover_count = 0
                leftover_bytes = []
                while not self._response_queue.empty():
                    try:
                        b = self._response_queue.get_nowait()
                        leftover_count += 1
                        leftover_bytes.append(f"0x{b:02X}")
                    except queue.Empty:
                        break

                if leftover_count > 0:
                    logger.error(
                        f"Response queue has {leftover_count} leftover bytes after command: {', '.join(leftover_bytes)}"
                    )
                    # Override result with error if command succeeded
                    if result and result.success:
                        logger.warning(f"Overriding successful result due to leftover bytes in queue")
                        result = SerialResult(
                            task.command,
                            False,
                            f"Protocol error: {leftover_count} extra bytes after response",
                            error="Leftover bytes"
                        )

                # Clear processing task
                with self._lock:
                    self._processing_task = None

                # Send result to callback
                if task.result_queue and result:
                    task.result_queue.put(result)

                self._task_queue.task_done()

            except queue.Empty:
                continue
            except Exception as e:
                logger.error(f"Task processor error: {e}", exc_info=True)
                # Clear processing task on error
                with self._lock:
                    self._processing_task = None

        logger.info("Task processor thread stopped")

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

            # Wait for response with timeout
            try:
                resp_type = self._response_queue.get(timeout=2.0)
            except queue.Empty:
                logger.error("SEND_LED: No response from device (timeout)")
                with self._lock:
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

            # Read NACK error code if needed (from response queue)
            err_code = 0
            if resp_type == RESP_NACK:
                try:
                    err_code = self._response_queue.get(timeout=1.0)
                except queue.Empty:
                    logger.warning("SEND_LED: NACK received but no error code")

            logger.info(f"SEND_LED: received response: type=0x{resp_type:02X}, err=0x{err_code:02X}")

            if resp_type == RESP_ACK:
                with self._lock:
                    self._led_state = state
                logger.info(f"SEND_LED: ACK - LED {'ON' if state else 'OFF'}")
                return SerialResult(
                    SerialCommand.SEND_LED,
                    True,
                    f"LED turned {'ON' if state else 'OFF'}",
                    led_state=state
                )
            else:  # RESP_NACK
                error_name = get_error_name(err_code)
                logger.warning(f"SEND_LED: NACK - {error_name}")
                return SerialResult(
                    SerialCommand.SEND_LED,
                    False,
                    f"Command failed: {error_name}",
                    error=error_name,
                    led_state=None
                )

        except serial.SerialException as e:
            error_msg = str(e)
            # Check for disconnection
            if "device disconnected" in error_msg.lower():
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

            # Wait for response from data reader
            try:
                resp_type = self._response_queue.get(timeout=2.0)
            except queue.Empty:
                logger.error("SEND_PWM: No response from device (timeout)")
                with self._lock:
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

            # Read NACK error code if needed (from response queue)
            err_code = 0
            if resp_type == RESP_NACK:
                try:
                    err_code = self._response_queue.get(timeout=1.0)
                except queue.Empty:
                    logger.warning("SEND_PWM: NACK received but no error code")

            logger.debug(f"SEND_PWM: received response: type=0x{resp_type:02X}, err=0x{err_code:02X}")

            if resp_type == RESP_ACK:
                with self._lock:
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
                logger.warning(f"SEND_PWM: NACK - {error_name}")
                return SerialResult(
                    SerialCommand.SEND_PWM,
                    False,
                    f"PWM failed: {error_name}",
                    error=error_name,
                    led_state=None
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

    def _handle_read_adc(self) -> SerialResult:
        """Handle ADC read command in worker thread."""
        cmd = SerialCommand.READ_ADC
        logger.info("READ_ADC: reading ADC value")

        try:
            with self._lock:
                if not self._serial_connection or not self._serial_connection.is_open:
                    logger.warning("READ_ADC: Not connected")
                    return SerialResult(cmd, False, "Not connected", error="No connection")

                # Build and send ADC read frame
                frame = build_adc_frame()
                logger.info(f"READ_ADC: sending frame: {frame.hex()}")

                self._serial_connection.write(frame)
                self._serial_connection.flush()

            # Read 4 bytes from response queue: [CMD][ADC_H][ADC_L][CRC]
            resp = []
            for i in range(4):
                try:
                    byte_val = self._response_queue.get(timeout=2.0)
                    resp.append(byte_val)
                except queue.Empty:
                    logger.error(f"READ_ADC: Timeout waiting for response byte {i+1}/4")
                    with self._lock:
                        self._is_connected = False
                        self._serial_connection = None
                    return SerialResult(cmd, False, "Device not responding (timeout)", error="Timeout")

            resp_bytes = bytes(resp)
            logger.info(f"READ_ADC: received response: {resp_bytes.hex()}")

            # Parse ADC response
            adc_value, err_code = parse_adc_response(resp_bytes)

            if adc_value >= 0:
                # Convert to voltage (0-3.3V)
                voltage = (adc_value / 4095.0) * 3.3
                logger.info(f"READ_ADC: ADC raw={adc_value}, voltage={voltage:.3f}V")
                return SerialResult(
                    cmd, True, f"ADC: {adc_value} ({voltage:.3f}V)",
                    adc_value=adc_value, adc_voltage=voltage
                )
            else:
                error_name = get_error_name(err_code)
                logger.warning(f"READ_ADC: Error - {error_name}")
                return SerialResult(cmd, False, f"ADC error: {error_name}", error=error_name)

        except serial.SerialException as e:
            error_msg = str(e)
            if "device disconnected" in error_msg.lower():
                with self._lock:
                    self._is_connected = False
                    self._serial_connection = None
                    self._led_state = False
                logger.warning("Device disconnected during ADC read")
                return SerialResult(cmd, False, "Device disconnected", error="Disconnected")
            else:
                logger.error(f"Serial error: {e}")
                return SerialResult(cmd, False, "Communication error", error=str(e))

        except Exception as e:
            logger.error(f"Unexpected error reading ADC: {e}", exc_info=True)
            return SerialResult(cmd, False, "Unexpected error", error=str(e))

    def _handle_start_stream(self, interval_ms: int) -> SerialResult:
        """Handle start ADC streaming command in worker thread."""
        cmd = SerialCommand.START_STREAM
        logger.info(f"START_STREAM: interval={interval_ms}ms")

        try:
            with self._lock:
                if not self._serial_connection or not self._serial_connection.is_open:
                    logger.warning("START_STREAM: Not connected")
                    return SerialResult(cmd, False, "Not connected", error="No connection")

                if self._is_streaming:
                    logger.warning("START_STREAM: Already streaming")
                    return SerialResult(cmd, False, "Already streaming", error="Already streaming")

                # Validate interval range
                if not (10 <= interval_ms <= 60000):
                    logger.warning(f"START_STREAM: Invalid interval {interval_ms}")
                    return SerialResult(cmd, False, "Invalid interval (10-60000ms)", error="Invalid value")

                # Build and send stream start frame
                frame = build_stream_start_frame(interval_ms)
                logger.info(f"START_STREAM: sending frame: {frame.hex()}")

                self._serial_connection.write(frame)
                self._serial_connection.flush()

            # Read response from response queue: 1 byte (ACK/NACK)
            try:
                resp_type = self._response_queue.get(timeout=2.0)
            except queue.Empty:
                logger.error("START_STREAM: No response from device (timeout)")
                return SerialResult(cmd, False, "Device not responding (timeout)", error="Timeout")

            # If NACK, read error code byte
            err_code = 0
            if resp_type == RESP_NACK:
                try:
                    err_code = self._response_queue.get(timeout=1.0)
                except queue.Empty:
                    logger.warning("START_STREAM: NACK received but no error code")

            logger.info(f"START_STREAM: received response: type=0x{resp_type:02X}, err=0x{err_code:02X}")

            if resp_type == RESP_ACK:
                with self._lock:
                    self._is_streaming = True
                    self._stream_interval = interval_ms
                logger.info(f"START_STREAM: ACK - Streaming started at {interval_ms}ms interval")
                return SerialResult(cmd, True, f"Streaming started ({interval_ms}ms interval)")
            else:  # RESP_NACK
                error_name = get_error_name(err_code)
                logger.warning(f"START_STREAM: NACK - {error_name}")
                return SerialResult(cmd, False, f"Start failed: {error_name}", error=error_name)

        except serial.SerialException as e:
            error_msg = str(e)
            if "device disconnected" in error_msg.lower():
                with self._lock:
                    self._is_connected = False
                    self._serial_connection = None
                    self._led_state = False
                logger.warning("Device disconnected during stream start")
                return SerialResult(cmd, False, "Device disconnected", error="Disconnected")
            else:
                logger.error(f"Serial error: {e}")
                return SerialResult(cmd, False, "Communication error", error=str(e))

        except Exception as e:
            logger.error(f"Unexpected error starting stream: {e}", exc_info=True)
            return SerialResult(cmd, False, "Unexpected error", error=str(e))

    def _handle_stop_stream(self) -> SerialResult:
        """Handle stop ADC streaming command in worker thread."""
        cmd = SerialCommand.STOP_STREAM
        logger.info("STOP_STREAM: Stopping ADC streaming")

        try:
            with self._lock:
                if not self._serial_connection or not self._serial_connection.is_open:
                    logger.warning("STOP_STREAM: Not connected")
                    return SerialResult(cmd, False, "Not connected", error="No connection")

                if not self._is_streaming:
                    logger.warning("STOP_STREAM: Not streaming")
                    return SerialResult(cmd, False, "Not streaming", error="Not streaming")

                # Build and send stream stop frame
                frame = build_stream_stop_frame()
                logger.info(f"STOP_STREAM: sending frame: {frame.hex()}")

                self._serial_connection.write(frame)
                self._serial_connection.flush()

            # Read response from response queue: 1 byte (ACK/NACK)
            try:
                resp_type = self._response_queue.get(timeout=2.0)
            except queue.Empty:
                logger.error("STOP_STREAM: No response from device (timeout)")
                with self._lock:
                    self._is_streaming = False
                return SerialResult(cmd, False, "Device not responding (timeout)", error="Timeout")

            # If NACK, read error code byte
            err_code = 0
            if resp_type == RESP_NACK:
                try:
                    err_code = self._response_queue.get(timeout=1.0)
                except queue.Empty:
                    logger.warning("STOP_STREAM: NACK received but no error code")

            logger.info(f"STOP_STREAM: received response: type=0x{resp_type:02X}, err=0x{err_code:02X}")

            if resp_type == RESP_ACK:
                with self._lock:
                    self._is_streaming = False
                logger.info("STOP_STREAM: ACK - Streaming stopped")
                return SerialResult(cmd, True, "Streaming stopped")
            else:  # RESP_NACK
                error_name = get_error_name(err_code)
                logger.warning(f"STOP_STREAM: NACK - {error_name}")
                return SerialResult(cmd, False, f"Stop failed: {error_name}", error=error_name)

        except serial.SerialException as e:
            error_msg = str(e)
            if "device disconnected" in error_msg.lower():
                with self._lock:
                    self._is_connected = False
                    self._serial_connection = None
                    self._led_state = False
                    self._is_streaming = False
                logger.warning("Device disconnected during stream stop")
                return SerialResult(cmd, False, "Device disconnected", error="Disconnected")
            else:
                logger.error(f"Serial error: {e}")
                return SerialResult(cmd, False, "Communication error", error=str(e))

        except Exception as e:
            logger.error(f"Unexpected error stopping stream: {e}", exc_info=True)
            return SerialResult(cmd, False, "Unexpected error", error=str(e))

    def _handle_set_p_mode(self, mode: int) -> SerialResult:
        """Handle set mode command in worker thread."""
        cmd = SerialCommand.SET_P_MODE
        logger.info(f"SET_P_MODE: mode={mode}")

        try:
            with self._lock:
                if not self._serial_connection or not self._serial_connection.is_open:
                    logger.warning("SET_P_MODE: Not connected")
                    return SerialResult(cmd, False, "Not connected", error="No connection")

                # Validate mode
                if mode not in (0, 1):
                    logger.warning(f"SET_P_MODE: Invalid mode {mode}")
                    return SerialResult(cmd, False, "Invalid mode (0=manual, 1=P-control)", error="Invalid value")

                # Build and send frame
                frame = build_set_mode_frame(mode)
                logger.info(f"SET_P_MODE: sending frame: {frame.hex()}")

                self._serial_connection.write(frame)
                self._serial_connection.flush()

            # Read response from response queue: 1 byte (ACK/NACK)
            try:
                resp_type = self._response_queue.get(timeout=2.0)
            except queue.Empty:
                logger.error("SET_P_MODE: No response from device (timeout)")
                return SerialResult(cmd, False, "Device not responding (timeout)", error="Timeout")

            # If NACK, read error code byte
            err_code = 0
            if resp_type == RESP_NACK:
                try:
                    err_code = self._response_queue.get(timeout=1.0)
                except queue.Empty:
                    logger.warning("SET_P_MODE: NACK received but no error code")

            logger.info(f"SET_P_MODE: received response: type=0x{resp_type:02X}, err=0x{err_code:02X}")

            if resp_type == RESP_ACK:
                with self._lock:
                    self._p_mode = mode
                logger.info(f"SET_P_MODE: ACK - Mode set to {mode}")
                return SerialResult(cmd, True, f"Mode set to {'Manual' if mode == 0 else 'P-Control'}")
            else:  # RESP_NACK
                error_name = get_error_name(err_code)
                logger.warning(f"SET_P_MODE: NACK - {error_name}")
                return SerialResult(cmd, False, f"Set mode failed: {error_name}", error=error_name)

        except serial.SerialException as e:
            error_msg = str(e)
            if "device disconnected" in error_msg.lower():
                with self._lock:
                    self._is_connected = False
                    self._serial_connection = None
                    self._led_state = False
                logger.warning("Device disconnected during set mode")
                return SerialResult(cmd, False, "Device disconnected", error="Disconnected")
            else:
                logger.error(f"Serial error: {e}")
                return SerialResult(cmd, False, "Communication error", error=str(e))

        except Exception as e:
            logger.error(f"Unexpected error setting mode: {e}", exc_info=True)
            return SerialResult(cmd, False, "Unexpected error", error=str(e))

    def _handle_set_p_setpoint(self, setpoint: int) -> SerialResult:
        """Handle set P setpoint command in worker thread."""
        cmd = SerialCommand.SET_P_SETPOINT
        logger.info(f"SET_P_SETPOINT: setpoint={setpoint}")

        try:
            with self._lock:
                if not self._serial_connection or not self._serial_connection.is_open:
                    logger.warning("SET_P_SETPOINT: Not connected")
                    return SerialResult(cmd, False, "Not connected", error="No connection")

                # Validate setpoint range
                if not (0 <= setpoint <= 4095):
                    logger.warning(f"SET_P_SETPOINT: Invalid setpoint {setpoint}")
                    return SerialResult(cmd, False, "Invalid setpoint (0-4095)", error="Invalid value")

                # Build and send frame
                frame = build_set_p_setpoint_frame(setpoint)
                logger.info(f"SET_P_SETPOINT: sending frame: {frame.hex()}")

                self._serial_connection.write(frame)
                self._serial_connection.flush()

            # Read response from response queue: 1 byte (ACK/NACK)
            try:
                resp_type = self._response_queue.get(timeout=2.0)
            except queue.Empty:
                logger.error("SET_P_SETPOINT: No response from device (timeout)")
                return SerialResult(cmd, False, "Device not responding (timeout)", error="Timeout")

            # If NACK, read error code byte
            err_code = 0
            if resp_type == RESP_NACK:
                try:
                    err_code = self._response_queue.get(timeout=1.0)
                except queue.Empty:
                    logger.warning("SET_P_SETPOINT: NACK received but no error code")

            logger.info(f"SET_P_SETPOINT: received response: type=0x{resp_type:02X}, err=0x{err_code:02X}")

            if resp_type == RESP_ACK:
                with self._lock:
                    self._p_setpoint = setpoint
                logger.info(f"SET_P_SETPOINT: ACK - Setpoint set to {setpoint}")
                return SerialResult(cmd, True, f"Setpoint set to {setpoint}")
            else:  # RESP_NACK
                error_name = get_error_name(err_code)
                logger.warning(f"SET_P_SETPOINT: NACK - {error_name}")
                return SerialResult(cmd, False, f"Set setpoint failed: {error_name}", error=error_name)

        except serial.SerialException as e:
            error_msg = str(e)
            if "device disconnected" in error_msg.lower():
                with self._lock:
                    self._is_connected = False
                    self._serial_connection = None
                    self._led_state = False
                logger.warning("Device disconnected during set setpoint")
                return SerialResult(cmd, False, "Device disconnected", error="Disconnected")
            else:
                logger.error(f"Serial error: {e}")
                return SerialResult(cmd, False, "Communication error", error=str(e))

        except Exception as e:
            logger.error(f"Unexpected error setting setpoint: {e}", exc_info=True)
            return SerialResult(cmd, False, "Unexpected error", error=str(e))

    def _handle_set_p_gain(self, gain: float) -> SerialResult:
        """Handle set P gain command in worker thread."""
        cmd = SerialCommand.SET_P_GAIN
        logger.info(f"SET_P_GAIN: gain={gain}")

        try:
            with self._lock:
                if not self._serial_connection or not self._serial_connection.is_open:
                    logger.warning("SET_P_GAIN: Not connected")
                    return SerialResult(cmd, False, "Not connected", error="No connection")

                # Validate gain range
                if not (0.0 <= gain <= 10.0):
                    logger.warning(f"SET_P_GAIN: Invalid gain {gain}")
                    return SerialResult(cmd, False, "Invalid gain (0.0-10.0)", error="Invalid value")

                # Build and send frame
                frame = build_set_p_gain_frame(gain)
                logger.info(f"SET_P_GAIN: sending frame: {frame.hex()}")

                self._serial_connection.write(frame)
                self._serial_connection.flush()

            # Read response from response queue: 1 byte (ACK/NACK)
            try:
                resp_type = self._response_queue.get(timeout=2.0)
            except queue.Empty:
                logger.error("SET_P_GAIN: No response from device (timeout)")
                return SerialResult(cmd, False, "Device not responding (timeout)", error="Timeout")

            # If NACK, read error code byte
            err_code = 0
            if resp_type == RESP_NACK:
                try:
                    err_code = self._response_queue.get(timeout=1.0)
                except queue.Empty:
                    logger.warning("SET_P_GAIN: NACK received but no error code")

            logger.info(f"SET_P_GAIN: received response: type=0x{resp_type:02X}, err=0x{err_code:02X}")

            if resp_type == RESP_ACK:
                with self._lock:
                    self._p_gain = gain
                logger.info(f"SET_P_GAIN: ACK - Gain set to {gain}")
                return SerialResult(cmd, True, f"P-Gain set to {gain:.2f}")
            else:  # RESP_NACK
                error_name = get_error_name(err_code)
                logger.warning(f"SET_P_GAIN: NACK - {error_name}")
                return SerialResult(cmd, False, f"Set gain failed: {error_name}", error=error_name)

        except serial.SerialException as e:
            error_msg = str(e)
            if "device disconnected" in error_msg.lower():
                with self._lock:
                    self._is_connected = False
                    self._serial_connection = None
                    self._led_state = False
                logger.warning("Device disconnected during set gain")
                return SerialResult(cmd, False, "Device disconnected", error="Disconnected")
            else:
                logger.error(f"Serial error: {e}")
                return SerialResult(cmd, False, "Communication error", error=str(e))

        except Exception as e:
            logger.error(f"Unexpected error setting gain: {e}", exc_info=True)
            return SerialResult(cmd, False, "Unexpected error", error=str(e))

    def _handle_set_p_feed_forward(self, ff: int) -> SerialResult:
        """Handle set feed-forward command in worker thread."""
        cmd = SerialCommand.SET_P_FEED_FORWARD
        logger.info(f"SET_P_FEED_FORWARD: ff={ff}%")

        try:
            with self._lock:
                if not self._serial_connection or not self._serial_connection.is_open:
                    logger.warning("SET_P_FEED_FORWARD: Not connected")
                    return SerialResult(cmd, False, "Not connected", error="No connection")

                # Validate feed-forward range
                if not (0 <= ff <= 100):
                    logger.warning(f"SET_P_FEED_FORWARD: Invalid feed-forward {ff}")
                    return SerialResult(cmd, False, "Invalid feed-forward (0-100)", error="Invalid value")

                # Build and send frame
                frame = build_set_feed_forward_frame(ff)
                logger.info(f"SET_P_FEED_FORWARD: sending frame: {frame.hex()}")

                self._serial_connection.write(frame)
                self._serial_connection.flush()

            # Read response from response queue: 1 byte (ACK/NACK)
            try:
                resp_type = self._response_queue.get(timeout=2.0)
            except queue.Empty:
                logger.error("SET_P_FEED_FORWARD: No response from device (timeout)")
                return SerialResult(cmd, False, "Device not responding (timeout)", error="Timeout")

            # If NACK, read error code byte
            err_code = 0
            if resp_type == RESP_NACK:
                try:
                    err_code = self._response_queue.get(timeout=1.0)
                except queue.Empty:
                    logger.warning("SET_P_FEED_FORWARD: NACK received but no error code")

            logger.info(f"SET_P_FEED_FORWARD: received response: type=0x{resp_type:02X}, err=0x{err_code:02X}")

            if resp_type == RESP_ACK:
                with self._lock:
                    self._p_feed_forward = ff
                logger.info(f"SET_P_FEED_FORWARD: ACK - Feed-forward set to {ff}%")
                return SerialResult(cmd, True, f"Feed-forward set to {ff}%")
            else:  # RESP_NACK
                error_name = get_error_name(err_code)
                logger.warning(f"SET_P_FEED_FORWARD: NACK - {error_name}")
                return SerialResult(cmd, False, f"Set feed-forward failed: {error_name}", error=error_name)

        except serial.SerialException as e:
            error_msg = str(e)
            if "device disconnected" in error_msg.lower():
                with self._lock:
                    self._is_connected = False
                    self._serial_connection = None
                    self._led_state = False
                logger.warning("Device disconnected during set feed-forward")
                return SerialResult(cmd, False, "Device disconnected", error="Disconnected")
            else:
                logger.error(f"Serial error: {e}")
                return SerialResult(cmd, False, "Communication error", error=str(e))

        except Exception as e:
            logger.error(f"Unexpected error setting feed-forward: {e}", exc_info=True)
            return SerialResult(cmd, False, "Unexpected error", error=str(e))

    def _handle_start_p_stream(self, interval_ms: int) -> SerialResult:
        """Handle start P-streaming command in worker thread."""
        cmd = SerialCommand.START_P_STREAM
        logger.info(f"START_P_STREAM: interval={interval_ms}ms")

        try:
            with self._lock:
                if not self._serial_connection or not self._serial_connection.is_open:
                    logger.warning("START_P_STREAM: Not connected")
                    return SerialResult(cmd, False, "Not connected", error="No connection")

                if self._p_streaming:
                    logger.warning("START_P_STREAM: Already streaming")
                    return SerialResult(cmd, False, "Already streaming", error="Already streaming")

                # Validate interval range
                if not (10 <= interval_ms <= 60000):
                    logger.warning(f"START_P_STREAM: Invalid interval {interval_ms}")
                    return SerialResult(cmd, False, "Invalid interval (10-60000ms)", error="Invalid value")

                # Build and send stream start frame
                frame = build_p_stream_start_frame(interval_ms)
                logger.info(f"START_P_STREAM: sending frame: {frame.hex()}")

                self._serial_connection.write(frame)
                self._serial_connection.flush()

            # Read response from response queue: 1 byte (ACK/NACK)
            try:
                resp_type = self._response_queue.get(timeout=2.0)
            except queue.Empty:
                logger.error("START_P_STREAM: No response from device (timeout)")
                return SerialResult(cmd, False, "Device not responding (timeout)", error="Timeout")

            # If NACK, read error code byte
            err_code = 0
            if resp_type == RESP_NACK:
                try:
                    err_code = self._response_queue.get(timeout=1.0)
                except queue.Empty:
                    logger.warning("START_P_STREAM: NACK received but no error code")

            logger.info(f"START_P_STREAM: received response: type=0x{resp_type:02X}, err=0x{err_code:02X}")

            if resp_type == RESP_ACK:
                with self._lock:
                    self._p_streaming = True
                logger.info(f"START_P_STREAM: ACK - P-streaming started at {interval_ms}ms interval")
                return SerialResult(cmd, True, f"P-streaming started ({interval_ms}ms interval)")
            else:  # RESP_NACK
                error_name = get_error_name(err_code)
                logger.warning(f"START_P_STREAM: NACK - {error_name}")
                return SerialResult(cmd, False, f"Start failed: {error_name}", error=error_name)

        except serial.SerialException as e:
            error_msg = str(e)
            if "device disconnected" in error_msg.lower():
                with self._lock:
                    self._is_connected = False
                    self._serial_connection = None
                    self._led_state = False
                logger.warning("Device disconnected during P-stream start")
                return SerialResult(cmd, False, "Device disconnected", error="Disconnected")
            else:
                logger.error(f"Serial error: {e}")
                return SerialResult(cmd, False, "Communication error", error=str(e))

        except Exception as e:
            logger.error(f"Unexpected error starting P-stream: {e}", exc_info=True)
            return SerialResult(cmd, False, "Unexpected error", error=str(e))

    def _handle_stop_p_stream(self) -> SerialResult:
        """Handle stop P-streaming command in worker thread."""
        cmd = SerialCommand.STOP_P_STREAM
        logger.info("STOP_P_STREAM: Stopping P-streaming")

        try:
            with self._lock:
                if not self._serial_connection or not self._serial_connection.is_open:
                    logger.warning("STOP_P_STREAM: Not connected")
                    return SerialResult(cmd, False, "Not connected", error="No connection")

                if not self._p_streaming:
                    logger.warning("STOP_P_STREAM: Not streaming")
                    return SerialResult(cmd, False, "Not streaming", error="Not streaming")

                # Build and send stream stop frame
                frame = build_p_stream_stop_frame()
                logger.info(f"STOP_P_STREAM: sending frame: {frame.hex()}")

                self._serial_connection.write(frame)
                self._serial_connection.flush()

            # Read response from response queue: 1 byte (ACK/NACK)
            try:
                resp_type = self._response_queue.get(timeout=2.0)
            except queue.Empty:
                logger.error("STOP_P_STREAM: No response from device (timeout)")
                with self._lock:
                    self._p_streaming = False
                return SerialResult(cmd, False, "Device not responding (timeout)", error="Timeout")

            # If NACK, read error code byte
            err_code = 0
            if resp_type == RESP_NACK:
                try:
                    err_code = self._response_queue.get(timeout=1.0)
                except queue.Empty:
                    logger.warning("STOP_P_STREAM: NACK received but no error code")

            logger.info(f"STOP_P_STREAM: received response: type=0x{resp_type:02X}, err=0x{err_code:02X}")

            if resp_type == RESP_ACK:
                with self._lock:
                    self._p_streaming = False
                logger.info("STOP_P_STREAM: ACK - P-streaming stopped")
                return SerialResult(cmd, True, "P-streaming stopped")
            else:  # RESP_NACK
                error_name = get_error_name(err_code)
                logger.warning(f"STOP_P_STREAM: NACK - {error_name}")
                return SerialResult(cmd, False, f"Stop failed: {error_name}", error=error_name)

        except serial.SerialException as e:
            error_msg = str(e)
            if "device disconnected" in error_msg.lower():
                with self._lock:
                    self._is_connected = False
                    self._serial_connection = None
                    self._led_state = False
                    self._p_streaming = False
                logger.warning("Device disconnected during P-stream stop")
                return SerialResult(cmd, False, "Device disconnected", error="Disconnected")
            else:
                logger.error(f"Serial error: {e}")
                return SerialResult(cmd, False, "Communication error", error=str(e))

        except Exception as e:
            logger.error(f"Unexpected error stopping P-stream: {e}", exc_info=True)
            return SerialResult(cmd, False, "Unexpected error", error=str(e))

    def _handle_start_p_record(self) -> SerialResult:
        """Handle start P-stream recording command in worker thread."""
        cmd = SerialCommand.START_P_RECORD
        logger.info("START_P_RECORD: Starting P-stream recording")

        try:
            with self._lock:
                if not self._p_streaming:
                    logger.warning("START_P_RECORD: Not streaming")
                    return SerialResult(cmd, False, "Not streaming", error="Not streaming")

                if self._p_recording:
                    logger.warning("START_P_RECORD: Already recording")
                    return SerialResult(cmd, False, "Already recording", error="Already recording")

            # Create data directory if needed
            data_dir = Path(__file__).parent.parent / "data"
            data_dir.mkdir(exist_ok=True)

            # Create filename with timestamp
            from datetime import datetime
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            filename = data_dir / f"p_control_{timestamp}.csv"

            # Open file for writing
            try:
                file_handle = open(filename, 'w', newline='')
                file_handle.write("timestamp,setpoint,measured,pwm,error\n")
            except IOError as e:
                logger.error(f"START_P_RECORD: Failed to create file: {e}")
                return SerialResult(cmd, False, f"Failed to create file: {e}", error=str(e))

            with self._lock:
                self._p_recording = True
                self._p_record_start_time = time.time()
                self._p_record_file = file_handle
                self._p_record_data.clear()

            logger.info(f"START_P_RECORD: Recording started to {filename}")
            return SerialResult(cmd, True, f"Recording started: {filename.name}")

        except Exception as e:
            logger.error(f"Unexpected error starting recording: {e}", exc_info=True)
            return SerialResult(cmd, False, "Unexpected error", error=str(e))

    def _handle_stop_p_record(self) -> SerialResult:
        """Handle stop P-stream recording command in worker thread."""
        cmd = SerialCommand.STOP_P_RECORD
        logger.info("STOP_P_RECORD: Stopping P-stream recording")

        try:
            with self._lock:
                if not self._p_recording:
                    logger.warning("STOP_P_RECORD: Not recording")
                    return SerialResult(cmd, False, "Not recording", error="Not recording")

                file_handle = self._p_record_file
                record_data = list(self._p_record_data)  # Copy data
                self._p_recording = False
                self._p_record_file = None

            # Write remaining buffered data
            samples_written = 0
            if file_handle and record_data:
                try:
                    import csv
                    writer = csv.writer(file_handle)
                    writer.writerows(record_data)
                    samples_written = len(record_data)
                except Exception as e:
                    logger.error(f"STOP_P_RECORD: Failed to write data: {e}")

            # Close file
            if file_handle:
                try:
                    file_handle.close()
                except Exception as e:
                    logger.error(f"STOP_P_RECORD: Failed to close file: {e}")

            # Clear buffer
            with self._lock:
                self._p_record_data.clear()

            logger.info(f"STOP_P_RECORD: Recording stopped, {samples_written} samples saved")
            return SerialResult(cmd, True, f"Recording stopped ({samples_written} samples)")

        except Exception as e:
            logger.error(f"Unexpected error stopping recording: {e}", exc_info=True)
            return SerialResult(cmd, False, "Unexpected error", error=str(e))

    # NOTE: Stream worker removed - only ONE thread can access serial!
    # Streaming data will be parsed in main worker loop when _is_streaming is True

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

    def is_streaming(self) -> bool:
        """Check if currently streaming ADC data."""
        with self._lock:
            return self._is_streaming

    def is_p_streaming(self) -> bool:
        """Check if currently P-streaming data."""
        with self._lock:
            return self._p_streaming

    def is_p_recording(self) -> bool:
        """Check if currently recording P-stream data."""
        with self._lock:
            return self._p_recording

    def flush_recorded_data(self) -> int:
        """Flush buffered recording data to file and return count.

        This should be called periodically from the GUI thread to write
        buffered data to the CSV file.
        """
        with self._lock:
            if not self._p_recording or not self._p_record_file or not self._p_record_data:
                return 0

            # Copy data to write outside the lock
            record_data = list(self._p_record_data)
            self._p_record_data.clear()

        # Write data outside the lock to minimize lock time
        if record_data:
            try:
                import csv
                writer = csv.writer(self._p_record_file)
                writer.writerows(record_data)
                self._p_record_file.flush()
                logger.debug(f"Flushed {len(record_data)} samples to recording file")
                return len(record_data)
            except Exception as e:
                logger.error(f"Failed to flush recorded data: {e}", exc_info=True)
                return 0
        return 0

    def get_stream_interval(self) -> int:
        """Get current streaming interval in milliseconds."""
        with self._lock:
            return self._stream_interval

    def send_task(self, task: SerialTask) -> queue.Queue:
        """Send task to worker thread and return result queue."""
        result_queue = queue.Queue()
        task.result_queue = result_queue

        with self._lock:
            self._pending_tasks.append(task)

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

    def add_adc_data(self, raw_value: int, voltage: float) -> None:
        """Add ADC reading to history (thread-safe)."""
        with self._lock:
            current_time = time.time()
            self._adc_time_history.append(current_time)
            self._adc_raw_history.append(raw_value)
            self._adc_voltage_history.append(voltage)
            # deque with maxlen automatically handles circular buffer

    def get_adc_history(self) -> tuple:
        """Get ADC history data with relative time from now for x-axis."""
        with self._lock:
            if not self._adc_time_history:
                return [], [], []

            # Calculate relative time from now (seconds ago)
            now = time.time()
            x_axis = [(t - now) for t in self._adc_time_history]
            return x_axis, list(self._adc_raw_history), list(self._adc_voltage_history)

    def get_pending_count(self) -> int:
        """Get number of tasks pending in queue."""
        with self._lock:
            return len(self._pending_tasks)

    def get_processing_task(self) -> Optional[str]:
        """Get name of currently processing task."""
        with self._lock:
            if self._processing_task:
                return self._processing_task.command.value
            return None

    def get_pending_list(self) -> List[str]:
        """Get list of pending task names."""
        with self._lock:
            return [t.command.value for t in self._pending_tasks]

    def shutdown(self) -> None:
        """Shutdown the application and worker thread."""
        logger.info("Shutting down...")
        self._running = False

        # Stop streaming if active
        with self._lock:
            if self._is_streaming:
                self._is_streaming = False

        # Stop recording if active
        with self._lock:
            if self._p_recording:
                self._p_recording = False
                # Close recording file
                if self._p_record_file:
                    try:
                        self._p_record_file.close()
                    except Exception:
                        pass
                    self._p_record_file = None
                self._p_record_data.clear()

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
