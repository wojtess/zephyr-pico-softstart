"""
Automatic PI Controller Tuning using Ziegler-Nichols Step Response Method.

This module implements host-side autotuning that:
1. Performs open-loop step response test
2. Extracts K (gain), L (dead time), T (time constant)
3. Calculates optimal PI parameters for zero steady-state error
"""

import time
import threading
import logging
import queue
from dataclasses import dataclass
from typing import Optional, Tuple, List, Dict, Any

from .constants import (
    SerialCommand, SerialTask,
    TUNE_STATE_IDLE, TUNE_STATE_PREPARING, TUNE_STATE_STEP_UP,
    TUNE_STATE_STEADY_STATE, TUNE_STATE_COMPLETE, TUNE_STATE_ERROR,
    TUNE_MAX_DURATION, TUNE_STABILITY_SAMPLES, TUNE_STEADY_THRESHOLD
)

logger = logging.getLogger(__name__)


@dataclass
class StepResponseParams:
    """Parameters extracted from step response."""
    K: float  # Steady-state gain (ADC counts per PWM %)
    L: float  # Dead time (seconds)
    T: float  # Time constant (seconds)
    adc_low: float  # Baseline ADC (low PWM)
    adc_high: float  # Steady-state ADC (high PWM)
    pwm_low: int  # Low PWM value
    pwm_high: int  # High PWM value


@dataclass
class PIParameters:
    """Calculated PI controller parameters."""
    kp: float  # Proportional gain (0.0-10.0)
    ki: float  # Integral gain (0.0-10.0)


class Autotuner:
    """Automatic PI controller tuner using Ziegler-Nichols method."""

    def __init__(self, app, adc_to_amps: float = 0.000285):
        """Initialize autotuner.

        Args:
            app: LEDControllerApp instance for serial communication
            adc_to_amps: Conversion factor from ADC counts to amps
        """
        self.app = app
        self.adc_to_amps = adc_to_amps
        self._running = False
        self._state = TUNE_STATE_IDLE
        self._data_lock = threading.Lock()
        self._response_data: List[Tuple[float, int]] = []  # (time, adc_value)
        self._start_time: float = 0.0
        self._params: Optional[StepResponseParams] = None
        self._result: Optional[PIParameters] = None
        self._error: Optional[str] = None

    def start_tuning(self, pwm_low: int, pwm_high: int) -> bool:
        """Start autotuning process in background thread.

        Args:
            pwm_low: Low PWM value for baseline (0-100)
            pwm_high: High PWM value for step (0-100)

        Returns:
            True if tuning started successfully
        """
        if self._running:
            logger.warning("Autotuning already in progress")
            return False

        # Validate parameters
        if not (0 <= pwm_low < pwm_high <= 100):
            logger.error(f"Invalid PWM values: low={pwm_low}, high={pwm_high}")
            return False

        self._running = True
        self._state = TUNE_STATE_PREPARING
        self._response_data = []
        self._params = None
        self._result = None
        self._error = None

        # Start tuning thread
        thread = threading.Thread(
            target=self._run_tuning,
            args=(pwm_low, pwm_high),
            daemon=True
        )
        thread.start()

        logger.info(f"Autotuning started: PWM {pwm_low}% -> {pwm_high}%")
        return True

    def _run_tuning(self, pwm_low: int, pwm_high: int):
        """Main tuning sequence (runs in background thread)."""
        try:
            # Phase 1: Preparation - switch to MANUAL mode
            self._state = TUNE_STATE_PREPARING
            if not self._switch_to_manual():
                self._error = "Failed to switch to MANUAL mode"
                self._state = TUNE_STATE_ERROR
                return

            # Phase 2: Establish baseline at low PWM
            self._state = TUNE_STATE_STEADY_STATE
            if not self._apply_pwm(pwm_low):
                self._error = "Failed to apply low PWM"
                self._state = TUNE_STATE_ERROR
                return

            # Wait for steady state at low PWM
            time.sleep(2.0)  # Allow system to settle
            adc_low = self._read_steady_adc(samples=10)
            if adc_low is None:
                self._error = "Failed to read baseline ADC"
                self._state = TUNE_STATE_ERROR
                return

            logger.info(f"Baseline ADC: {adc_low:.1f} at PWM {pwm_low}%")

            # Phase 3: Apply step
            self._state = TUNE_STATE_STEP_UP
            self._start_time = time.time()
            if not self._apply_pwm(pwm_high):
                self._error = "Failed to apply step PWM"
                self._state = TUNE_STATE_ERROR
                return

            logger.info(f"Step applied: PWM {pwm_high}%")

            # Phase 4: Capture response until steady state
            self._capture_response(pwm_high, timeout=TUNE_MAX_DURATION)

            # Phase 5: Step down and restore
            self._apply_pwm(pwm_low)
            logger.info("Restored low PWM after tuning")

            # Phase 6: Analyze response
            if not self._response_data:
                self._error = "No response data captured"
                self._state = TUNE_STATE_ERROR
                return

            params = self._analyze_response(pwm_low, pwm_high, adc_low)
            if params is None:
                self._error = "Failed to analyze response"
                self._state = TUNE_STATE_ERROR
                return

            self._params = params

            # Phase 7: Calculate PI parameters
            self._result = self._calculate_pi(params)

            self._state = TUNE_STATE_COMPLETE
            logger.info(f"Autotuning complete: Kp={self._result.kp:.3f}, Ki={self._result.ki:.3f}")

        except Exception as e:
            logger.error(f"Autotuning error: {e}", exc_info=True)
            self._error = str(e)
            self._state = TUNE_STATE_ERROR
        finally:
            self._running = False

    def _switch_to_manual(self) -> bool:
        """Switch system to MANUAL mode."""
        task = SerialTask(command=SerialCommand.SET_P_MODE, p_mode=0)
        result_queue = self.app.send_task(task)

        try:
            result = result_queue.get(timeout=3.0)
            if result.success:
                logger.info("Switched to MANUAL mode")
                return True
            else:
                logger.error(f"Failed to switch to MANUAL: {result.message}")
                return False
        except queue.Empty:
            logger.error("Timeout switching to MANUAL mode")
            return False

    def _apply_pwm(self, pwm: int) -> bool:
        """Apply PWM value and wait for confirmation."""
        task = SerialTask(command=SerialCommand.SEND_PWM, pwm_duty=pwm)
        result_queue = self.app.send_task(task)

        try:
            result = result_queue.get(timeout=3.0)
            return result.success
        except queue.Empty:
            return False

    def _read_steady_adc(self, samples: int = 10) -> Optional[float]:
        """Read multiple ADC samples and return average."""
        values = []
        for _ in range(samples):
            task = SerialTask(command=SerialCommand.READ_ADC)
            result_queue = self.app.send_task(task)
            try:
                result = result_queue.get(timeout=2.0)
                if result.success and result.adc_value is not None:
                    values.append(result.adc_value)
            except queue.Empty:
                continue
            time.sleep(0.1)

        if not values:
            return None
        return sum(values) / len(values)

    def _capture_response(self, target_pwm: int, timeout: float = 30.0):
        """Capture step response data until steady state or timeout."""
        start = time.time()
        steady_count = 0

        while (time.time() - start) < timeout:
            # Read current ADC value
            task = SerialTask(command=SerialCommand.READ_ADC)
            result_queue = self.app.send_task(task)

            try:
                result = result_queue.get(timeout=2.0)
                if result.success and result.adc_value is not None:
                    current_time = time.time() - self._start_time
                    current_value = float(result.adc_value)

                    with self._data_lock:
                        self._response_data.append((current_time, current_value))

                    # Check for steady state (2% variation over 50 samples)
                    if len(self._response_data) > TUNE_STABILITY_SAMPLES:
                        recent = [v for t, v in list(self._response_data)[-TUNE_STABILITY_SAMPLES:]]
                        avg = sum(recent) / len(recent)
                        if avg > 0:
                            variation = (max(recent) - min(recent)) / avg

                            if variation < TUNE_STEADY_THRESHOLD:
                                steady_count += 1
                                if steady_count >= 5:  # Confirm stability
                                    logger.info(f"Steady state reached at t={current_time:.2f}s")
                                    break
                            else:
                                steady_count = 0

            except queue.Empty:
                pass

            time.sleep(0.05)  # 20Hz sampling

        elapsed = time.time() - start
        logger.info(f"Captured {len(self._response_data)} samples in {elapsed:.1f}s")

    def _analyze_response(self, pwm_low: int, pwm_high: int, adc_low: float) -> Optional[StepResponseParams]:
        """Analyze step response and extract K, L, T parameters."""
        with self._data_lock:
            if not self._response_data:
                return None
            data = list(self._response_data)

        # Extract values
        times = [t for t, v in data]
        values = [v for t, v in data]

        # Find final steady-state value
        adc_high = sum(values[-20:]) / min(20, len(values))  # Average of last 20 samples

        # Calculate K (gain)
        delta_adc = adc_high - adc_low
        delta_pwm = pwm_high - pwm_low
        K = delta_adc / delta_pwm if delta_pwm > 0 else 0

        # Find L (dead time) - first significant change (5% of step)
        threshold = adc_low + 0.05 * delta_adc
        L = 0.0
        for t, v in data:
            if v > threshold:
                L = t
                break

        # Find T (time constant) - 63.2% response time
        target = adc_low + 0.632 * delta_adc
        T = 0.0
        for t, v in data:
            if v >= target:
                T = t - L
                break

        # Validate parameters
        if K <= 0 or L <= 0 or T <= 0:
            logger.error(f"Invalid parameters: K={K}, L={L}, T={T}")
            return None

        logger.info(f"Extracted parameters: K={K:.3f}, L={L:.3f}s, T={T:.3f}s")
        return StepResponseParams(K, L, T, adc_low, adc_high, pwm_low, pwm_high)

    def _calculate_pi(self, params: StepResponseParams) -> PIParameters:
        """Calculate PI parameters using Ziegler-Nichols formulas.

        For zero steady-state error, use more aggressive integral:
        Kp = 0.9 * T / (K * L)
        Ki = 0.3 * T / (K * L^2)
        """
        # Standard Z-N for PI controller
        kp = 0.9 * params.T / (params.K * params.L)
        ki = 0.3 * params.T / (params.K * params.L * params.L)

        # Clamp to firmware limits (0.0-10.0)
        kp = max(0.0, min(10.0, kp))
        ki = max(0.0, min(10.0, ki))

        logger.info(f"Calculated PI: Kp={kp:.3f}, Ki={ki:.3f}")
        return PIParameters(kp, ki)

    def get_state(self) -> str:
        """Get current tuning state."""
        return self._state

    def get_progress(self) -> float:
        """Get progress (0.0-1.0)."""
        if self._state == TUNE_STATE_IDLE:
            return 0.0
        elif self._state == TUNE_STATE_PREPARING:
            return 0.1
        elif self._state == TUNE_STATE_STEADY_STATE:
            return 0.2
        elif self._state == TUNE_STATE_STEP_UP:
            return 0.5
        elif self._state == TUNE_STATE_COMPLETE:
            return 1.0
        elif self._state == TUNE_STATE_ERROR:
            return 0.0
        return 0.0

    def get_result(self) -> Optional[PIParameters]:
        """Get calculated PI parameters."""
        return self._result

    def get_params(self) -> Optional[StepResponseParams]:
        """Get extracted step response parameters."""
        return self._params

    def get_error(self) -> Optional[str]:
        """Get error message if tuning failed."""
        return self._error

    def is_running(self) -> bool:
        """Check if tuning is in progress."""
        return self._running

    def get_response_data(self) -> List[Tuple[float, int]]:
        """Get captured response data (for plotting)."""
        with self._data_lock:
            return list(self._response_data)
