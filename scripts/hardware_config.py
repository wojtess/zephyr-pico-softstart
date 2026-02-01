"""
Hardware configuration and ADC-to-current conversion for RP2040 LED Control.

Current sensing circuit:
- Shunt resistor: 0.05 ohm
- Op-amp gain: 11x
- Vref: 3.3V
- ADC: 12-bit (0-4095)
- Current range: 0-6A
"""

from typing import Tuple


# Hardware constants
ADC_BITS: int = 12
ADC_MAX: int = (1 << ADC_BITS) - 1  # 4095
VREF: float = 3.3
SHUNT_R: float = 0.05  # ohms
OPAMP_GAIN: float = 11.0

# Derived constants
VOLTAGE_PER_ADC: float = VREF / ADC_MAX  # Volts per ADC LSB
OHMS_TOTAL: float = SHUNT_R * OPAMP_GAIN  # Total resistance seen by ADC
AMPS_PER_ADC: float = VOLTAGE_PER_ADC / OHMS_TOTAL  # Amps per ADC LSB


def adc_to_current(adc_raw: int) -> float:
    """
    Convert ADC raw value to current in amperes.

    Args:
        adc_raw: Raw ADC value (0-4095)

    Returns:
        Current in amperes (0.0 - 6.0)

    Raises:
        ValueError: If adc_raw is out of range
    """
    if not (0 <= adc_raw <= ADC_MAX):
        raise ValueError(f"ADC value {adc_raw} out of range [0, {ADC_MAX}]")

    voltage = adc_raw * VOLTAGE_PER_ADC
    shunt_voltage = voltage / OPAMP_GAIN
    current = shunt_voltage / SHUNT_R
    return current


def current_to_adc(current: float) -> int:
    """
    Convert current in amperes to ADC raw value.

    Args:
        current: Current in amperes (0.0 - 6.0)

    Returns:
        ADC raw value (0-4095)

    Raises:
        ValueError: If current is negative
    """
    if current < 0:
        raise ValueError(f"Current {current} cannot be negative")

    # Clamp to max measurable current
    current_clamped = min(current, 6.0)

    shunt_voltage = current_clamped * SHUNT_R
    voltage = shunt_voltage * OPAMP_GAIN
    adc_raw = int((voltage / VREF) * ADC_MAX)

    # Clamp to ADC range
    return min(adc_raw, ADC_MAX)


def get_adc_range() -> Tuple[int, int]:
    """Return ADC range as (min, max)."""
    return (0, ADC_MAX)


def get_current_range() -> Tuple[float, float]:
    """Return current range in amperes as (min, max)."""
    return (0.0, 6.0)


def get_adc_resolution() -> float:
    """
    Return ADC resolution in amperes per LSB.

    At 6A full scale over 4095 steps: ~1.47 mA per LSB.
    """
    return AMPS_PER_ADC
