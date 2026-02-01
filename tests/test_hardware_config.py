"""
Tests for hardware_config.py - ADC to current conversion.

Hardware specs:
- Shunt R: 0.05 ohm
- Op-amp gain: 11x
- Vref: 3.3V
- ADC: 12-bit (0-4095)
- Current range: 0-6A
"""

import pytest
import math
from scripts.hardware_config import (
    adc_to_current,
    current_to_adc,
    get_adc_range,
    get_current_range,
    get_adc_resolution,
    ADC_MAX,
    VREF,
    SHUNT_R,
    OPAMP_GAIN,
)

# =============================================================================
# Test Constants
# =============================================================================

# ADC to current conversion tolerances
ADC_CURRENT_TOLERANCE = 0.01
PRECISION_THRESHOLD_PCT = 0.5
ROUNDTRIP_ERROR_THRESHOLD_PCT = 1.0


# =============================================================================
# Test Suite 1: Basic Conversions
# =============================================================================

@pytest.mark.unit
class TestBasicConversions:
    """Test basic ADC to current conversion at key points."""

    def test_adc_to_current_zero(self):
        """ADC 0 should give 0A."""
        assert adc_to_current(0) == 0.0

    def test_adc_to_current_mid_scale(self):
        """Mid-scale (2048) should give ~3.0A."""
        result = adc_to_current(2048)
        expected = 3.0
        assert result == pytest.approx(expected, abs=ADC_CURRENT_TOLERANCE)

    def test_adc_to_current_full_scale(self):
        """Full scale (4095) should give 6.0A."""
        result = adc_to_current(4095)
        expected = 6.0
        assert result == pytest.approx(expected, abs=ADC_CURRENT_TOLERANCE)

    def test_adc_to_current_quarter_scale(self):
        """Quarter scale (1024) should give ~1.5A."""
        result = adc_to_current(1024)
        expected = 1.5
        assert result == pytest.approx(expected, abs=ADC_CURRENT_TOLERANCE)

    def test_adc_to_current_1_amp(self):
        """ADC ~683 should give 1.0A."""
        result = adc_to_current(683)
        assert result == pytest.approx(1.0, abs=ADC_CURRENT_TOLERANCE)

    def test_adc_to_current_2_amp(self):
        """ADC ~1366 should give 2.0A."""
        result = adc_to_current(1366)
        assert result == pytest.approx(2.0, abs=ADC_CURRENT_TOLERANCE)


# =============================================================================
# Test Suite 2: Roundtrip Accuracy
# =============================================================================

@pytest.mark.unit
class TestRoundtripAccuracy:
    """Test current -> ADC -> current conversion accuracy."""

    @pytest.mark.parametrize("current", [
        0.0, 0.5, 1.0, 1.5, 2.0, 2.5, 3.0, 4.0, 5.0, 6.0
    ])
    def test_roundtrip_accuracy(self, current):
        """Roundtrip error should be < 1% for common currents."""
        adc = current_to_adc(current)
        recovered = adc_to_current(adc)

        if current > 0:
            error_pct = abs(recovered - current) / current * 100
            assert error_pct < ROUNDTRIP_ERROR_THRESHOLD_PCT, \
                f"Roundtrip error {error_pct:.2f}% exceeds {ROUNDTRIP_ERROR_THRESHOLD_PCT}% for {current}A -> ADC {adc} -> {recovered}A"
        else:
            assert recovered == 0.0, "Zero current should roundtrip to zero"


# =============================================================================
# Test Suite 3: Precision (1% Requirement)
# =============================================================================

@pytest.mark.unit
class TestPrecisionRequirement:
    """Test that conversions meet 1% precision requirement across range."""

    @pytest.mark.parametrize("pct", range(0, 101, 10))
    def test_precision_at_percent_scale(self, pct):
        """Conversion error should be < 0.5% at 10% increments."""
        adc_raw = int(ADC_MAX * pct / 100)
        expected_current = 6.0 * pct / 100
        actual_current = adc_to_current(adc_raw)

        if expected_current > 0:
            error_pct = abs(actual_current - expected_current) / expected_current * 100
            assert error_pct < PRECISION_THRESHOLD_PCT, \
                f"Conversion error {error_pct:.3f}% at {pct}% scale (ADC={adc_raw})"


# =============================================================================
# Test Suite 4: Edge Cases
# =============================================================================

@pytest.mark.unit
class TestEdgeCases:
    """Test edge cases and boundary conditions."""

    def test_adc_min(self):
        """ADC minimum should give 0A."""
        assert adc_to_current(0) == 0.0

    def test_adc_max(self):
        """ADC maximum should give ~6A."""
        assert adc_to_current(ADC_MAX) == pytest.approx(6.0, abs=ADC_CURRENT_TOLERANCE)

    def test_current_zero_to_adc(self):
        """0A should give ADC 0."""
        assert current_to_adc(0.0) == 0

    def test_current_max_to_adc(self):
        """6A should give ADC 4095."""
        assert current_to_adc(6.0) == ADC_MAX

    def test_negative_current_raises_error(self):
        """Negative current should raise ValueError."""
        with pytest.raises(ValueError, match="cannot be negative"):
            current_to_adc(-1.0)

    def test_adc_below_zero_raises_error(self):
        """ADC < 0 should raise ValueError."""
        with pytest.raises(ValueError, match="out of range"):
            adc_to_current(-1)

    def test_adc_above_max_raises_error(self):
        """ADC > 4095 should raise ValueError."""
        with pytest.raises(ValueError, match="out of range"):
            adc_to_current(4096)

    def test_current_above_6a_clamps(self):
        """Current > 6A should clamp to ADC max."""
        result = current_to_adc(7.0)
        assert result == ADC_MAX, "Current >6A should clamp to ADC 4095"

    def test_very_large_current_clamps(self):
        """Very large current should clamp gracefully."""
        result = current_to_adc(100.0)
        assert result == ADC_MAX


# =============================================================================
# Test Suite 5: Practical Values
# =============================================================================

@pytest.mark.unit
class TestPracticalValues:
    """Test commonly used current values in the application."""

    @pytest.mark.parametrize("current,expected_adc,tolerance", [
        (0.5, 341, 2),
        (1.0, 683, 2),
        (2.0, 1365, 2),
        (3.0, 2048, 2),
        (4.0, 2730, 2),
        (5.0, 3413, 2),
        (6.0, 4095, 2),
    ])
    def test_current_to_adc_practical(self, current, expected_adc, tolerance):
        """Common currents should convert to expected ADC values."""
        actual_adc = current_to_adc(current)
        assert abs(actual_adc - expected_adc) <= tolerance, \
            f"{current}A should give ADC ~{expected_adc}±{tolerance}, got {actual_adc}"

    @pytest.mark.parametrize("expected_current,adc,tolerance", [
        (0.5, 341, ADC_CURRENT_TOLERANCE),
        (1.0, 683, ADC_CURRENT_TOLERANCE),
        (2.0, 1365, ADC_CURRENT_TOLERANCE),
        (3.0, 2048, ADC_CURRENT_TOLERANCE),
        (4.0, 2730, ADC_CURRENT_TOLERANCE),
        (5.0, 3413, ADC_CURRENT_TOLERANCE),
        (6.0, 4095, ADC_CURRENT_TOLERANCE),
    ])
    def test_adc_to_current_practical(self, expected_current, adc, tolerance):
        """Common ADC values should convert to expected currents."""
        actual_current = adc_to_current(adc)
        assert actual_current == pytest.approx(expected_current, abs=tolerance)


# =============================================================================
# Test Suite 6: Linearity
# =============================================================================

@pytest.mark.unit
class TestLinearity:
    """Test that conversion is linear across the range."""

    def test_linear_at_multiple_points(self):
        """Conversion should be linear at multiple points."""
        adc_values = [0, 500, 1000, 1500, 2000, 2500, 3000, 3500, 4000, 4095]
        currents = [adc_to_current(adc) for adc in adc_values]

        # Check that each step is proportional
        for i in range(1, len(currents)):
            if adc_values[i - 1] > 0:
                expected_ratio = adc_values[i] / adc_values[i - 1]
                actual_ratio = currents[i] / currents[i - 1]
                error = abs(actual_ratio - expected_ratio)
                assert error < ADC_CURRENT_TOLERANCE, \
                    f"Non-linearity detected between ADC {adc_values[i - 1]} and {adc_values[i]}"

    def test_constant_ratio(self):
        """Current/ADC ratio should be constant."""
        ratio_1 = adc_to_current(1000) / 1000
        ratio_2 = adc_to_current(2000) / 2000
        ratio_3 = adc_to_current(3000) / 3000

        assert abs(ratio_1 - ratio_2) < 0.0001
        assert abs(ratio_2 - ratio_3) < 0.0001


# =============================================================================
# Test Suite 7: Floating Point Precision
# =============================================================================

@pytest.mark.unit
class TestFloatingPointPrecision:
    """Test handling of floating point edge cases."""

    def test_very_small_current(self):
        """Very small current (1mA) should be representable."""
        very_small = 0.001  # 1mA
        adc = current_to_adc(very_small)
        assert adc >= 0, "Small current should give valid ADC"

    def test_one_percent_scale(self):
        """1% of full scale (60mA) should be measurable."""
        one_percent = 0.06  # 60mA = 1% of 6A
        adc = current_to_adc(one_percent)
        recovered = adc_to_current(adc)

        # At low end, we may have more error due to quantization
        assert adc > 0, "1% scale should give non-zero ADC"
        assert abs(recovered - one_percent) / one_percent < 0.15, \
            f"1% scale measurement has large error: {abs(recovered - one_percent) / one_percent * 100:.1f}%"

    def test_fractional_currents(self):
        """Fractional currents should convert correctly."""
        test_values = [0.1, 0.25, 0.75, 1.25, 2.5]
        for current in test_values:
            adc = current_to_adc(current)
            recovered = adc_to_current(adc)
            error_pct = abs(recovered - current) / current * 100
            assert error_pct < ROUNDTRIP_ERROR_THRESHOLD_PCT, f"Fractional {current}A has {error_pct:.2f}% error"


# =============================================================================
# Test Suite 8: Utility Functions
# =============================================================================

@pytest.mark.unit
class TestUtilityFunctions:
    """Test utility functions."""

    def test_get_adc_range(self):
        """ADC range should be (0, 4095)."""
        assert get_adc_range() == (0, 4095)

    def test_get_current_range(self):
        """Current range should be (0.0, 6.0)."""
        assert get_current_range() == (0.0, 6.0)

    def test_get_adc_resolution(self):
        """ADC resolution should be ~1.47mA per LSB."""
        resolution = get_adc_resolution()
        expected = 6.0 / 4095  # ~0.001465
        assert abs(resolution - expected) < 0.0001
        assert resolution > 0.0014  # ~1.4mA
        assert resolution < 0.0015  # ~1.5mA


# =============================================================================
# Test Suite 9: Hardware Constants
# =============================================================================

@pytest.mark.unit
class TestHardwareConstants:
    """Verify hardware constants are correctly defined."""

    def test_adc_max_is_12bit(self):
        """ADC_MAX should be 4095 (12-bit)."""
        assert ADC_MAX == 4095

    def test_vref_is_3v3(self):
        """VREF should be 3.3V."""
        assert VREF == 3.3

    def test_shunt_resistor(self):
        """Shunt resistor should be 0.05 ohm."""
        assert SHUNT_R == 0.05

    def test_opamp_gain(self):
        """Op-amp gain should be 11x."""
        assert OPAMP_GAIN == 11.0


# =============================================================================
# Test Suite 10: Integration with Protocol (placeholder)
# =============================================================================

@pytest.mark.integration
class TestProtocolIntegration:
    """Test that current conversion works with protocol data structures."""

    def test_adc_value_typical_range(self):
        """Typical ADC readings from protocol should be in valid range."""
        # Simulate ADC values from protocol response
        test_adc_values = [100, 500, 1000, 2048, 3000, 4000]

        for adc in test_adc_values:
            current = adc_to_current(adc)
            assert 0.0 <= current <= 6.0, f"ADC {adc} gave invalid current {current}A"

    def test_conversion_for_setpoints(self):
        """Common setpoint values should convert correctly."""
        # P-controller setpoints in amperes
        setpoints_amps = [0.5, 1.0, 2.0, 3.0, 4.0, 5.0]

        for setpoint in setpoints_amps:
            adc = current_to_adc(setpoint)
            current = adc_to_current(adc)
            error_pct = abs(current - setpoint) / setpoint * 100
            assert error_pct < ROUNDTRIP_ERROR_THRESHOLD_PCT, f"Setpoint {setpoint}A has {error_pct:.2f}% error"


# =============================================================================
# Test Suite 11: Type Validation Tests
# =============================================================================

@pytest.mark.unit
class TestTypeValidation:
    """Test that conversion functions handle invalid input types correctly."""

    def test_adc_to_current_with_string_raises_error(self):
        """String input should raise TypeError or ValueError."""
        with pytest.raises((TypeError, ValueError)):
            adc_to_current("123")

    def test_adc_to_current_with_none_raises_error(self):
        """None input should raise TypeError or ValueError."""
        with pytest.raises((TypeError, ValueError)):
            adc_to_current(None)

    def test_adc_to_current_with_list_raises_error(self):
        """List input should raise TypeError or ValueError."""
        with pytest.raises((TypeError, ValueError)):
            adc_to_current([1000])

    def test_current_to_adc_with_string_raises_error(self):
        """String input should raise TypeError or ValueError."""
        with pytest.raises((TypeError, ValueError)):
            current_to_adc("1.5")

    def test_current_to_adc_with_none_raises_error(self):
        """None input should raise TypeError or ValueError."""
        with pytest.raises((TypeError, ValueError)):
            current_to_adc(None)

    def test_current_to_adc_with_list_raises_error(self):
        """List input should raise TypeError or ValueError."""
        with pytest.raises((TypeError, ValueError)):
            current_to_adc([1.5])


# =============================================================================
# Test Suite 12: Edge Case Tests (Critical Edge Cases)
# =============================================================================

@pytest.mark.unit
class TestCriticalEdgeCases:
    """Test critical edge cases including special float values."""

    def test_adc_to_current_with_infinity_setpoint(self):
        """Test handling of infinity in setpoint context."""
        # The function clamps to 6.0, so infinity gives ADC_MAX
        result = current_to_adc(float('inf'))
        assert result == ADC_MAX, "Infinity should clamp to ADC max"

    def test_adc_to_current_with_nan_setpoint(self):
        """Test handling of NaN in setpoint context."""
        # NaN cannot be converted to int, so it raises ValueError
        with pytest.raises(ValueError, match="cannot convert float NaN to integer"):
            current_to_adc(float('nan'))

    def test_current_to_adc_with_large_value_clamps(self):
        """Setpoint values > 4095 should clamp to ADC max."""
        # Test a value that would exceed ADC max
        large_current = 100.0  # Would give ADC way above 4095
        result = current_to_adc(large_current)
        assert result == ADC_MAX, "Very large setpoint should clamp to ADC max"

    def test_current_to_adc_with_negative_raises_error(self):
        """Negative setpoint should raise ValueError."""
        with pytest.raises(ValueError, match="cannot be negative"):
            current_to_adc(-1.0)

    def test_negative_setpoint_edge_case(self):
        """Test various negative setpoint values."""
        negative_values = [-0.1, -1.0, -100.0]
        for val in negative_values:
            with pytest.raises(ValueError, match="cannot be negative"):
                current_to_adc(val)

    def test_zero_current_boundary(self):
        """Test zero current handling (boundary condition)."""
        adc = current_to_adc(0.0)
        current = adc_to_current(adc)
        assert current == 0.0, "Zero current should roundtrip perfectly"

    def test_maximum_current_boundary(self):
        """Test maximum current boundary (6A)."""
        adc = current_to_adc(6.0)
        assert adc == ADC_MAX, "6A should give max ADC value"
        current = adc_to_current(adc)
        assert current == pytest.approx(6.0, abs=ADC_CURRENT_TOLERANCE)

    def test_very_small_positive_current(self):
        """Test very small positive current values."""
        # Test sub-milliamp values
        small_values = [0.0001, 0.001, 0.01]  # 0.1mA, 1mA, 10mA
        for val in small_values:
            adc = current_to_adc(val)
            assert adc >= 0, f"Small current {val}A should give non-negative ADC"
            assert adc <= ADC_MAX, f"Small current {val}A should not exceed max ADC"

