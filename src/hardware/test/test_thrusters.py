import pytest
import numpy as np
from hardware.utils.thrusters import (
    thrust_to_pwm,
    inverse_quadratic_model,
    BATTERY_VOLTAGES_TO_PWM_COEFFICIENTS,
)


@pytest.mark.parametrize(
    "thrust, voltage, expected_pwm",
    [
        (0, 18, 1491),
        (0, 15, 1482),
        (0, 12, 1483),
        (2, 18, 1787),
        (2, 15, 1864),
        (2, 12, 1913),
        (-2, 18, 1178),
        (-2, 15, 1063),
        (-2, 12, 997),
    ],
)
def test_thrust_to_pwm(thrust, voltage, expected_pwm):
    """Test thrust to PWM conversion with different voltage and thrust values."""
    result = thrust_to_pwm(thrust, voltage)
    assert result == expected_pwm


@pytest.mark.parametrize(
    "voltage, thrust, exception_type, exception_message",
    [
        (22, 0, ValueError, "Voltage exceeds the maximum allowed limit of 20V."),
        (9, 0, ValueError, "Voltage is below the minimum allowed voltage of 10V."),
        (15, 100, ValueError, "Forward thrust exceeds the maximum limit"),
        (15, -100, ValueError, "Reverse thrust exceeds the maximum limit"),
    ],
)
def test_thrust_to_pwm_exceptions(voltage, thrust, exception_type, exception_message):
    """Test exception handling in thrust to PWM conversion."""
    with pytest.raises(exception_type, match=exception_message):
        thrust_to_pwm(thrust, voltage)


@pytest.mark.parametrize(
    "x, a, b, expected_output",
    [
        (
            2,
            BATTERY_VOLTAGES_TO_PWM_COEFFICIENTS[14][0],
            BATTERY_VOLTAGES_TO_PWM_COEFFICIENTS[14][1],
            1480.8191181032553
            + np.sqrt(2) / np.sqrt(BATTERY_VOLTAGES_TO_PWM_COEFFICIENTS[14][0]),
        ),
        (
            -2,
            BATTERY_VOLTAGES_TO_PWM_COEFFICIENTS[12][0],
            BATTERY_VOLTAGES_TO_PWM_COEFFICIENTS[12][1],
            1481.0593557569864
            - np.sqrt(2) / np.sqrt(BATTERY_VOLTAGES_TO_PWM_COEFFICIENTS[12][0]),
        ),
    ],
)
def test_inverse_quadratic_model(x, a, b, expected_output):
    """Test the inverse quadratic model."""
    result = inverse_quadratic_model(x, a, b)
    assert np.isclose(result, expected_output).all()
