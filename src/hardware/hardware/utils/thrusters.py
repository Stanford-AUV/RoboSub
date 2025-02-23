import numpy as np

# EXPERIMENTAL_OFFSET = 15
# Forward: 1.3 s
# Backward: 1.5 s

# EXPERIMENTAL_OFFSET = 17
# Forward: 0.8 s
# Backward 1.2 s

EXPERIMENTAL_OFFSET = 15

BATTERY_VOLTAGES_TO_PWM_COEFFICIENTS = {
    10: [1.8343560975506323e-05, 1481.466452853926],
    12: [2.3194713277650457e-05, 1481.0593557569864],
    14: [2.8235512746477604e-05, 1480.8191181032553],
    16: [3.28040783880607e-05, 1480.6115720889093],
    18: [3.638563822982247e-05, 1481.421310875163],
    20: [3.8972515653759295e-05, 1480.4018523627565],
}


def quadratic_model(x, a, b):
    return a * np.square(x - b)


def inverse_quadratic_model(x, a, b):
    second_part = np.sqrt(np.abs(x)) / np.sqrt(a)
    return np.where(x >= 0, b + second_part, b - second_part)


def thrust_to_pwm(thrust: float, voltage=14.8):
    """
    Convert a desired thrust to motor PWM values.

    Parameters
    ----------
    thrust : float
        The desired thrust in N.
    voltage : float, optional
        The battery voltage in V, by default 14.8

    Returns
    -------
    int
        The PWM value for the motor.

    """
    if voltage > 20:
        raise ValueError("Voltage exceeds the maximum allowed limit of 20V.")
    elif voltage < 10:
        raise ValueError("Voltage is below the minimum allowed voltage of 10V.")

    # Requested thrust must be 20% below saturation limit, according to https://bluerobotics.com/store/thrusters/t100-t200-thrusters/t200-thruster-r2-rp/
    if thrust > 0.8 * (0.374 * voltage - 0.78):  # mx + b, calculated by hand
        raise ValueError("Forward thrust exceeds the maximum limit")
    if -thrust > 0.8 * (0.266 * voltage - 0.272):  # mx + b, calculated by hand
        raise ValueError("Reverse thrust exceeds the maximum limit")

    if voltage >= 18:
        low = 18
        high = 20
    elif voltage >= 16:
        low = 16
        high = 18
    elif voltage >= 14:
        low = 14
        high = 16
    elif voltage >= 12:
        low = 12
        high = 14
    else:
        low = 10
        high = 12

    weight = (thrust - low) / 2
    low_pwm: float = inverse_quadratic_model(
        thrust, *BATTERY_VOLTAGES_TO_PWM_COEFFICIENTS[low]
    )
    high_pwm: float = inverse_quadratic_model(
        thrust, *BATTERY_VOLTAGES_TO_PWM_COEFFICIENTS[high]
    )
    pwm = (1 - weight) * low_pwm + weight * high_pwm

    return np.round(pwm).astype(int) + EXPERIMENTAL_OFFSET
