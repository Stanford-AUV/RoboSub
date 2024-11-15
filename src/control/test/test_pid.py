from control.utils.pid import PID
from control.utils.state import State
import pytest
import numpy as np
import spatialmath as sm

def test_wrench_output_():
    pid = PID(
        kP_position=np.array([1, 1, 1]),
        kD_position=np.array([0, 0, 0]),
        kI_position=np.array([0, 0, 0]),
        kP_orientation=np.array([1, 1, 1]),
        kD_orientation=np.array([0, 0, 0]),
        kI_orientation=np.array([0, 0, 0]),
        max_signal_force=np.array([1, 1, 1]),
        max_signal_torque=np.array([1, 1, 1]),
        max_integral_position=np.array([1, 1, 1]),
        max_integral_orientation=np.array([1, 1, 1]),
    )

