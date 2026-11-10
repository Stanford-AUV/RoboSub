"""Sensor measurement models for the error-state EKF (hsr_ekf.ekf).

Each sensor is one module with one subclass of Measurement (base.py). The
filter core uses only the Measurement interface, so a sensor is added as a
new module here, re-exported below, and the core does not change.

Conventions (see also hsr_ekf/__init__.py):
  * Frames: world ``odom`` and body ``base_link``, x forward, y left, z up
    (ROS REP-103), not NED. No Euler angles.
  * Quaternions are scalar-last, [x, y, z, w], body to world.
  * Full state x (13,) = [p, q, v, w]; error state dx (12,) =
    [dp, dtheta, dv, dw]; every H is (dim, 12), taken with respect to the
    error state.
  * SI units.

Modules:
  base          Measurement interface.
  depth         DepthMeasurement: world z of base_link from /position (dim 1).
  dvl           DvlMeasurement: body velocity of the DVL point, v + w x r (dim 3).
  imu_attitude  ImuAttitudeMeasurement: IMU orientation quaternion (dim 3).
  gyro          GyroMeasurement: body angular velocity w (dim 3).
"""
from __future__ import annotations

from .base import Measurement
from .depth import DepthMeasurement
from .dvl import DvlMeasurement
from .imu_attitude import ImuAttitudeMeasurement
from .gyro import GyroMeasurement

__all__ = [
    "Measurement",
    "DepthMeasurement",
    "DvlMeasurement",
    "ImuAttitudeMeasurement",
    "GyroMeasurement",
]
