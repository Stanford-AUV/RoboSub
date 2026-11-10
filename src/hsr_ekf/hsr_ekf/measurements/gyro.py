"""Gyroscope: body-frame angular velocity.

Conventions (see also hsr_ekf/__init__.py and measurements/base.py):
  * Frames: world ``odom`` and body ``base_link``, x forward, y left, z up
    (ROS REP-103), not NED.
  * Full state x (13,) = [p, q, v, w], with w = x[10:13] the angular
    velocity in the body frame, rad/s. Error state dx (12,) =
    [dp, dtheta, dv, dw].
  * SI units: rad/s.

Model:
    z = w + n,   n ~ N(0, sigma^2 I3),   body frame, rad/s

Gyro bias is NOT modelled. The error state has no bias entry, so a
constant gyro offset b makes the reading w + b look like truth to this
model; the filter pulls w toward w + b, and through attitude integration
the offset turns into heading drift. Adding a bias would change the state
layout (hsr_ekf.state, the process models and every measurement's H),
not only this module.

On the vehicle, the Xsens driver publishes the gyro on /imu/angular_velocity
(geometry_msgs/Vector3Stamped) when pub_angular_velocity is true, which
xsens_mti_node.yaml sets. sensors.yaml records imu_0.R_sensor_to_base as
identity because the mount rotation is stored in the MTi itself, so the
driver's axes are taken as base_link axes. hardware/nodes/imu.py does not
forward the gyro; a node using this measurement subscribes to the driver
topic directly.

Noise: the sensors.yaml comments give a gyro white-noise density
N = 1.4e-4 rad/s/sqrt(Hz) from a 12 h static log at 400 Hz. That is a
per-sample standard deviation of about N * sqrt(f_s) = 2.8e-3 rad/s at
f_s = 400 Hz (smaller after averaging down to a lower rate). The
angular_covariance of 0.01 in sensors.yaml is a placeholder, not a
measurement. In the same log the IMU heading drifted -3096 deg over 12 h,
an average of about -1.25e-3 rad/s, so a long-term rate error of that
order is what the missing bias state leaves uncorrected.
"""
from __future__ import annotations

from typing import Optional

import numpy as np

from .base import Measurement


class GyroMeasurement(Measurement):
    """Body angular velocity w (dim 3); the gyro bias is not modelled.

    Reading z: np.ndarray, shape (3,), rad/s, base_link axes. The residual
    is the inherited default, z - z_pred.
    """

    def __init__(self, sigma: float, gate: Optional[float] = None):
        """Store the noise model and gate.

        Parameters
        ----------
        sigma : float
            Standard deviation of each angular-rate component, rad/s, the
            same for all three axes. Must be > 0.
        gate : float or None, optional
            Chi-squared threshold on NIS with 3 degrees of freedom (11.345
            for 99%), or None (default) to accept every reading.

        Sets
        ----
        dim : int
            3.
        R : np.ndarray, shape (3, 3)
            sigma**2 * I3, in (rad/s)^2.
        gate : float or None
            The ``gate`` argument, unchanged.

        Raises
        ------
        ValueError
            If ``sigma`` is not positive.
        """
        raise NotImplementedError("GyroMeasurement.__init__")

    def predict(self, x: np.ndarray) -> np.ndarray:
        """Predicted reading: the body angular velocity.

        Parameters
        ----------
        x : np.ndarray, shape (13,)
            Full nominal state [p, q, v, w]. Not modified.

        Returns
        -------
        np.ndarray, shape (3,)
            A copy of w = x[10:13], rad/s, body frame.
        """
        raise NotImplementedError("GyroMeasurement.predict")

    def jacobian(self, x: np.ndarray) -> np.ndarray:
        """H for the gyro reading with respect to the error state.

        The reading is w itself, so H picks out the dw block and is the
        same for every x.

        Parameters
        ----------
        x : np.ndarray, shape (13,)
            Full nominal state. Not used beyond its shape.

        Returns
        -------
        np.ndarray, shape (3, 12)
            Zeros except H[:, 9:12] = I3 (the dw columns).
        """
        raise NotImplementedError("GyroMeasurement.jacobian")
