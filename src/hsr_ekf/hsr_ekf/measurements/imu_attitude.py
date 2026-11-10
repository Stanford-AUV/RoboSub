"""IMU orientation: the attitude quaternion reported by the IMU's own filter.

Conventions (see also hsr_ekf/__init__.py and measurements/base.py):
  * Frames: world ``odom`` and body ``base_link``, x forward, y left, z up
    (ROS REP-103), not NED. No Euler angles.
  * Quaternions are scalar-last, q = [x, y, z, w], Hamilton product, body to
    world: v_world = R(q) @ v_body.
  * Full state x (13,) = [p, q, v, w]; error state dx (12,) =
    [dp, dtheta, dv, dw] with right perturbation q_true = q (x) Exp(dtheta),
    dtheta a body-frame rotation vector in rad.

Model. The reading z is a unit quaternion (4,), body to world. Its noise is
a small body-frame rotation:

    z = q_true (x) Exp(n),   n ~ N(0, R),   n in rad, shape (3,)

The residual is the body-frame rotation vector that takes the predicted
attitude to the measured one:

    y = quat_error(z_pred, z) = Log(conj(z_pred) (x) z),   shape (3,)

using hsr_ekf.quaternions. quat_error works in the w >= 0 hemisphere, so
z and -z (the same rotation) give the same residual. Because the reading
has 4 numbers but the residual has 3, dim is 3 and R is (3, 3) in
rotation-vector space.

On the vehicle, hardware/nodes/imu.py publishes /imu/orientation
(sensor_msgs/Imu, orientation field). It reads the Xsens filter quaternion
from /filter/quaternion, re-expresses it for base_link with
imu_0.R_sensor_to_base, and removes the heading at the first message while
keeping the tilt absolute (referenced to gravity). So the reading's
world frame has z up, opposite gravity, and x along the vehicle's heading
at the first IMU message; that frame is the filter's odom. The heading has
no external reference (the MTi-200 VRU filter does not use the
magnetometer) and drifts. sensors.yaml uses
rotation variances of 1e-5 rad^2 about the two horizontal axes (tilt, std
about 0.0032 rad) and 0.05 rad^2 about the vertical (heading, std about
0.22 rad), which is why sigma may be given per axis. With the vehicle near
level, the body-z entry is effectively the heading uncertainty.

New sensors are added as new modules implementing Measurement
(measurements/base.py); the filter core does not change.
"""
from __future__ import annotations

from typing import Optional

import numpy as np

from .base import Measurement


class ImuAttitudeMeasurement(Measurement):
    """IMU orientation quaternion, body to world, scalar-last (residual dim 3).

    Reading z: np.ndarray, shape (4,), [x, y, z, w], body to world.
    predict returns the state quaternion (4,); residual returns a (3,)
    body-frame rotation vector; dim is 3.
    """

    def __init__(self, sigma, gate: Optional[float] = None):
        """Store the noise model and gate.

        Parameters
        ----------
        sigma : float or array-like, shape (3,)
            Standard deviation of the attitude noise n, in rad, about the
            body axes [x, y, z]. A scalar (float or int) uses the same value
            on all three axes. Every entry must be > 0.
        gate : float or None, optional
            Chi-squared threshold on NIS with 3 degrees of freedom (11.345
            for 99%), or None (default) to accept every reading.

        Sets
        ----
        dim : int
            3 (the residual dimension, not the length of z).
        R : np.ndarray, shape (3, 3)
            diag(sigma**2), in rad^2; sigma**2 * I3 for a float sigma.
        gate : float or None
            The ``gate`` argument, unchanged.

        Raises
        ------
        ValueError
            If ``sigma`` is neither a scalar nor shape (3,), or any entry
            is not positive.
        """
        raise NotImplementedError("ImuAttitudeMeasurement.__init__")

    def predict(self, x: np.ndarray) -> np.ndarray:
        """Predicted reading: the state's attitude quaternion.

        Parameters
        ----------
        x : np.ndarray, shape (13,)
            Full nominal state [p, q, v, w]. Not modified.

        Returns
        -------
        np.ndarray, shape (4,)
            A copy of q = x[3:7], scalar-last, body to world, returned as
            stored (its sign is not changed).
        """
        raise NotImplementedError("ImuAttitudeMeasurement.predict")

    def residual(self, z, z_pred) -> np.ndarray:
        """Body-frame rotation vector from the predicted to the measured attitude.

        y = quat_error(z_pred, normalize(z)) = Log(conj(z_pred) (x) z), so
        that z = +/- z_pred (x) Exp(y). The angle |y| lies in [0, pi], and
        z and -z give the same y, except when z and z_pred differ by
        exactly pi, where y and -y are both valid and either may be
        returned.

        Parameters
        ----------
        z : array-like, shape (4,)
            Measured quaternion, scalar-last, body to world. It is
            normalized first (quat_normalize), so a reading whose norm is
            off by rounding is accepted. Its sign does not matter.
        z_pred : array-like, shape (4,)
            Predicted quaternion from ``predict``; unit norm.

        Returns
        -------
        np.ndarray, shape (3,)
            Rotation vector in rad, body frame.

        Raises
        ------
        ValueError
            If ``z`` or ``z_pred`` does not have shape (4,), or ``z`` has
            zero norm (raised by quat_normalize).
        """
        raise NotImplementedError("ImuAttitudeMeasurement.residual")

    def jacobian(self, x: np.ndarray) -> np.ndarray:
        """H for the attitude residual with respect to the error state.

        With x_true = x (+) dx, the noise-free reading is q (x) Exp(dtheta)
        and the residual is Log(conj(q) (x) q (x) Exp(dtheta)) = dtheta
        exactly for |dtheta| < pi. So H is the identity on the attitude
        columns, zero elsewhere, and the same for every x.

        Parameters
        ----------
        x : np.ndarray, shape (13,)
            Full nominal state. Not used beyond its shape.

        Returns
        -------
        np.ndarray, shape (3, 12)
            Zeros except H[:, 3:6] = I3 (the dtheta columns).
        """
        raise NotImplementedError("ImuAttitudeMeasurement.jacobian")
