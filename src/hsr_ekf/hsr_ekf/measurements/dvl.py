"""Doppler velocity log (DVL): body-frame velocity of the DVL's mounting point.

Conventions (see also hsr_ekf/__init__.py and measurements/base.py):
  * Frames: world ``odom`` and body ``base_link``, x forward, y left, z up
    (ROS REP-103), not NED.
  * Full state x (13,) = [p, q, v, w], with v (m/s) and w (rad/s) in the
    body frame. Error state dx (12,) = [dp, dtheta, dv, dw].
  * SI units: m, m/s, rad/s.

Model. A DVL mounted at body offset r_dvl from base_link measures the
velocity of its own location, not of base_link. For a rigid body, in
base_link axes:

    z = v + w x r_dvl + n,   n ~ N(0, sigma^2 I3)

The reading does not depend on attitude or position: both v and r_dvl are
body-frame quantities.

On the vehicle, hardware/nodes/dvl.py publishes /velocity
(geometry_msgs/TwistWithCovarianceStamped, frame_id "base_link"). The node
already rotates the velocity from DVL axes into base_link axes with
dvl_0.R_sensor_to_base, so z is in base_link axes. The node can also
subtract w x r itself, using omega from /angular and
dvl_0.sensor_pos_in_base. Choose r_dvl so the lever arm is applied exactly
once:
  * If /velocity is still the raw velocity of the DVL point, pass the DVL
    offset as r_dvl. This is the current case: no node in this repository
    publishes /angular (imu.py forwards only orientation, and bno085.py
    publishes on /bno085_<n>/angular), so the node's omega stays zero and
    its correction is zero.
  * If the node's correction becomes active, /velocity is already v, so
    pass r_dvl = [0, 0, 0].
The offset itself is not settled: sensors.yaml gives sensor_pos_in_base =
[-0.125, 0.0, -0.03] m, while the static transform base_to_dvl_tf in
main/launch/localization.py uses [-0.105, 0.0, -0.0625] m.

R is fixed at construction from sigma. The per-message covariance that the
node writes from the DVL's reported velocity error is not used here.
sensors.yaml's fallback velocity variance is 0.01 (m/s)^2 (std 0.1 m/s).

New sensors are added as new modules implementing Measurement
(measurements/base.py); the filter core does not change.
"""
from __future__ import annotations

from typing import Optional

import numpy as np

from .base import Measurement


class DvlMeasurement(Measurement):
    """Body-frame velocity of the DVL at body offset r_dvl: v + w x r_dvl (dim 3).

    Reading z: np.ndarray, shape (3,), m/s, base_link axes. The residual is
    the inherited default, z - z_pred.

    Attributes
    ----------
    r_dvl : np.ndarray, shape (3,)
        DVL position relative to base_link, body frame, metres.
    """

    def __init__(self, r_dvl, sigma: float, gate: Optional[float] = None):
        """Store the lever arm, the noise model and the gate.

        Parameters
        ----------
        r_dvl : array-like, shape (3,)
            DVL position relative to base_link, body frame, metres. Stored
            as a float copy. Use zeros if the incoming velocity has already
            been shifted to base_link (see the module docstring).
        sigma : float
            Standard deviation of each velocity component, m/s, the same
            for all three axes. Must be > 0.
        gate : float or None, optional
            Chi-squared threshold on NIS with 3 degrees of freedom (11.345
            for 99%), or None (default) to accept every reading.

        Sets
        ----
        r_dvl : np.ndarray, shape (3,)
            Float copy of the ``r_dvl`` argument.
        dim : int
            3.
        R : np.ndarray, shape (3, 3)
            sigma**2 * I3, in (m/s)^2.
        gate : float or None
            The ``gate`` argument, unchanged.

        Raises
        ------
        ValueError
            If ``r_dvl`` does not have shape (3,), or ``sigma`` is not
            positive.
        """
        raise NotImplementedError("DvlMeasurement.__init__")

    def predict(self, x: np.ndarray) -> np.ndarray:
        """Predicted velocity of the DVL point, in base_link axes.

        Parameters
        ----------
        x : np.ndarray, shape (13,)
            Full nominal state [p, q, v, w]. Not modified.

        Returns
        -------
        np.ndarray, shape (3,)
            v + w x r_dvl, with v = x[7:10] and w = x[10:13]; m/s, body
            frame. Equals v when r_dvl is zero or w is zero.
        """
        raise NotImplementedError("DvlMeasurement.predict")

    def jacobian(self, x: np.ndarray) -> np.ndarray:
        """H for the DVL reading with respect to the error state.

        The model is linear in v and w, so H is the same for every x:
            d z / d dv = I3
            d z / d dw = -skew(r_dvl)      (since w x r = -r x w = -skew(r) w)
            d z / d dp = d z / d dtheta = 0
        skew is hsr_ekf.rotations.skew.

        Parameters
        ----------
        x : np.ndarray, shape (13,)
            Full nominal state. Not used beyond its shape.

        Returns
        -------
        np.ndarray, shape (3, 12)
            Zeros except H[:, 6:9] = I3 (dv columns) and
            H[:, 9:12] = -skew(r_dvl) (dw columns).
        """
        raise NotImplementedError("DvlMeasurement.jacobian")
