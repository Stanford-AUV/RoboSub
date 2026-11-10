"""Depth sensor: the world-frame z position of base_link.

Conventions (see also hsr_ekf/__init__.py and measurements/base.py):
  * Frames: world ``odom`` and body ``base_link``, x forward, y left, z up
    (ROS REP-103), not NED. Because z points up, p[2] is NEGATIVE below the
    water surface: 2 m deep is p[2] = -2.0.
  * Full state x (13,) = [p, q, v, w]; error state dx (12,) =
    [dp, dtheta, dv, dw]; dp is added straight to p in the world frame.
  * SI units: metres.

Model:
    z = p[2] + n,   n ~ N(0, sigma^2)

The reading is treated as the height of base_link itself. A pressure port
at body offset r from base_link would really read p[2] + (R(q) r)[2]; that
offset is not modelled here, so it appears as a bias that depends on the
tilt (a constant bias when the vehicle is level).

On the vehicle, hardware/nodes/depth_sensor.py publishes this reading on
/position (geometry_msgs/PoseWithCovarianceStamped, frame_id "odom") as
pose.position.z = -depth, so the published value already has this
module's sign. depth comes from msgs/SensorsStamped on /arduino/sensors;
the message does not declare units, and the rest of the stack treats it
as metres. Only z carries information; x and y are zero with a
1e9 variance. sensors.yaml gives the z variance as 0.001 m^2 (std about
0.032 m). The DVL node can also publish its range to the bottom on
/position (frame_id "base_link") when dvl_0 has position enabled; that is
off in sensors.yaml, and such messages are not depth readings.

New sensors are added as new modules implementing Measurement
(measurements/base.py); the filter core does not change.
"""
from __future__ import annotations

from typing import Optional

import numpy as np

from .base import Measurement


class DepthMeasurement(Measurement):
    """World-frame z position of base_link, as published on /position (dim 1).

    Reading z: np.ndarray, shape (1,), metres, world frame, z up
    (negative under water). Pass a scalar as np.array([value]).
    The residual is the inherited default, z - z_pred.
    """

    def __init__(self, sigma: float, gate: Optional[float] = None):
        """Store the noise model and gate.

        Parameters
        ----------
        sigma : float
            Standard deviation of one depth reading, in metres. Must be > 0.
        gate : float or None, optional
            Chi-squared threshold on NIS with 1 degree of freedom (6.635
            for 99%), or None (default) to accept every reading.

        Sets
        ----
        dim : int
            1.
        R : np.ndarray, shape (1, 1)
            [[sigma**2]], in m^2.
        gate : float or None
            The ``gate`` argument, unchanged.

        Raises
        ------
        ValueError
            If ``sigma`` is not positive.
        """
        raise NotImplementedError("DepthMeasurement.__init__")

    def predict(self, x: np.ndarray) -> np.ndarray:
        """Predicted reading: the world z position of base_link.

        Parameters
        ----------
        x : np.ndarray, shape (13,)
            Full nominal state [p, q, v, w]. Not modified.

        Returns
        -------
        np.ndarray, shape (1,)
            [x[2]], metres, world frame, z up.
        """
        raise NotImplementedError("DepthMeasurement.predict")

    def jacobian(self, x: np.ndarray) -> np.ndarray:
        """H for the depth reading with respect to the error state.

        state_plus adds dp to p directly in the world frame, so the reading
        changes one-for-one with dp_z and does not depend on attitude or
        velocity. H is the same for every x.

        Parameters
        ----------
        x : np.ndarray, shape (13,)
            Full nominal state. Not used beyond its shape.

        Returns
        -------
        np.ndarray, shape (1, 12)
            All zeros except H[0, 2] = 1.0 (the dp_z column).
        """
        raise NotImplementedError("DepthMeasurement.jacobian")
