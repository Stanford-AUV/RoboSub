"""This module provides utilities for generating thrust configurations and converting forces to individual motor thrusts."""

import numpy as np

from geometry_msgs.msg import Wrench


def thruster_configs_to_TAM_inv(
    thruster_positions: np.ndarray,
    thruster_orientations: np.ndarray,
) -> np.ndarray:
    """
    Compute thrust allocation matrix inverse from thruster configurations.

    Parameters
    ----------
    thruster_positions : np.ndarray
        The position of each thruster in the body frame.
    thruster_orientations : np.ndarray
        The orientation of each thruster in the body frame.

    Returns
    -------
    np.ndarray
        The thrust allocation matrix inverse.

    """
    thruster_count = len(thruster_orientations)
    norms = np.linalg.norm(thruster_orientations, axis=1, keepdims=True)
    thruster_orientations = thruster_orientations / norms
    TAM = np.empty(shape=(6, thruster_count))
    TAM[:3, :] = thruster_orientations.T
    TAM[3:, :] = np.cross(thruster_positions, thruster_orientations).T

    # Physical yaw handedness is inverted relative to the geometric model: a
    # commanded +torque.z (model = CCW about body +Z) drives the vehicle CW in
    # the water. Verified 2026-07-01 by an open-loop /wrench bench test (pure
    # +z torque spun the sub clockwise), which is why closed-loop yaw ran away.
    # Negate the yaw row so a +yaw request maps to thrusts that physically
    # produce +yaw. Only affects yaw; surge/sway/heave/roll/pitch are untouched.
    # TODO: proper fix is a per-thruster direction audit to correct
    # thrusters.yaml (dx/dy signs or motor wiring); this is the contained fix.
    TAM[5, :] *= -1

    TAM_inv = np.linalg.pinv(TAM)
    return TAM_inv


max_wrench = np.array([0.4, 0.4, 0.4, 0.1, 0.1, 0.1])
min_wrench = -max_wrench


def total_force_to_individual_thrusts(TAM_inv: np.ndarray, wrench: Wrench):
    """
    Convert a desired force to motor thrusts.

    Wrench is a 6x1 vector with the desired force in the x, y, z, roll, pitch,
    and yaw directions.

    Parameters
    ----------
    TAM_inv : np.ndarray
        The thrust allocation matrix inverse.
    wrench : Wrench
        The desired force to convert to motor thrusts.

    Returns
    -------
    np.ndarray
        The individual thrusts for each motor.

    """
    wrench_vector = np.array(
        [
            wrench.force.x,
            wrench.force.y,
            wrench.force.z,
            wrench.torque.x,
            wrench.torque.y,
            wrench.torque.z,
        ]
    )
    wrench_vector = np.clip(wrench_vector, min_wrench, max_wrench)
    return TAM_inv @ wrench_vector
