"""This module provides utilities for generating thrust configurations and converting forces to individual motor thrusts."""

import numpy as np
from geometry_msgs.msg import Wrench


def thruster_configs_to_TAM_inv(
    thruster_count: int,
    thruster_positions: np.ndarray,
    thruster_orientations: np.ndarray,
) -> np.ndarray:
    """
    Convert thruster positions and orientations to a thrust allocation matrix inverse.

    Parameters
    ----------
    thruster_count : int
        The number of thrusters.
    thruster_positions : np.ndarray
        The position of each thruster in the body frame.
    thruster_orientations : np.ndarray
        The orientation of each thruster in the body frame.

    Returns
    -------
    np.ndarray
        The thrust allocation matrix inverse.

    """
    TAM = np.empty(shape=(6, thruster_count))
    TAM[:3, :] = thruster_orientations.T
    TAM[3:, :] = np.cross(thruster_positions, thruster_orientations).T
    TAM_inv = np.linalg.pinv(TAM)
    return TAM_inv


def total_force_to_individual_thrusts(TAM_inv: np.ndarray, wrench: Wrench):
    """
    Convert a desired force to motor thrusts.

    Force is a 6x1 vector with the desired force in the x, y, z, roll, pitch, and yaw directions.

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
    return TAM_inv @ wrench_vector
