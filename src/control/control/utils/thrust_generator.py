"""This module provides utilities for generating thrust configurations and converting forces to individual motor thrusts."""

import numpy as np

from geometry_msgs.msg import Wrench


def thruster_configs_to_TAM_inv(
    thruster_count: int,
    thruster_positions: np.ndarray,
    thruster_orientations: np.ndarray,
) -> np.ndarray:
    """
    Compute thrust allocation matrix inverse from thruster configurations.

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

max_wrench = np.array([0.2, 0.2, 0.2, 0.05, 0.05, 0.05])
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
