from control.utils.thrusters import (
    thruster_configs_to_TAM_inv,
    total_force_to_individual_thrusts,
)
from geometry_msgs.msg import Wrench
import pytest
import numpy as np


def test_thruster_configs_to_TAM_inv_expected_output():
    # Test specific configuration of thrusters with known outputs
    thruster_count = 2
    thruster_positions = np.array([[1, 0, 0], [0, 1, 0]])
    thruster_orientations = np.array([[0, 1, 0], [1, 0, 0]])

    expected_TAM_inv = np.array(
        [[1 / 3, 2 / 3, 0, 0, 0, 1 / 3], [2 / 3, 1 / 3, 0, 0, 0, -1 / 3]]
    )

    TAM_inv = thruster_configs_to_TAM_inv(
        thruster_count, thruster_positions, thruster_orientations
    )

    assert np.allclose(
        TAM_inv, expected_TAM_inv
    ), f"Expected TAM_inv to be {expected_TAM_inv}, but got {TAM_inv}"


def test_total_force_to_individual_thrusts_expected_output():
    # Test specific force and torque with known TAM_inv and expected thrusts
    TAM_inv = np.array(
        [[1, 0, 0, 0, 0, -1], [0, 1, 0, 0, 0, 1]]
    )  # Example TAM inverse for 2 thrusters
    wrench = Wrench()
    wrench.force.x = 1
    wrench.force.y = 2
    wrench.torque.z = 3

    expected_thrusts = np.array([-2, 5])

    thrusts = total_force_to_individual_thrusts(TAM_inv, wrench)

    assert np.allclose(
        thrusts, expected_thrusts
    ), f"Expected thrusts to be {expected_thrusts}, but got {thrusts}"


def test_thruster_configs_to_TAM_inv_single_thruster():
    # Test the case with a single thruster
    thruster_count = 1
    thruster_positions = np.array([[0, 0, 0]])  # Position at origin
    thruster_orientations = np.array([[1, 0, 0]])  # Oriented along the x-axis

    expected_TAM = np.array(
        [
            [1],  # x orientation
            [0],  # y orientation
            [0],  # z orientation
            [0],  # roll torque
            [0],  # pitch torque
            [0],  # yaw torque
        ]
    )

    expected_TAM_inv = np.linalg.pinv(expected_TAM)

    TAM_inv = thruster_configs_to_TAM_inv(
        thruster_count, thruster_positions, thruster_orientations
    )

    assert np.allclose(
        TAM_inv, expected_TAM_inv
    ), f"Expected TAM_inv to be {expected_TAM_inv}, but got {TAM_inv}"


def test_thruster_configs_to_TAM_inv_basic():
    # Test basic configuration of thrusters
    thruster_count = 4
    thruster_positions = np.array([[1, 0, 0], [0, 1, 0], [0, 0, 1], [-1, -1, -1]])
    thruster_orientations = np.array([[1, 0, 0], [0, 1, 0], [0, 0, 1], [1, 1, 1]])

    TAM_inv = thruster_configs_to_TAM_inv(
        thruster_count, thruster_positions, thruster_orientations
    )

    assert TAM_inv.shape == (
        thruster_count,
        6,
    ), "TAM_inv matrix should be of size (4, 6)"
    assert isinstance(TAM_inv, np.ndarray), "TAM_inv should be a numpy array"


def test_thruster_configs_to_TAM_inv_zero_thrusters():
    # Test with zero thrusters
    thruster_count = 0
    thruster_positions = np.empty((0, 3))
    thruster_orientations = np.empty((0, 3))

    TAM_inv = thruster_configs_to_TAM_inv(
        thruster_count, thruster_positions, thruster_orientations
    )

    assert (
        TAM_inv.size == 0
    ), "TAM_inv should be an empty array when no thrusters are provided"


def test_thruster_configs_to_TAM_inv_invalid_inputs():
    # Test invalid inputs (mismatch between thruster count and positions/orientations)
    thruster_count = 3
    thruster_positions = np.array([[1, 0, 0], [0, 1, 0]])  # Incorrect size
    thruster_orientations = np.array([[1, 0, 0], [0, 1, 0], [0, 0, 1]])

    with pytest.raises(ValueError):
        thruster_configs_to_TAM_inv(
            thruster_count, thruster_positions, thruster_orientations
        )


def test_total_force_to_individual_thrusts_basic():
    # Test the conversion from wrench to individual thrusts
    TAM_inv = np.identity(6)
    wrench = Wrench()
    wrench.force.x = 1
    wrench.force.y = 2
    wrench.force.z = 3
    wrench.torque.x = 4
    wrench.torque.y = 5
    wrench.torque.z = 6

    thrusts = total_force_to_individual_thrusts(TAM_inv, wrench)

    assert thrusts.shape == (6,), "Thrust vector should have 6 elements"
    assert np.array_equal(
        thrusts, np.array([1, 2, 3, 4, 5, 6])
    ), "Thrusts should match wrench inputs"


def test_total_force_to_individual_thrusts_non_square_TAM():
    # Test with a non-square TAM matrix
    TAM_inv = np.random.rand(4, 6)  # Non-square matrix
    wrench = Wrench()
    wrench.force.x = 1
    wrench.force.y = 2
    wrench.force.z = 3
    wrench.torque.x = 4
    wrench.torque.y = 5
    wrench.torque.z = 6

    thrusts = total_force_to_individual_thrusts(TAM_inv, wrench)

    assert thrusts.shape == (
        4,
    ), "Thrust vector should have 4 elements when TAM_inv has shape (4, 6)"
