from perception.utils.bounding_box import oriented_bounding_box
from numpy.testing import assert_almost_equal, assert_array_almost_equal
import numpy as np


def test_oriented_bounding_box():
    # Define a rectangular prism aligned along the coordinate axes for a known solution
    points = np.array(
        [
            [1.0, 2.0, 3.0],
            [1.0, 2.0, 6.0],
            [1.0, 5.0, 3.0],
            [1.0, 5.0, 6.0],
            [4.0, 2.0, 3.0],
            [4.0, 2.0, 6.0],
            [4.0, 5.0, 3.0],
            [4.0, 5.0, 6.0],
        ]
    )

    # Expected values based on the known rectangular prism
    expected_center = np.array([2.5, 3.5, 4.5])
    expected_size = np.array([3.0, 3.0, 3.0])  # width, height, depth
    expected_rotation_matrix = np.array(
        [[0.0, 0.0, 1.0], [0.0, 1.0, 0.0], [1.0, 0.0, 0.0]]
    )

    # Calculate bounding box
    center, size, rotation_matrix = oriented_bounding_box(points)

    # Test exact center, size, and rotation matrix
    assert_almost_equal(
        center, expected_center, decimal=5, err_msg="Center is incorrect"
    )
    assert_almost_equal(size, expected_size, decimal=5, err_msg="Size is incorrect")
    assert_array_almost_equal(
        rotation_matrix,
        expected_rotation_matrix,
        decimal=5,
        err_msg="Rotation matrix is incorrect",
    )
