import pytest
from unittest.mock import MagicMock, patch
import rclpy


def test_reference_callback():
    """
    Test the reference_callback method.
    """
    from control.nodes.controller import Controller
    from threading import Lock

    rclpy.init()

    # Mock the Controller node
    mock_policy = MagicMock()
    node = Controller(mock_policy)

    # Mock cur_state and path
    node.cur_state = MagicMock()
    node.path = MagicMock()

    # Replace the lock with a mock lock
    node.lock = MagicMock(spec=Lock)

    # Mock nextReference
    with patch("controller.nextReference") as mock_next_reference:
        # Configure nextReference to return a mock ref_state object
        mock_ref_state = MagicMock()
        mock_ref_state.path = MagicMock()
        mock_next_reference.return_value = mock_ref_state

        # Mock getNextReference on the path object
        mock_ref_state.path.getNextReference = MagicMock(
            return_value="mock_next_reference"
        )

        # Call the method under test
        node.reference_callback()

        # Assertions
        mock_next_reference.assert_called_once_with(node.cur_state, node.path)
        assert node.ref_state == "mock_next_reference"
        assert node.path_follower == mock_ref_state.path
        # mock_ref_state.path.getNextReference.assert_called_once()
        # node.lock.__enter__.assert_called_once()  # Ensure the lock was acquired
        # node.lock.__exit__.assert_called_once()  # Ensure the lock was released

    # Clean up the node
    node.destroy_node()
