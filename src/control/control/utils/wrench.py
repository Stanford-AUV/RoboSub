"""numpy force/torque vectors -> geometry_msgs/WrenchStamped."""

from geometry_msgs.msg import Vector3, Wrench, WrenchStamped

import numpy as np



class AbstractWrench():
    """Wrench class to abstract conversion of numpy arrays to ROS messages."""

    def __init__(self, force: np.array, torque: np.array):
        """
        Initialize a Wrench object with force and torque vectors.

        Parameters
        ----------
        force : np.array
            The force vector.
        torque : np.array
            The torque vector.
        """
        self.force = force
        self.torque = torque
        self.wrench_msg = Wrench(
            force=Vector3(x=force[0], y=force[1], z=force[2]),
            torque=Vector3(x=torque[0], y=torque[1], z=torque[2])
        )

    def to_msg(self):
        """Convert the Wrench object to a ROS WrenchStamped message."""
        msg = WrenchStamped()
        msg.wrench = self.wrench_msg
        return msg
