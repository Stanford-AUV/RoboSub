↳ src Contains all the core packages and functionalities for the robot's operation, structured into distinct modules.

   ➤ control ⟹ Manages the robot's actuation and movement, including PID control, trajectory execution, and motor commands.
   ➤ hardware ⟹ Interfaces with the robot's physical components such as sensors, actuators, and microcontrollers.
   ➤ perception ⟹ Processes sensory data (e.g., vision, LIDAR) for tasks like object detection, localization, and mapping.
   ➤ planning ⟹ Handles motion planning, path optimization, and high-level decision-making algorithms.
   ➤ simulation ⟹ Provides a virtual testing environment for the robot, integrating with tools like Gazebo for realistic simulations.

   ➤ launch ⟹ Contains ROS2 launch files to easily set up and execute various configurations and modules.
   ➤ msgs ⟹ Defines custom ROS2 message types used for communication between nodes.
