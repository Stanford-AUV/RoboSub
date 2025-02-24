# Simulation Module

**Authors and Maintainers**: [List maintainers and contact info]  

---

## Purpose  
The Simulation Module is designed to simulate RoboSub in a Gazebo environment. It integrates various sensors and actuators to mimic real-world operations, providing a platform for testing and development of control and navigation algorithms.

---

## Nodes  
- **PathBridgeNode**: Bridges ROS 2 path messages to Gazebo custom messages for path visualization.
- **DVLBridgeNode**: Converts Gazebo Doppler Velocity Log (DVL) messages to ROS 2 messages for velocity tracking.
- **Sensors**: Converts sensor messages from Gazebo into a unified format for state estimation and control.
- **Thrusters**: Converts individual thruster commands into Gazebo messages to control the AUV's thrusters.

---

## Communication  
- **Inputs**: 
  - `/generated_path` (ROS 2): Subscribed by `PathBridgeNode` to receive path data.
  - `dvl` (Gazebo): Subscribed by `DVLBridgeNode` to receive DVL data.
  - `gz/imu`, `gz/depth`, `gz/pose` (Gazebo): Subscribed by `Sensors` node for IMU, depth, and pose data.
  - `thrusts` (ROS 2): Subscribed by `Thrusters` node for thruster commands.

- **Outputs**: 
  - `/generated_path` (Gazebo): Published by `PathBridgeNode` for path visualization.
  - `dvl` (ROS 2): Published by `DVLBridgeNode` for DVL data.
  - `imu`, `depth`, `odometry` (ROS 2): Published by `Sensors` node for processed sensor data.
  - `gz/thruster_X` (Gazebo): Published by `Thrusters` node for each thruster control.

- **Interconnections**: 
  - The `PathVisualizer` plugin in Gazebo uses the `/generated_path` topic to visualize the path.
  - The `DVLBridgeNode` and `Sensors` node facilitate data exchange between Gazebo and ROS 2.

---

## Development Notes  
- The `PathVisualizer` plugin is a custom Gazebo GUI plugin that visualizes paths using cones to represent waypoints and orientations.
- The AUV model is defined in `model.urdf` and includes sensors like IMU, altimeter, and DVL, as well as thrusters for movement.
- The simulation environment is configured in `world.urdf`, which includes plugins for physics, sensors, and visualization.

---

## Testing  
To test the `PathVisualizer` and other components, follow these steps in separate terminals:

1. Run the simulation script:
   ```bash
   ./sim.sh
   ```

2. Launch the simulation environment:
   ```bash
   ros2 launch src/launch/simulation.py
   ```

3. Launch the planning module:
   ```bash
   ros2 launch src/launch/planning.py
   ```

These steps will set up the necessary environment to test the `PathVisualizer` functionality and other simulation components.

---

## References  
- [List any relevant documentation, papers, or resources.]