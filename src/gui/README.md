# Package: `gui`

**Build type:** `ament_python`  
**Role:** Web HUD bridge — subscribes to IMU and odometry, broadcasts JSON over WebSocket to `auv_hud.html`, and accepts joystick / e-stop / launch commands from the browser.

## Executable

```bash
ros2 run gui bridge
```

Entry point: `gui.ros2_gui_bridge:main` → `HudBridgeNode` in `gui/ros2_gui_bridge.py`.

## Topics (defaults in code)

Constants at top of `ros2_gui_bridge.py` (edit to match your robot):

| Direction | Topic | Type |
|-----------|-------|------|
| Subscribe | `/imu/data` | `sensor_msgs/Imu` |
| Subscribe | `/odometry/filtered` | `nav_msgs/Odometry` |
| Publish | `/cmd_vel` | `geometry_msgs/Twist` |
| Publish | `/estop` | `std_msgs/Bool` |

## WebSocket

- **Bind:** `0.0.0.0`:**`9091`** (`WS_HOST`, `WS_PORT` in code). Docstring at file top still mentions `9090` in places; **9091 is what the server uses.**
- **Protocol:** newline-delimited JSON. Incoming types include `cmd_vel`, `estop`, `launch`; server pushes `type: "state"` with pose, velocities, IMU summary.

## Dependencies

- **Python:** `websockets` (see file header for install hint; may use venv fallback).
- **ROS:** `rclpy`, `geometry_msgs`, `std_msgs`; code imports **`nav_msgs`** — ensure `ros-jazzy-nav-msgs` is available (consider adding `<exec_depend>nav_msgs</exec_depend>` to `package.xml` if builds fail).

## Static assets

- `gui/auv_hud.html` — open in a browser pointed at the WebSocket host/port.
- `gui/launch_sub.sh` — invoked when the HUD sends a `launch` message (`subprocess.Popen`).

## package.xml

The current description in `package.xml` is copy-pasted from another subsystem; treat this README as accurate for the GUI package.
