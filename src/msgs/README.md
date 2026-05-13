# Package: `msgs`

**Build type:** `ament_cmake` + `rosidl_generate_interfaces`  
**Role:** Custom ROS 2 message and service definitions shared across `hardware`, `control`, `perception`, `planning`, and `simulation`.

## Messages (`msg/`)

Defined in `CMakeLists.txt` (source of truth):

| Message | Typical use |
|---------|----------------|
| `ThrustsStamped` | Per-thruster normalized thrust commands (`/thrusts`). |
| `PWMsStamped` | ESC PWM values from `hardware/thrusters` → `arduino`. |
| `SensorsStamped` | Teensy/Arduino sensor feedback (e.g. depth, voltage). |
| `Float32Stamped` | Generic stamped scalar (e.g. depth for joystick). |
| `DVLBeam`, `DVLData`, `DVLTarget`, `DVLVelocity` | DVL-related payloads. |
| `MTi200Data` | Xsens-related payload (if used). |
| `AlignedDepthImage` | RGB + aligned depth + intrinsics for perception. |
| `Detection3DPoints`, `Detection3DPointsArray` | 3D detection outputs. |
| `GeneratedPath` | Dense pose/twist trajectory (planning ↔ Gazebo bridge). |

## Services (`srv/`)

| Service | Definition |
|---------|------------|
| `GetWaypoints` | Request: empty. Response: `nav_msgs/Path waypoints`. |

Used by `control/path_tracker` (client name `get_waypoints`). **No server node ships in this repo**; implement or launch one when using path tracking.

## Build notes

Regenerate interfaces after edits:

```bash
colcon build --packages-select msgs
```

IDL dependencies include `std_msgs`, `geometry_msgs`, `sensor_msgs`, `vision_msgs`, `nav_msgs`, `nmea_msgs`, `mavros_msgs`, `builtin_interfaces`.
