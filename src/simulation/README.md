# Package: `simulation`

**Build type:** `ament_python`  
**Role:** Bridge ROS 2 to Gazebo Harmonic — fake odometry from sim pose, thrust commands to Gazebo topics, and optional path visualization bridge.

## Executables (`ros2 run simulation <name>`)

| Executable | Module | Summary |
|------------|--------|---------|
| `sensors` | `nodes/sensors.py` | Sub: **`gz/pose`** (`geometry_msgs/PoseStamped`). Pub: **`/odometry/filtered`** (`nav_msgs/Odometry`) with finite-difference twist from pose history. |
| `thrusters` | `nodes/thrusters.py` | Sub: **`thrusts`** (`msgs/ThrustsStamped`) — topic name is relative (often remapped to `/thrusts`). Pubs: **`gz/thruster_0`** … **`gz/thruster_7`** (`std_msgs/Float64`), scaling raw thrust × 1000. |
| `path_bridge` | `nodes/path_bridge.py` | Sub: **`/generated_path`** (`msgs/GeneratedPath`). Republishes to Gazebo transport on **`/generated_path`** using protobuf `gz.custom_msgs.GeneratedPath`. **Contains a hardcoded `sys.path.append` to a build directory** (`/workspaces/RoboSub2/...`) — adjust or remove for your machine. |

## Python helpers (not ROS nodes)

- `simulation/bridge.py`, `bridge_docker.py`, `bridge_local.py` — Docker/host gz-transport bridging (see [SIMULATION.md](../../SIMULATION.md)).

## Launch

`ros2 launch main simulation.py` starts all three simulation executables with `main/launch/params/global.yaml`.

## Nested package: `custom_gz_plugins`

CMake package under `simulation/simulation/custom_gz_plugins/` — Gazebo GUI / rendering plugins and custom protobuf messages for path visualization. Built as a separate colcon package; see [custom_gz_plugins README](simulation/custom_gz_plugins/README.md).

## Tests

`simulation/test/` — ament copyright / template tests.
