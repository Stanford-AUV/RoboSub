# `simulation`

**Build type:** `ament_python` · **Depends on:** `msgs`, `geometry_msgs`, `nav_msgs`, `std_msgs`, `rclpy`
**External:** Gazebo Harmonic (`gz-msgs10` Python bindings)

Stands in for [`hardware`](../hardware/README.md) when there is no vehicle: the same
`/thrusts` in, the same `/odometry/filtered` out, with Gazebo in between. Everything above
the hardware boundary — control, planning, perception — runs unchanged.

> **Not part of the vehicle runtime.** Gazebo Harmonic is Jazzy-era and unavailable in the
> `robosub` Humble conda env, so the sub builds with
> `colcon build --packages-skip simulation`, and [`main`](../main/README.md) dropped its
> dependency on this package. Run simulation on a separate machine — see
> [SIMULATION.md](../../SIMULATION.md).

---

## Entry points

Registered in [`setup.py`](setup.py) — run with `ros2 run simulation <name>`.

| Executable | Module | Node name |
|---|---|---|
| `thrusters` | `simulation/nodes/thrusters.py` | `thrusters` |
| `sensors` | `simulation/nodes/sensors.py` | `sensors` |
| `path_bridge` | `simulation/nodes/path_bridge.py` | `path_bridge` |

---

## Nodes

### `thrusters` — ROS thrust → Gazebo
Mirrors `hardware/thrusters`, but emits a Gazebo force command per thruster instead of
PWM. Thrust is scaled ×1000 into Gazebo units; 8 thrusters, `thruster_0 … thruster_7`.

| Direction | Topic | Type |
|---|---|---|
| Subscribes | `thrusts` (relative) | `msgs/ThrustsStamped` |
| Publishes | `gz/thruster_<i>` | `std_msgs/Float64` |

### `sensors` — Gazebo pose → odometry
Replaces the whole sensing + EKF chain: takes ground-truth pose from Gazebo and
differentiates it for angular velocity, publishing the topic the real EKF would.

| Direction | Topic | Type |
|---|---|---|
| Subscribes | `gz/pose` | `geometry_msgs/PoseStamped` |
| Publishes | `/odometry/filtered` | `nav_msgs/Odometry` |

Never run alongside `robot_localization` or `control/fake_localizer` — all three own
`/odometry/filtered`.

### `path_bridge` — path visualization
Converts a ROS path into the Gazebo protobuf message the `PathVisualizer` plugin renders.
Publishes no ROS topics (it writes to a Gazebo transport topic).

| Direction | Topic | Type |
|---|---|---|
| Subscribes | `/generated_path` | `msgs/GeneratedPath` |

> This node prepends a hardcoded Docker-era path
> (`/workspaces/RoboSub2/build/custom_gz_plugins/...`) to `sys.path` for the generated
> protobuf modules. Fix that for your machine before running it.

---

## Launch

`ros2 launch main simulation.py` ([`../main`](../main/README.md)) starts all three nodes.
Host-side bridges live at the repo root: [`sim_local.sh`](../../sim_local.sh),
[`sim_docker.sh`](../../sim_docker.sh), backed by `simulation/bridge_local.py`,
`simulation/bridge_docker.py`, `simulation/bridge.py`.

## Gazebo assets

| Path | Contents |
|---|---|
| `simulation/models/auv/` | vehicle SDF model |
| `simulation/custom_gz_plugins/` | `PathVisualizer` GUI plugin (C++/QML) plus its protobuf definitions under `proto/gz/custom_msgs/`; built as its own `ament_cmake` package |

## Tests

`colcon test --packages-select simulation`.
