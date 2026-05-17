# Package: `control`

**Build type:** `ament_python`  
**Role:** Wrench commands → thruster allocation, closed-loop PID control, waypoint following helpers, logging and test nodes.

## Executables (`ros2 run control <name>`)

| Executable | Module | Summary |
|------------|--------|---------|
| `thrust_generator` | `nodes/thrust_generator.py` | Sub: `geometry_msgs/WrenchStamped` **`/wrench`**. Pub: `msgs/ThrustsStamped` **`/thrusts`**. Timer ~60 Hz. Loads thruster geometry from `hardware/hardware/thrusters.yaml` via hardcoded relative path. |
| `controller` | `nodes/controller.py` | Sub: `nav_msgs/Odometry` **`/odometry/filtered`**, **`waypoint`** (relative name → resolves under node namespace). Pub: **`wrench`** (`WrenchStamped`). Parameter `velocity_only` (bool). Uses `control.utils.pid` + `State`. |
| `pid_control` | `nodes/pid_control.py` | Sub: `/world/pose`, `/desired/pose`. Pub: `/wrench`. PID gains from `control/pid.yaml` (path resolved in node). |
| `path_tracker` | `nodes/path_tracker.py` | Client: `get_waypoints` (`msgs/GetWaypoints`). Sub: `/odometry/filtered`. Pub: `waypoint` (`Odometry`). Marked deprecated in source; still the only structured waypoint follower. |
| `test_controller` | `nodes/test_controller.py` | Timer pub: **`waypoint`** — constant test pose for controller debugging. |
| `test_thrust` | `nodes/test_thrust.py` | Publishes fixed `ThrustsStamped` on topic **`thrusts`** (relative; not `/thrusts`). Marked deprecated. |
| `logger` | `nodes/logger.py` | Sub: `odometry` (relative). Matplotlib logging. Marked deprecated. |

**Stub (not usable as a node):** `nodes/execute_action.py` — `ExecuteAction` class does not call `super().__init__`; not in `setup.py`.

## Launches

- **`main/launch/control.py`** (installed with package **`main`**): **`controller`** + **`test_controller`** only. **`thrust_generator`** is intentionally **not** started here so **`ros2 launch main main.py`** has a single thrust node from **`hardware.py`**. See file comments for running **`control.py` without** `hardware.py`.
- **`share/control/launch/control.py`** (package **`control`**): currently runs **`pid_control`** only (paths point at `main`’s `global.yaml`).
- **`wrench_to_pwm.py`** — `thrust_generator` + `hardware/thrusters` with `global.yaml` from `main`.

## Library code (`control/utils/`)

Includes PID (`pid.py`), wrench/thruster math (`thrust_generator.py`, `wrench.py`), path sampling helpers (`path_2d_no_rotation.py`, `path_3d_no_rotation.py`, `path_3d_rotation_unbounded.py`), and `state.py` (used by `path_tracker`, `pid_control`).

## Simulation smoke test

1. Run your Gazebo / sim stack per [SIMULATION.md](../../SIMULATION.md).
2. `ros2 launch main simulation.py` — sim sensors + thrusters + path bridge.
3. Full autonomy-style stack: `ros2 launch main main.py`, or add **`thrust_generator`** when using **`main/launch/control.py`** alone (see that file’s header comment). Package launch: `ros2 launch control control.py`.

## Tests

Package tests live under `control/test/`. From workspace root: `./test.sh` or `colcon test --packages-select control`.
