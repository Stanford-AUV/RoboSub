# `control`

**Build type:** `ament_python` · **Depends on:** `msgs`, `geometry_msgs`, `nav_msgs`, `rclpy`

Turns a desired pose into forces: a PID loop produces a body-frame **wrench**, and the
thrust allocator splits that wrench into per-thruster magnitudes. Everything downstream
of `/thrusts` belongs to [`hardware`](../hardware/README.md); everything upstream of
`/desired/pose` belongs to [`planning`](../planning/README.md).

**Maintainers:** Ali Ahmad (aliahmad@stanford.edu), Khaled Messai (kmessai@stanford.edu),
Iris Xu (izxu28@stanford.edu), Deren Ji (derenji@stanford.edu)

---

## Entry points

Registered in [`setup.py`](setup.py) as `console_scripts` — run with `ros2 run control <name>`.

| Executable | Module | Node name |
|---|---|---|
| `pid_control` | `control/nodes/pid_control.py` | `pid_control` |
| `thrust_generator` | `control/nodes/thrust_generator.py` | `thrust_generator` |
| `controller` | `control/nodes/controller.py` | `controller` |
| `path_tracker` | `control/nodes/path_tracker.py` | `path_tracker` |
| `logger` | `control/nodes/logger.py` | `logger` |
| `test_controller` | `control/nodes/test_controller.py` | `test_controller` |
| `test_thrust` | `control/nodes/test_thrust.py` | `thrust_generator` |
| `fake_localizer` | `control/nodes/fake_localizer.py` | `fake_localizer` |
| `pid_visualizer` | `control/nodes/pid_visualizer.py` | `pid_visualizer` |

---

## Nodes

### `pid_control` — the flight controller
The node actually flown. Position + orientation PID; gains come from
[`control/pid.yaml`](control/pid.yaml), resolved from the package **share** dir
(`get_package_share_directory("control")`) with a source-tree fallback.

| Direction | Topic | Type |
|---|---|---|
| Subscribes | `/odometry/filtered` | `nav_msgs/Odometry` |
| Subscribes | `/desired/pose` | `nav_msgs/Odometry` |
| Publishes | `/wrench` | `geometry_msgs/WrenchStamped` |

Runs its loop on a 60 Hz timer. It declares **no** parameters — the
`velocity_only` parameter set in `main/launch/control.py` is inert here (it belongs to
`controller`).

### `thrust_generator` — wrench → per-thruster thrust
Builds the inverse thrust-allocation matrix (TAM) once at startup from
[`hardware/hardware/thrusters.yaml`](../hardware/hardware/thrusters.yaml) (read from the
**`hardware`** package share dir), then republishes allocated thrusts at 60 Hz.

| Direction | Topic | Type |
|---|---|---|
| Subscribes | `/wrench` | `geometry_msgs/WrenchStamped` |
| Publishes | `/thrusts` | `msgs/ThrustsStamped` |

### `controller` — older waypoint-chasing controller
| Direction | Topic | Type |
|---|---|---|
| Subscribes | `/odometry/filtered` | `nav_msgs/Odometry` |
| Subscribes | `waypoint` (relative) | `nav_msgs/Odometry` |
| Publishes | `wrench` (relative) | `geometry_msgs/WrenchStamped` |

**Parameter:** `velocity_only` (bool) — controls velocity only instead of pose + velocity.
Superseded by `pid_control` in the flown stack.

### `fake_localizer` — hardware-free pipeline test
Integrates `F = ma` with drag so the full control loop can be exercised on a laptop.

| Direction | Topic | Type |
|---|---|---|
| Subscribes | `/wrench` | `geometry_msgs/WrenchStamped` |
| Publishes | `/odometry/filtered` | `nav_msgs/Odometry` (50 Hz) |

Never run this alongside the EKF — both own `/odometry/filtered`.

### `pid_visualizer` — live PID plot
Subscribes `/odometry/filtered`, `/desired/pose`, `/wrench` (all read-only) and plots
current vs. desired pose, error magnitude and wrench output. Publishes nothing.
Needs a `DISPLAY`.

### Deprecated nodes
Marked `DEPRECATED` in source; kept for bench work and reference.

| Node | Subscribes | Publishes | Note |
|---|---|---|---|
| `path_tracker` | `/odometry/filtered` | `waypoint` (`Odometry`) | Also a **service client** of `get_waypoints` (`msgs/GetWaypoints`). No server node exists in this repo, so it blocks at startup. |
| `test_controller` | — | `waypoint` (`Odometry`), 1 Hz | Walks a constant x setpoint; drives `controller` on the bench. |
| `test_thrust` | — | `thrusts` (**relative**, not `/thrusts`) | Fixed `ThrustsStamped` at 100 Hz. |
| `logger` | `odometry` (relative) | — | Writes matplotlib PNGs headlessly. |

`control/nodes/execute_action.py` is a stub (no `super().__init__`, not an entry point).

---

## Launch files

| File | Starts |
|---|---|
| [`launch/control.py`](launch/control.py) (`ros2 launch control control.py`) | `pid_control` only |
| [`launch/wrench_to_pwm.py`](launch/wrench_to_pwm.py) | `thrust_generator` + `hardware/thrusters` — bench wrench→PWM chain |
| `main/launch/control.py` ([`../main`](../main/README.md)) | `pid_control` + `logger`; `thrust_generator` deliberately **not** started here (it comes from `hardware.py`) |

---

## Library code (`control/utils/`)

| Module | Contents |
|---|---|
| `pid.py` | `PID` class — per-axis position/velocity PID with limits |
| `state.py` | `State` — pose/twist container, `from_odometry_msg()` |
| `thrust_generator.py` | `thruster_configs_to_TAM_inv()`, `total_force_to_individual_thrusts()` |
| `wrench.py` | numpy → `WrenchStamped` helper |
| `path_*.py` | standalone spline/path experiments (not imported by nodes) |

---

## Config

[`control/pid.yaml`](control/pid.yaml) — position and orientation gains, limits, and
optional feed-forward terms. Installed to `share/control/pid.yaml`, which is what the node
reads, so **rebuild after editing it** or your new gains will not be loaded.

## Tests

`colcon test --packages-select control`, or `./test.sh` from the workspace root.
