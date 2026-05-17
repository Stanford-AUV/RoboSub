# Package: `main`

**Build type:** `ament_cmake`  
**Role:** Installs top-level launch files and shared parameters. This package does not contain application logic; it composes other packages.

## What is installed

- `share/main/launch/*.py` — system bring-up
- `share/main/launch/params/global.yaml` — shared parameters (thruster geometry, EKF `robot_localization` config, camera snippets, timer/history defaults)

## Launch files (as coded)

| Launch file | Behavior |
|-------------|----------|
| `main.py` | Includes **`hardware.py`**, **`localization.py`**, **`control.py`**. **`manual.py` is commented out** — autonomy does **not** start keyboard/joystick. **`thrust_generator`** runs only inside **`hardware.py`**; **`main/launch/control.py`** runs **`controller`** + **`test_controller`** (no second thrust node). |
| `hardware.py` | `xsens_mti_ros2_driver` + `hardware` nodes (`imu`, `dvl`, `thrusters`, `arduino`) + `control/thrust_generator`; optional `sensors_plot` via launch arg `plot`. Requires **xsens** package in workspace/overlay. |
| `localization.py` | `hardware/sensors` + `robot_localization/ekf_node` + static TFs `base_link` → `imu_frame`, `dvl_frame`. |
| `state.py` | Same EKF stack as `localization.py` without manual/hardware extras. |
| `manual.py` | **`control/thrust_generator`** + **`manual/keyboard`** (NATS bridge). Requires **`nats-server`** and **`./keyboard_local.sh`**. Do **not** launch alongside **`hardware.py`** (duplicate **`thrust_generator`**). |
| `control.py` | **`controller`** + **`test_controller`** only. **`thrust_generator`** is expected from **`hardware.py`** when using **`main.py`**. For **`ros2 launch main control.py`** without hardware, run **`ros2 run control thrust_generator`** (see comments in `control.py`) or use **`manual.py`** / **`wrench_to_pwm`**. |
| `perception.py` | Single node: `perception/object_localizer`. |
| `planning.py` | Empty `LaunchDescription` (placeholder). |
| `simulation.py` | `simulation/thrusters`, `simulation/sensors`, `simulation/path_bridge` with `global.yaml`. |

## How to run

From workspace root after `source install/setup.bash`:

```bash
ros2 launch main main.py
ros2 launch main control.py
ros2 launch main hardware.py
# …etc.
```

## Dependencies (package.xml)

`control`, `hardware`, `manual`, `perception`, `planning`, `simulation`, `gui`, plus standard message/runtime packages. External stacks (e.g. `xsens_mti_ros2_driver`, `robot_localization`) are referenced from launch files but not always listed as `exec_depend` here.
