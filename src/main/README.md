# `main`

**Build type:** `ament_cmake` · **Depends on:** `control`, `hardware`, `manual`, `perception`, `planning`, `gui`

Launch files only — no nodes, no Python modules. [`CMakeLists.txt`](CMakeLists.txt)
installs `launch/` into `share/main/launch`, which is what makes
`ros2 launch main <file>` work. This package is the single place where the subsystem
packages are composed into a running vehicle.

> `simulation` was deliberately dropped from the `exec_depend` list: it needs Gazebo
> Harmonic, which is not available on the Humble runtime env. Build the workspace with
> `--packages-skip simulation` on the sub.

---

## Entry points

There are no `console_scripts`. The entry points are the launch files:

| Command | What it starts |
|---|---|
| `ros2 launch main main.py` | the whole vehicle: `imu.py` + `hardware.py` + `localization.py` + `control.py` (`manual.py` is commented out, so **autonomy never starts teleop**) |
| `ros2 launch main imu.py` | `xsens_mti_ros2_driver/xsens_mti_node` + `hardware/imu` |
| `ros2 launch main hardware.py` | `hardware/dvl`, `control/thrust_generator`, `hardware/thrusters`, `hardware/arduino`, `hardware/daisy`, `hardware/depth`, and `hardware/sensors_plot` (argument `plot:=true` by default) |
| `ros2 launch main localization.py` | `hardware/sensors`, `robot_localization/ekf_node` (params from [`../localization`](../localization/README.md)), and the `base_link→imu_frame` / `base_link→dvl_frame` static TFs |
| `ros2 launch main control.py` | `control/pid_control` + `control/logger` |
| `ros2 launch main planning.py` | `planning/path_generator` + `planning/path_streamer`; argument `waypoints_path:=<file>.yaml` (default `segments.yaml`) |
| `ros2 launch main perception.py` | `perception/object_world_localizer`, `perception/oak_node`, `perception/heading_corrector` (`object_localizer` commented out — needs `ultralytics`) |
| `ros2 launch main manual.py` | `control/thrust_generator` + `manual/keyboard` |
| `ros2 launch main state.py` | minimal `hardware/sensors` + `ekf_node` (no params file, no TFs) — bench EKF smoke test |
| `ros2 launch main simulation.py` | `simulation/thrusters`, `simulation/sensors`, `simulation/path_bridge` |

The launcher you actually use on the sub is [`launch_sub.sh`](../../launch_sub.sh) at the
repo root, which sequences these files (see below) and guarantees a safe teardown.

---

## Composition rules

These are the constraints baked into the launch files. Violating them gives you two
publishers on one topic.

- **`thrust_generator` is started by `hardware.py`, not `control.py`.** That is why
  `ros2 launch main control.py` alone produces no thrust — pair it with `hardware.py`, or
  run `ros2 run control thrust_generator` yourself.
- **Never run `manual.py` together with `hardware.py`** — both start
  `thrust_generator`. On the vehicle use `hardware.py` + `ros2 run manual keyboard`.
- **Never run teleop together with `main.py`** — `manual/keyboard` and
  `control/pid_control` both publish `/wrench`.
- **`imu.py` is separate from `hardware.py` on purpose.** The Xsens onboard filter (AHS
  heading) needs ~20 s to converge before anything consumes it, so `launch_sub.sh` starts
  `imu.py` first and waits.
- `main/launch/control.py` passes a `velocity_only` parameter to `pid_control`, which does
  not declare it. The parameter is inert; it belongs to `control/controller`.

### Three-phase bring-up in `launch_sub.sh`

```
1. imu                      wait IMU_SETTLE_DELAY (20 s)   Xsens filter converges
2. hardware localization control   wait PATH_DELAY (5 s)    EKF converges, thrusters idle
3. planning                                                 setpoints start flowing
```

`path_generator` exits ~3 s after the mission finishes; `planning.py` wires
`on_exit=Shutdown()` to that, which is how `launch_sub.sh` detects completion and tears
the stack down with neutral PWM on every thruster.

---

## Dataflow across the launch files

```
xsens_mti_node ──/filter/quaternion──▶ hardware/imu ──/imu/orientation──┐
hardware/dvl   ──/velocity, /position ──────────────────────────────────┤
hardware/depth ──/position ─────────────────────────────────────────────┼─▶ ekf_node ──/odometry/filtered──┐
perception/heading_corrector ──/heading_correction ─────────────────────┘                                   │
                                                                                                            │
planning/path_generator ──/desired/pose──▶ control/pid_control ◀────────────────────────────────────────────┘
                                                   │
                                                   └──/wrench──▶ control/thrust_generator ──/thrusts──▶ hardware/thrusters ──pwms──▶ hardware/arduino
```

## Params

[`launch/params/thrusters.yaml`](launch/params/thrusters.yaml) is a launch-level copy of
the thruster geometry; the authoritative file the nodes read is
[`hardware/hardware/thrusters.yaml`](../hardware/hardware/thrusters.yaml).
[`launch/save_replay.sh`](launch/save_replay.sh) archives a run's bag.

Note that `CMakeLists.txt` installs the whole `launch/` directory, so these non-launch
files ship with the package too.
