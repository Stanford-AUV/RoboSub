# `gui`

**Build type:** `ament_python` · **Depends on:** `geometry_msgs`, `std_msgs`, `rclpy`
**Non-ROS deps:** `websockets`

A browser HUD for the sub. One ROS node bridges the ROS graph to a WebSocket, and a
self-contained HTML page renders it. Nothing in the autonomy stack depends on this
package — it is read-mostly instrumentation plus a manual override path.

> There are two HUD/dashboard surfaces in the repo. This one is the WebSocket HUD. The
> dashboard you see on a normal run is `hardware/sensors_plot`'s HTTP dashboard on port
> **8080** — see [`../hardware`](../hardware/README.md).

---

## Entry points

Registered in [`setup.py`](setup.py):

| Executable | Module | Node name |
|---|---|---|
| `bridge` | `gui/ros2_gui_bridge.py` | `hud_bridge` |

```bash
ros2 run gui bridge
```

---

## `bridge` (`hud_bridge`)

| Direction | Topic | Type |
|---|---|---|
| Subscribes | `/imu/data` | `sensor_msgs/Imu` |
| Subscribes | `/odometry/filtered` | `nav_msgs/Odometry` |
| Subscribes | `/velocity` | `geometry_msgs/TwistWithCovarianceStamped` |
| Publishes | `/cmd_vel` | `geometry_msgs/Twist` |
| Publishes | `/estop` | `std_msgs/Bool` |

Topic names are constants at the top of `gui/ros2_gui_bridge.py`.

**WebSocket server:** `0.0.0.0:9091`, newline-delimited JSON, auto-reconnecting client.
The node can also shell out to `gui/scripts/launch_sub.sh` and
`gui/scripts/start_hardware.sh` so the page can bring the stack up.

> `/cmd_vel` and `/estop` have **no subscriber** in this workspace — nothing in `control`
> or `hardware` consumes them. The override buttons are wired on the browser side only
> until a consumer is added.
>
> `/imu/data` is the raw Xsens driver topic. The topic the EKF fuses is `/imu/orientation`
> (published by `hardware/imu`).

---

## Running the HUD

```bash
# on the sub
ros2 run gui bridge                         # WebSocket on :9091

# serve the page (any static server)
cd src/gui && python3 -m http.server 8080
```

Then forward both ports to your laptop (VS Code **Ports** panel, or
`ssh -L 8080:localhost:8080 -L 9091:localhost:9091 robosub`) and open
<http://localhost:8080/auv_hud.html>.

## Files

| Path | Role |
|---|---|
| `gui/ros2_gui_bridge.py` | the installed node (`ros2 run gui bridge`) |
| `ros2_gui_bridge.py` (package root) | older copy, no `/velocity` subscription — **not** installed; prefer the one under `gui/` |
| `auv_hud.html` | the HUD page; WebSocket URL defaults to `ws://localhost:9091` |
| `scripts/launch_sub.sh`, `scripts/start_hardware.sh` | shell hooks the bridge can invoke |

**TODO:** move the hardcoded panel layout in `auv_hud.html` into YAML.
