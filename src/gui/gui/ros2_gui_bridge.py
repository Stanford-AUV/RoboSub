#!/usr/bin/env python3
"""
ros2_gui_bridge.py — Stanford AUV HUD WebSocket Bridge
=======================================================
Runs as a ROS 2 node that:
  • SUBscribes to /imu, /odometry/filtered, /thrust/wrench (or /cmd_vel)
  • PUBlishes to /cmd_vel (geometry_msgs/Twist) for joystick override
  • Exposes a WebSocket server on ws://localhost:9090 that the HTML GUI
    connects to.  Messages are newline-delimited JSON.

Install once (inside the Docker container):
    pip install websockets --break-system-packages

Run (after building & sourcing):
    python3 ros2_gui_bridge.py

The GUI auto-reconnects if this node restarts.
"""

import atexit
import asyncio
import json
import math
import os
import signal
import subprocess
import threading
import time

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist, TwistWithCovarianceStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from std_msgs.msg import Bool

try:
    import websockets
    from websockets.server import serve as ws_serve
except ImportError:
    import os, glob as _glob, sys as _sys
    _venv = os.environ.get("VIRTUAL_ENV")
    if _venv:
        _sp = _glob.glob(f"{_venv}/lib/python*/site-packages")
        if _sp:
            _sys.path.insert(0, _sp[0])
    try:
        import websockets
        from websockets.server import serve as ws_serve
    except ImportError:
        raise SystemExit("websockets not found in system or active venv. Run: pip install websockets")

# ── config ─────────────────────────────────────────────────────────────────
WS_HOST = "0.0.0.0"
WS_PORT = 9091

LAUNCH_SCRIPT = os.path.join(os.path.dirname(__file__), "..", "scripts", "launch_sub.sh")
START_HARDWARE_SCRIPT = os.path.join(os.path.dirname(__file__), "..", "scripts", "start_hardware.sh")

# Topics (edit to match your actual remapped topic names)
TOPIC_IMU      = "/imu/data"          # sensor_msgs/Imu
TOPIC_ODOM     = "/odometry/filtered" # nav_msgs/Odometry
TOPIC_CMD_VEL  = "/cmd_vel"           # geometry_msgs/Twist  (publish)
TOPIC_ESTOP    = "/estop"             # std_msgs/Bool        (publish)
TOPIC_VELOCITY = "/velocity"

# ── helpers ────────────────────────────────────────────────────────────────

def quat_to_euler(qx, qy, qz, qw):
    """Return (roll, pitch, yaw) in degrees from a quaternion."""
    sinr_cosp = 2.0 * (qw * qx + qy * qz)
    cosr_cosp = 1.0 - 2.0 * (qx * qx + qy * qy)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    sinp = 2.0 * (qw * qy - qz * qx)
    sinp = max(-1.0, min(1.0, sinp))
    pitch = math.asin(sinp)

    siny_cosp = 2.0 * (qw * qz + qx * qy)
    cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
    yaw = math.atan2(siny_cosp, cosy_cosp)

    return (
        round(math.degrees(roll),  2),
        round(math.degrees(pitch), 2),
        round(math.degrees(yaw),   2),
    )


# ── shared state (thread-safe via lock) ────────────────────────────────────

_lock = threading.Lock()
_state = {
    # position (metres)
    "x": 0.0, "y": 0.0, "z": 0.0,
    # orientation (degrees)
    "roll": 0.0, "pitch": 0.0, "yaw": 0.0,
    # velocities
    "vx": 0.0, "vy": 0.0, "vz": 0.0,
    # angular velocities (deg/s)
    "wx": 0.0, "wy": 0.0, "wz": 0.0,
    # linear acceleration (m/s²)
    "ax": 0.0, "ay": 0.0, "az": 0.0,
    # meta
    "imu_ok":  False,
    "odom_ok": False,
    "stamp": 0.0,
}

_connected_ws: set = set()
_cmd_pub = None   # set after node is ready
_estop_pub = None
_ws_loop = None   # asyncio event loop, set in _ws_thread
_procs: dict = {}  # source -> subprocess.Popen


def _kill_proc(source: str):
    proc = _procs.get(source)
    if proc and proc.poll() is None:
        try:
            os.killpg(os.getpgid(proc.pid), signal.SIGTERM)
        except Exception:
            pass
    _procs.pop(source, None)


def _kill_all_procs():
    for source in list(_procs.keys()):
        _kill_proc(source)


atexit.register(_kill_all_procs)


def _sigterm_handler(signum, frame):
    _kill_all_procs()
    raise SystemExit(0)


signal.signal(signal.SIGTERM, _sigterm_handler)


def _broadcast_log(source: str, text: str):
    payload = json.dumps({"type": "log", "source": source, "text": text})
    for ws in list(_connected_ws):
        try:
            asyncio.run_coroutine_threadsafe(ws.send(payload), _ws_loop)
        except Exception:
            pass


def _stream_script(script: str, source: str):
    """Kill any existing process for source, then start fresh and stream output."""
    _kill_proc(source)
    def _run():
        try:
            proc = subprocess.Popen(
                ["bash", script],
                stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                text=True, start_new_session=True,
            )
            _procs[source] = proc
            for line in proc.stdout:
                _broadcast_log(source, line.rstrip())
            proc.wait()
            _broadcast_log(source, f"[process exited with code {proc.returncode}]")
            _procs.pop(source, None)
        except Exception as e:
            _broadcast_log(source, f"[error launching script: {e}]")
    threading.Thread(target=_run, daemon=True).start()


def _update_state(**kwargs):
    with _lock:
        _state.update(kwargs)
        _state["stamp"] = time.time()


def _get_state_snapshot():
    with _lock:
        return dict(_state)


# ── ROS 2 node ─────────────────────────────────────────────────────────────

class HudBridgeNode(Node):
    def __init__(self):
        super().__init__("hud_bridge")
        self.get_logger().info("HUD bridge node starting…")

        self.create_subscription(Imu,      TOPIC_IMU,   self._cb_imu,  10)
        self.create_subscription(Odometry, TOPIC_ODOM,  self._cb_odom, 10)
        self.create_subscription(TwistWithCovarianceStamped, TOPIC_VELOCITY, self._cb_velocity, 10)

        global _cmd_pub, _estop_pub
        _cmd_pub   = self.create_publisher(Twist, TOPIC_CMD_VEL, 10)
        _estop_pub = self.create_publisher(Bool,  TOPIC_ESTOP,   10)

        self.get_logger().info(
            f"Subscribed: {TOPIC_IMU}, {TOPIC_ODOM} | "
            f"Publishing: {TOPIC_CMD_VEL}, {TOPIC_ESTOP}"
        )

    # ── subscribers ──────────────────────────────────────────────────────

    def _cb_imu(self, msg: Imu):
        qx = msg.orientation.x
        qy = msg.orientation.y
        qz = msg.orientation.z
        qw = msg.orientation.w

        roll, pitch, yaw = quat_to_euler(qx, qy, qz, qw)

        _update_state(
            roll=roll, pitch=pitch, yaw=yaw,
            wx=round(math.degrees(msg.angular_velocity.x),    2),
            wy=round(math.degrees(msg.angular_velocity.y),    2),
            wz=round(math.degrees(msg.angular_velocity.z),    2),
            ax=round(msg.linear_acceleration.x, 3),
            ay=round(msg.linear_acceleration.y, 3),
            az=round(msg.linear_acceleration.z, 3),
            imu_ok=True,
        )

    def _cb_odom(self, msg: Odometry):
        p = msg.pose.pose.position
        v = msg.twist.twist.linear

        _update_state(
            x=round(p.x, 3),
            y=round(p.y, 3),
            z=round(p.z, 3),
            vx=round(v.x, 3),
            vy=round(v.y, 3),
            vz=round(v.z, 3),
            odom_ok=True,
        )

    def _cb_velocity(self, msg: TwistWithCovarianceStamped):
        v = msg.twist.twist.linear
        _update_state(
            vx=round(v.x, 3),
            vy=round(v.y, 3),
            vz=round(v.z, 3),
        )

# ── WebSocket server ────────────────────────────────────────────────────────

async def _ws_handler(websocket):
    """One coroutine per connected client."""
    _connected_ws.add(websocket)
    client = websocket.remote_address
    print(f"[WS] client connected: {client}")
    try:
        # Send current state immediately on connect
        await websocket.send(json.dumps({"type": "state", **_get_state_snapshot()}))

        async for raw in websocket:
            try:
                msg = json.loads(raw)
            except json.JSONDecodeError:
                continue

            mtype = msg.get("type", "")

            # ── joystick / manual cmd_vel ──────────────────────────────
            if mtype == "cmd_vel" and _cmd_pub is not None:
                twist = Twist()
                twist.linear.x  = float(msg.get("lx", 0.0))
                twist.linear.y  = float(msg.get("ly", 0.0))
                twist.linear.z  = float(msg.get("lz", 0.0))
                twist.angular.x = float(msg.get("ax", 0.0))
                twist.angular.y = float(msg.get("ay", 0.0))
                twist.angular.z = float(msg.get("az", 0.0))
                _cmd_pub.publish(twist)

            # ── launch ───────────────────────────────────────────────
            elif mtype == "launch":
                script = os.path.realpath(LAUNCH_SCRIPT)
                print(f"[LAUNCH] running {script}")
                _stream_script(script, "launch")

            # ── start hardware ────────────────────────────────────────
            elif mtype == "start_hardware":
                script = os.path.realpath(START_HARDWARE_SCRIPT)
                print(f"[HARDWARE] running {script}")
                _stream_script(script, "hardware")

            # ── kill individual or all spawned processes ──────────────
            elif mtype == "kill":
                _kill_proc(msg.get("source", ""))
            elif mtype == "kill_all":
                _kill_all_procs()

            # ── e-stop ────────────────────────────────────────────────
            elif mtype == "estop" and _estop_pub is not None:
                b = Bool()
                b.data = bool(msg.get("active", True))
                _estop_pub.publish(b)

    except Exception as e:
        print(f"[WS] client {client} disconnected: {e}")
    finally:
        _connected_ws.discard(websocket)


async def _broadcast_loop():
    """Push latest state to all connected clients at ~20 Hz."""
    while True:
        await asyncio.sleep(0.05)
        if not _connected_ws:
            continue
        snap = _get_state_snapshot()
        payload = json.dumps({"type": "state", **snap})
        dead = set()
        for ws in list(_connected_ws):
            try:
                await ws.send(payload)
            except Exception:
                dead.add(ws)
        _connected_ws.difference_update(dead)


async def _ws_main():
    async with ws_serve(_ws_handler, WS_HOST, WS_PORT):
        print(f"[WS] server listening on ws://{WS_HOST}:{WS_PORT}")
        await _broadcast_loop()  # runs forever


def _ws_thread():
    global _ws_loop
    _ws_loop = asyncio.new_event_loop()
    asyncio.set_event_loop(_ws_loop)
    _ws_loop.run_until_complete(_ws_main())


# ── entry point ─────────────────────────────────────────────────────────────

def main():
    # Start WebSocket server in a background thread
    t = threading.Thread(target=_ws_thread, daemon=True)
    t.start()

    # Start ROS 2
    rclpy.init()
    node = HudBridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

