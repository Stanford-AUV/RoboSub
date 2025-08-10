"""
Controller Node.

This file implements a ROS2 node that computes a control signal based on the
state of the system and a reference state. The node subscribes to the 'state'
and 'path' topics to receive the current state and reference state, and
publishes the computed control signal to the 'wrench' topic. The control signal
is computed using a given control policy, which is a PID controller in this
case.

Dependencies:
    threading
    control.utils.pid.PID
    control.utils.state.State
    geometry_msgs.msg.WrenchStamped
    nav_msgs.msg.Odometry
    numpy
    rclpy.node.Node

Authors:
    Ali Ahmad
    Khaled Messai

Version:
    1.1.0
"""

import sys
import select
import termios
import tty
import threading
from control.utils.pid import PID
from control.utils.state import State

from geometry_msgs.msg import WrenchStamped
from spatialmath.quaternion import UnitQuaternion
from tf_transformations import euler_from_quaternion

from nav_msgs.msg import Odometry

import numpy as np

import rclpy
from rclpy.node import Node
from rclpy import Parameter
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy

from msgs.msg import Waypoint

def _publish_stop(node: "Controller"):
    """Publish a single zero wrench and give DDS a brief moment to send it."""
    if node is None:
        return
    if not rclpy.ok() or not node.context.ok():
        return
    try:
        msg = WrenchStamped()  # zeros by default
        msg.header.stamp = node.get_clock().now().to_msg()
        node.control_publisher.publish(msg)
        # tiny flush so it goes out before the executor stops
        rclpy.spin_once(node, timeout_sec=0.05)
    except Exception:
        pass


def _keyboard_watch(node: "Controller"):
    """Background watcher: on 'q' publish stop and initiate shutdown."""
    if node is None:
        return
    if not sys.stdin.isatty():
        node.get_logger().warn("stdin is not a TTY; 'q' to quit is disabled")
        return

    fd = sys.stdin.fileno()
    orig_attr = termios.tcgetattr(fd)
    try:
        tty.setcbreak(fd)
        node.get_logger().info("Press 'q' to publish zero wrench and quit")
        while rclpy.ok() and node.context.ok() and node._alive:
            # poll stdin every 100ms without blocking the executor
            if select.select([sys.stdin], [], [], 0.1)[0]:
                ch = sys.stdin.read(1)
                if ch.lower() == 'q':
                    node.get_logger().info("q pressed: publishing zero wrench and shutting down")
                    _publish_stop(node)
                    node._alive = False
                    try:
                        rclpy.shutdown()
                    except Exception:
                        pass
                    break
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, orig_attr)


class Controller(Node):
    """
    Node that computes a control signal based on state and reference poses.
    """

    def __init__(self, policy):
        super().__init__("controller")

        self.get_logger().info("Starting controller")
        self._alive = True  # disallow publishes once we start shutting down

        self.declare_parameter("velocity_only", False)
        self.velocity_only = (
            self.get_parameter("velocity_only").get_parameter_value().bool_value
        )

        self.state_subscription = self.create_subscription(
            Odometry, "/odometry/filtered", self.state_callback, 10
        )

        qos = QoSProfile(depth=1)
        qos.durability = QoSDurabilityPolicy.TRANSIENT_LOCAL
        qos.reliability = QoSReliabilityPolicy.RELIABLE

        self.reference_subscription = self.create_subscription(
            Waypoint, 'waypoint', self.reference_callback, qos
        )
        self.control_publisher = self.create_publisher(WrenchStamped, "wrench", 10)

        self.lock = threading.Lock()

        self.time = self.get_clock().now()
        self.policy = policy
        self.ref_state = None
        self.count = 0

        self.follow_subject = None
        self.follow_sub = None
        self.latest_follow_target = None


    def reset(self):
        self.get_logger().info("Resetting controller...")
        with self.lock:
            self.policy.reset()

    def state_callback(self, msg: Odometry):
        if self.count >= 10:
            px = msg.pose.pose.position.x
            py = msg.pose.pose.position.y
            pz = msg.pose.pose.position.z
            if self.ref_state is not None:
                fx, fy, fz = self.ref_state.position
                self.get_logger().info(
                    f"Currently at {px:.3f}, {py:.3f}, {pz:.3f}, goal is {fx:.3f}, {fy:.3f}, {fz:.3f}"
                )
            else:
                self.get_logger().info(
                    f"Currently at {px:.3f}, {py:.3f}, {pz:.3f}, no goal yet"
                )

            q = msg.pose.pose.orientation
            roll, pitch, yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])
            if self.ref_state is not None:
                fq = self.ref_state.orientation
                froll, fpitch, fyaw = euler_from_quaternion([fq.v[0], fq.v[1], fq.v[2], fq.s])
                self.get_logger().info(
                    f"Orientation (r,p,y): {roll:.3f}, {pitch:.3f}, {yaw:.3f} | "
                    f"goal {froll:.3f}, {fpitch:.3f}, {fyaw:.3f}"
                )
            else:
                self.get_logger().info(
                    f"Orientation (r,p,y): {roll:.3f}, {pitch:.3f}, {yaw:.3f}, no goal yet"
                )
            self.count = 0
        else:
            self.count += 1

        cur_state = State.from_odometry_msg(msg)
        with self.lock:
            ref_state = self.ref_state.copy() if self.ref_state is not None else None
        if ref_state is not None:
            self.update(cur_state, ref_state)

    def _attach_follow(self, subject: str):
        topic = "/cam_lock/waypoint"
        qos = QoSProfile(depth=1)
        qos.durability = QoSDurabilityPolicy.TRANSIENT_LOCAL
        qos.reliability = QoSReliabilityPolicy.RELIABLE

        if self.follow_sub is not None:
            try:
                self.destroy_subscription(self.follow_sub)
            except Exception:
                pass
            self.follow_sub = None

        self.follow_subject = subject
        self.latest_follow_target = None
        # NOTE: Waypoint, not Odometry
        self.follow_sub = self.create_subscription(
            Waypoint, topic, self._follow_cb, qos
        )
        self.get_logger().info(f"FOLLOW: listening for subject '{subject}' at {topic}")


    def _follow_cb(self, wp: Waypoint):
        # Only take follow waypoints for the subject we’re tracking
        if (wp.purpose or "").lower() != "follow":
            return
        if (self.follow_subject or "").lower() != (wp.subject or "").lower():
            return

        with self.lock:
            self.ref_state = State.from_odometry_msg(wp.target)  # use embedded Odometry
            self.latest_follow_target = wp.target


    def reference_callback(self, msg: Waypoint):
        purpose = (msg.purpose or "").lower()

        match purpose:
            case "target":
                self.get_logger().info("Received TARGET waypoint update")
                with self.lock:
                    self.ref_state = State.from_odometry_msg(msg.target)
                self.get_logger().info("Reference state updated (TARGET)")

            case "follow":
                subject = (msg.subject or "").strip()
                if not subject:
                    self.get_logger().warn("FOLLOW waypoint missing 'subject' — ignoring")
                    return
                self.get_logger().info(f"Received FOLLOW waypoint for subject '{subject}'")
                self._attach_follow(subject)

            case "stop" | "abort":
                self.get_logger().info(f"Received {purpose.upper()} waypoint — ignoring updates for now")
                # TODO: publish zero wrench or set a stop flag

            case _:
                self.get_logger().info(f"Ignoring waypoint with unknown purpose '{msg.purpose}'")


    def update(self, cur_state, ref_state):
        newTime = self.get_clock().now()
        dt = (newTime - self.time).nanoseconds / 1e9
        self.time = newTime

        wrench = self.policy.update(cur_state, ref_state, dt, self.velocity_only)
        wrench.header.stamp = self.time.to_msg()

        if self.count >= 9:
            self.get_logger().info(f"Publishing wrench {wrench}")

        # Guard publishes once shutdown has begun
        if not self._alive or not self.context.ok() or not rclpy.ok():
            return
        try:
            self.control_publisher.publish(wrench)
        except Exception:
            pass


def main(args=None):
    rclpy.init(args=args)

    pid = PID(
        kP_position=np.array([1.5, 1.5, 1.5]),
        kD_position=np.array([0.05, 0.05, 0.05]),
        kI_position=np.array([0.2, 0.2, 0.2]),
        kP_orientation=np.array([0.05, 0.05, 0.025]),
        kD_orientation=np.array([0.05, 0.05, 0.025]),
        kI_orientation=np.array([0, 0, 0]),
        max_integral_position=np.array([1, 1, 1]),
        max_integral_orientation=np.array([1, 1, 1]),
    )

    node = Controller(pid)

    # Start background keyboard watcher (uses only stdlib)
    watcher = threading.Thread(target=_keyboard_watch, args=(node,), daemon=True)
    watcher.start()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Ctrl-C: publishing zero wrench and shutting down")
        _publish_stop(node)
    finally:
        node._alive = False
        try:
            node.destroy_node()
        except Exception:
            pass
        if rclpy.ok():
            try:
                rclpy.shutdown()
            except RuntimeError:
                pass


if __name__ == "__main__":
    main()