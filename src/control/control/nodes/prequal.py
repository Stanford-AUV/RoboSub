#!/usr/bin/env python3
"""
Prequal Node

Behavior:
1) DESCEND until reaching target depth (param: target_depth_m)
2) CRUISE forward (body-x) at full-send for a fixed duration (param: forward_duration_s)
3) STOP (publish zero wrench and remain idle)

Topics:
- Subscribes: /odometry/filtered (nav_msgs/Odometry)
- Publishes:  wrench (geometry_msgs/WrenchStamped)

Params (override in your launch/params yaml):
- odom_topic:               string,  default "/odometry/filtered"
- wrench_topic:             string,  default "wrench"
- control_rate_hz:          float,   default 20.0
- target_depth_m:           float,   default 1.0          # "positive down" convention assumed when depth_positive_down=True
- depth_tolerance_m:        float,   default 0.05
- depth_positive_down:      bool,    default True         # if False, we treat z-up as positive (depth = -z)
- descend_force_z:          float,   default -40.0        # N, sign depends on your vehicle convention (negative z = down)
- forward_force_x:          float,   default 60.0         # N, "full send" along +X
- forward_duration_s:       float,   default 7.5
- descend_timeout_s:        float,   default 25.0         # safety—start cruise even if depth not reached
"""

import time
import threading

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.qos import QoSProfile, QoSReliabilityPolicy

from nav_msgs.msg import Odometry
from geometry_msgs.msg import WrenchStamped


class Prequal(Node):
    def __init__(self):
        super().__init__("prequal")

        # -------- params --------
        self.declare_parameter("odom_topic", "/odometry/filtered")
        self.declare_parameter("wrench_topic", "wrench")
        self.declare_parameter("control_rate_hz", 20.0)

        self.declare_parameter("target_depth_m", 1.0)
        self.declare_parameter("depth_tolerance_m", 0.05)
        self.declare_parameter("depth_positive_down", True)

        self.declare_parameter("descend_force_z", -40.0)     # N (tune sign/mag for your vehicle)
        self.declare_parameter("forward_force_x", 60.0)       # N
        self.declare_parameter("forward_duration_s", 7.5)
        self.declare_parameter("descend_timeout_s", 25.0)

        self.odom_topic = self.get_parameter("odom_topic").get_parameter_value().string_value
        self.wrench_topic = self.get_parameter("wrench_topic").get_parameter_value().string_value
        self.rate_hz = float(self.get_parameter("control_rate_hz").value)

        self.target_depth = float(self.get_parameter("target_depth_m").value)
        self.depth_tol = float(self.get_parameter("depth_tolerance_m").value)
        self.depth_pos_down = bool(self.get_parameter("depth_positive_down").value)

        self.descend_force_z = float(self.get_parameter("descend_force_z").value)
        self.forward_force_x = float(self.get_parameter("forward_force_x").value)
        self.forward_duration_s = float(self.get_parameter("forward_duration_s").value)
        self.descend_timeout_s = float(self.get_parameter("descend_timeout_s").value)

        # -------- comms --------
        qos = QoSProfile(depth=5)
        qos.reliability = QoSReliabilityPolicy.RELIABLE

        self.odom_sub = self.create_subscription(Odometry, self.odom_topic, self._odom_cb, qos)
        self.wrench_pub = self.create_publisher(WrenchStamped, self.wrench_topic, 10)

        # -------- state --------
        self._lock = threading.Lock()
        self._last_odom = None
        self._state = "WAIT_ODOM"      # WAIT_ODOM -> DESCEND -> CRUISE -> STOPPED
        self._state_enter_time = self.get_clock().now()

        # control loop
        period = 1.0 / max(1e-6, self.rate_hz)
        self.timer = self.create_timer(period, self._tick)

        self.get_logger().info(
            f"Prequal up: odom={self.odom_topic}, out={self.wrench_topic}, "
            f"target_depth={self.target_depth:.2f}m ({'pos-down' if self.depth_pos_down else 'pos-up'}), "
            f"forward={self.forward_force_x:.1f}N for {self.forward_duration_s:.2f}s"
        )

    # ----------------- callbacks -----------------
    def _odom_cb(self, msg: Odometry):
        with self._lock:
            self._last_odom = msg
            if self._state == "WAIT_ODOM":
                self._transition("DESCEND")

    def _tick(self):
        # Guard shutdown
        if not rclpy.ok():
            return

        with self._lock:
            odom = self._last_odom
            state = self._state

        if state == "WAIT_ODOM":
            # publish zero while waiting
            self._publish_wrench(0.0, 0.0, 0.0)
            return

        if state == "DESCEND":
            depth = self._depth_from_odom(odom)
            elapsed = (self.get_clock().now() - self._state_enter_time).nanoseconds / 1e9
            reached = depth is not None and (depth >= (self.target_depth - self.depth_tol))
            timeout = elapsed >= self.descend_timeout_s

            if reached or timeout:
                if timeout and not reached:
                    self.get_logger().warn(
                        f"DESCEND timeout at depth={depth if depth is not None else 'unknown'} m; proceeding to CRUISE"
                    )
                self._transition("CRUISE")
                return

            # Command downward force
            self._publish_wrench(self.forward_force_x * 0.0, 0.0, self.descend_force_z)
            return

        if state == "CRUISE":
            elapsed = (self.get_clock().now() - self._state_enter_time).nanoseconds / 1e9
            if elapsed >= self.forward_duration_s:
                self._transition("STOPPED")
                return
            # Full send forward (body +X)
            self._publish_wrench(self.forward_force_x, 0.0, 0.0)
            return

        if state == "STOPPED":
            self._publish_wrench(0.0, 0.0, 0.0)
            return

    # ----------------- helpers -----------------
    def _depth_from_odom(self, odom: Odometry):
        if odom is None:
            return None
        z = float(odom.pose.pose.position.z)
        return z if self.depth_pos_down else -z  # convert to "positive down" depth

    def _publish_wrench(self, fx: float, fy: float, fz: float):
        msg = WrenchStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.wrench.force.x = float(fx)
        msg.wrench.force.y = float(fy)
        msg.wrench.force.z = float(fz)
        # torques left zero
        try:
            self.wrench_pub.publish(msg)
        except Exception:
            pass

    def _transition(self, new_state: str):
        self._state = new_state
        self._state_enter_time = self.get_clock().now()
        self.get_logger().info(f"STATE → {new_state}")
        # On entering STOPPED, publish one zero immediately
        if new_state == "STOPPED":
            self._publish_wrench(0.0, 0.0, 0.0)


def main(args=None):
    rclpy.init(args=args)
    node = Prequal()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Keyboard interrupt: stopping")
    finally:
        # ensure a zero wrench on exit
        node._publish_wrench(0.0, 0.0, 0.0)
        try:
            node.destroy_node()
        except Exception:
            pass
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
