#!/usr/bin/env python3
"""Publish a constant wrench on /wrench for a fixed duration, then zero it.

Goes through the real pipeline: /wrench -> thrust_generator -> /thrusts ->
thrusters -> pwms -> arduino -> ESCs. On completion OR Ctrl+C a zero wrench is
published repeatedly so the thrusters always come back to neutral.

Examples (forces in N, torques in N*m at body frame):
    python send_thrust.py --fx 60 --duration 3        # hard surge forward
    python send_thrust.py --tz 20 --duration 2        # hard yaw spin
    python send_thrust.py --fz -40 --ty 10            # dive + pitch
"""

import argparse
import time

import rclpy
from geometry_msgs.msg import WrenchStamped
from rclpy.node import Node
from tqdm import tqdm


def make_msg(node, fx, fy, fz, tx, ty, tz):
    msg = WrenchStamped()
    msg.header.stamp = node.get_clock().now().to_msg()
    msg.header.frame_id = "base_link"
    msg.wrench.force.x = float(fx)
    msg.wrench.force.y = float(fy)
    msg.wrench.force.z = float(fz)
    msg.wrench.torque.x = float(tx)
    msg.wrench.torque.y = float(ty)
    msg.wrench.torque.z = float(tz)
    return msg


def main():
    ap = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    for name, hlp in [("fx", "surge force [N]"), ("fy", "sway force [N]"),
                      ("fz", "heave force [N]"), ("tx", "roll torque [N*m]"),
                      ("ty", "pitch torque [N*m]"), ("tz", "yaw torque [N*m]")]:
        ap.add_argument(f"--{name}", type=float, default=0.0, help=hlp)
    ap.add_argument("--duration", type=float, default=3.0,
                    help="seconds to hold the wrench (default 3)")
    ap.add_argument("--rate", type=float, default=20.0, help="publish rate [Hz]")
    args = ap.parse_args()

    rclpy.init()
    node = Node("thrust_experiment_cmd")
    pub = node.create_publisher(WrenchStamped, "/wrench", 10)
    time.sleep(0.5)  # let the publisher match with thrust_generator

    vec = (args.fx, args.fy, args.fz, args.tx, args.ty, args.tz)
    period = 1.0 / args.rate
    ticks = max(1, int(args.duration * args.rate))
    print(f"Wrench F=({vec[0]}, {vec[1]}, {vec[2]}) N  "
          f"T=({vec[3]}, {vec[4]}, {vec[5]}) N*m for {args.duration}s")

    try:
        for _ in tqdm(range(ticks), desc="thrusting", unit="msg"):
            pub.publish(make_msg(node, *vec))
            time.sleep(period)
    except KeyboardInterrupt:
        print("\nInterrupted -- zeroing wrench.")
    finally:
        # Zero repeatedly so thrust_generator (which latches the last wrench)
        # reliably drives everything back to neutral.
        for _ in range(15):
            pub.publish(make_msg(node, 0, 0, 0, 0, 0, 0))
            time.sleep(0.03)
        print("Wrench zeroed.")
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
