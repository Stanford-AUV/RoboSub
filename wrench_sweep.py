#!/usr/bin/env python3
"""
wrench_sweep.py — open-loop, per-axis /wrench bench test.

Publishes a small wrench on ONE axis at a time so you can watch the sub's
PHYSICAL response and compare it to the expected (model) direction. Use it to
diagnose thruster sign/direction problems (e.g. the yaw-inversion bug): the
*pattern* of which axes come out reversed pinpoints the root cause.

Body frame (ROS REP-103): x forward, y left, z up  → right-handed.
So a POSITIVE command should physically do:
    +Fx  surge forward            +Mx  roll: port side rises / starboard dips
    +Fy  sway left (to port)      +My  pitch: nose down (top leans forward)
    +Fz  heave up                 +Mz  yaw LEFT / CCW-from-above (bow to port)

PREREQ — in a separate terminal bring up the actuation chain:
    conda activate robosub && source install/setup.bash
    ros2 launch main hardware.py     # thrust_generator + thrusters + arduino (+ imu/dvl)
  (Run ONLY hardware.py, not launch_sub.sh — the controller would fight for /wrench.)

Then run this:
    conda activate robosub && source install/setup.bash
    python wrench_sweep.py                       # + sweep of all 6 axes
    python wrench_sweep.py --both                # + and - each axis
    python wrench_sweep.py --axes Mz --both --interactive
    python wrench_sweep.py --axes Fx Fy Mz       # just the horizontal-thruster axes

SAFETY: out of water, keep holds short — do NOT run thrusters >10 s dry.
Ctrl-C aborts and sends a zero wrench.
"""
import argparse
import time

import rclpy
from std_msgs.msg import Header
from geometry_msgs.msg import WrenchStamped

AXES = ["Fx", "Fy", "Fz", "Mx", "My", "Mz"]
IDX = {a: i for i, a in enumerate(AXES)}
EXPECT = {
    "Fx": "SURGE forward (bow ahead)",
    "Fy": "SWAY left (toward port)",
    "Fz": "HEAVE up",
    "Mx": "ROLL: port side rises / starboard dips",
    "My": "PITCH: nose down (top leans forward)",
    "Mz": "YAW left / CCW from above (bow swings to port)",
}


def make_stamped(node, axis, val):
    """A WrenchStamped with `val` on the given axis, everything else zero."""
    comp = [0.0] * 6
    comp[IDX[axis]] = val
    msg = WrenchStamped()
    msg.header = Header()
    msg.header.stamp = node.get_clock().now().to_msg()
    msg.header.frame_id = "base_link"
    msg.wrench.force.x, msg.wrench.force.y, msg.wrench.force.z = comp[0], comp[1], comp[2]
    msg.wrench.torque.x, msg.wrench.torque.y, msg.wrench.torque.z = comp[3], comp[4], comp[5]
    return msg


def publish_for(node, pub, axis, val, seconds, rate_hz=20.0):
    """Publish the axis wrench continuously for `seconds` (ESCs need a steady stream)."""
    period = 1.0 / rate_hz
    ticks = max(1, int(seconds * rate_hz))
    for _ in range(ticks):
        pub.publish(make_stamped(node, axis, val))
        rclpy.spin_once(node, timeout_sec=0.0)
        time.sleep(period)


def main():
    ap = argparse.ArgumentParser(description="Open-loop per-axis /wrench bench sweep")
    ap.add_argument("--axes", nargs="+", default=AXES, metavar="AXIS",
                    help="subset of: Fx Fy Fz Mx My Mz (default: all)")
    ap.add_argument("--force", type=float, default=0.05, help="force magnitude N (default 0.05)")
    ap.add_argument("--torque", type=float, default=0.05, help="torque magnitude N·m (default 0.05)")
    ap.add_argument("--hold", type=float, default=3.0, help="seconds to hold each command (default 3)")
    ap.add_argument("--gap", type=float, default=2.0, help="seconds of zero between commands (default 2)")
    ap.add_argument("--both", action="store_true", help="test + and - for each axis")
    ap.add_argument("--interactive", action="store_true", help="wait for Enter before each command")
    args = ap.parse_args()

    for ax in args.axes:
        if ax not in AXES:
            raise SystemExit(f"unknown axis {ax!r}; choose from {AXES}")

    rclpy.init()
    node = rclpy.create_node("wrench_sweep")
    pub = node.create_publisher(WrenchStamped, "/wrench", 10)

    # let the publisher connect + start from a known zero
    publish_for(node, pub, "Fx", 0.0, 0.5)

    signs = [1, -1] if args.both else [1]
    try:
        print("=== /wrench sweep — frame x-fwd, y-left, z-up (right-handed) ===")
        print("Watch the SUB and compare to EXPECT. Note any axis that moves the")
        print("OPPOSITE way. Ctrl-C aborts (sends zero).\n")
        for axis in args.axes:
            mag = args.torque if axis.startswith("M") else args.force
            for s in signs:
                val = s * mag
                label = f"{'+' if s > 0 else '-'}{axis}"
                exp = EXPECT[axis] if s > 0 else f"OPPOSITE of ({EXPECT[axis]})"
                if args.interactive:
                    input(f"[next: {label} {val:+.3f}]  press Enter to command for {args.hold}s ...")
                print(f">>> {label} = {val:+.3f}  hold {args.hold}s   EXPECT: {exp}")
                publish_for(node, pub, axis, val, args.hold)
                print(f"    ... zero {args.gap}s")
                publish_for(node, pub, axis, 0.0, args.gap)
        print("\nDone. Root-cause guide (which axes were REVERSED):")
        print("  surge+sway+yaw reversed  -> horizontal thrusters reversed as a group (ESC/prop/wiring)")
        print("  sway+yaw+roll reversed   -> body-frame y-axis convention is mirrored in thrusters.yaml")
        print("  ONLY yaw reversed        -> horizontal thruster (x,y) positions are point-reflected in yaml")
    except KeyboardInterrupt:
        print("\nAborted — sending zero wrench.")
    finally:
        publish_for(node, pub, "Fx", 0.0, 0.5)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
