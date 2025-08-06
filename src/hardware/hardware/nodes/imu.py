#!/usr/bin/env python3
import numpy as np
from scipy.spatial.transform import Rotation as R

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from hardware.utils.average_quaternions import average_quaternions

NUM_CALIBRATION_SAMPLES = 200

class IMUZeroed(Node):
    def __init__(self):
        super().__init__('imu_zeroed')
        self.get_logger().info(f"Collecting {NUM_CALIBRATION_SAMPLES} samples for zero‐pose…")

        # 1) build fixed “raw→ENU” rotation:
        #    raw axes: +X=right, +Y=up, +Z=back
        #    ENU axes: +X=forward, +Y=left, +Z=up
        axis_map = {
            'x': ('z', -1),   # ENU X = –raw Z
            'y': ('x', -1),   # ENU Y = –raw X
            'z': ('y', +1),   # ENU Z =  raw Y
        }
        R_hw = np.zeros((3,3))
        for i, a in enumerate(('x','y','z')):
            src, sgn = axis_map[a]
            j = ('x','y','z').index(src)
            R_hw[i, j] = sgn
        self.q_hw = R.from_matrix(R_hw)

        # 2) buffer to average your “zero‐pose” ENU quaternions
        self._buf = np.zeros((NUM_CALIBRATION_SAMPLES, 4))
        self._count = 0
        self._q_ref_inv = None

        # subscribe raw → publish zeroed ENU
        self.create_subscription(Imu, '/imu/data', self._on_raw, 10)
        self._pub = self.create_publisher(Imu, 'imu', 10)

    def _on_raw(self, msg: Imu):
        # normalize raw quaternion [x,y,z,w]
        q_xyzw = np.array([
            msg.orientation.x,
            msg.orientation.y,
            msg.orientation.z,
            msg.orientation.w
        ], dtype=float)
        q_xyzw /= np.linalg.norm(q_xyzw)
        r_raw = R.from_quat(q_xyzw)

        # --- collect calibration samples (must conjugate to ENU) ---
        if self._count < NUM_CALIBRATION_SAMPLES:
            r_mapped = self.q_hw * r_raw * self.q_hw.inv()
            qm       = r_mapped.as_quat()       # [x,y,z,w]
            q_wxyz   = np.array([qm[3], qm[0], qm[1], qm[2]])
            q_wxyz  /= np.linalg.norm(q_wxyz)
            self._buf[self._count] = q_wxyz
            self._count += 1
            if self._count == NUM_CALIBRATION_SAMPLES:
                self._finish_calibration()
            return

        # --- after calibration, compute zeroed‐ENU outputs ---

        # 1) change‐of‐basis: conjugate raw→ENU
        r_mapped = self.q_hw * r_raw * self.q_hw.inv()
        # 2) zero‐pose delta: left‐multiply by q_ref_inv
        r_out = self._q_ref_inv * r_mapped
        q_out = r_out.as_quat()  # [x,y,z,w]

        # self.get_logger().info(f"Δ-quat = {q_out}")

        # now build the same composite for vector fields:
        #    first hw→ENU, then zero‐pose
        r_vec = self._q_ref_inv * self.q_hw

        # raw gyro & accel
        av_raw = np.array([
            msg.angular_velocity.x,
            msg.angular_velocity.y,
            msg.angular_velocity.z
        ], dtype=float)
        la_raw = np.array([
            msg.linear_acceleration.x,
            msg.linear_acceleration.y,
            msg.linear_acceleration.z
        ], dtype=float)

        # apply identical ENU + zero transforms
        av_out = self.q_hw.apply(av_raw)
        la_out = self.q_hw.apply(la_raw)

        # self.get_logger().info(f"lav-raw = {av_raw}")
        # self.get_logger().info(f"av-out = {av_out}")


        # publish the “zeroed” ENU IMU
        out = Imu()
        out.header = msg.header
        out.orientation.x, out.orientation.y, out.orientation.z, out.orientation.w = q_out
        out.angular_velocity.x, out.angular_velocity.y, out.angular_velocity.z = av_out
        out.linear_acceleration.x, out.linear_acceleration.y, out.linear_acceleration.z = la_out
        self._pub.publish(out)

    def _finish_calibration(self):
        # average_quaternions expects rows=[w,x,y,z]
        avg_w, avg_x, avg_y, avg_z = average_quaternions(self._buf)
        # convert to SciPy’s [x,y,z,w]
        q_ref = [avg_x, avg_y, avg_z, avg_w]
        r_ref = R.from_quat(q_ref)
        # invert so reference pose → identity
        self._q_ref_inv = r_ref.inv()
        self.get_logger().info(f"Zero-pose calibration done; q_ref = {q_ref}")

def main():
    rclpy.init()
    node = IMUZeroed()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__=='__main__':
    main()
