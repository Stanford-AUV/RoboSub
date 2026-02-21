import copy
import rclpy
from sensor_msgs.msg import Imu
import numpy as np
from scipy.spatial.transform import Rotation as R
from hardware.utils.average_quaternions import average_quaternions
from hardware.nodes.generic_sensor import GenericSensor

NUM_CALIBRATION_QUATERNIONS = 200

COVARIANCE_IGNORE = [-1.0] + [0.0] * 8
DISABLE_IMU_TRANSFORM = False

class IMU(GenericSensor):
    def __init__(self):
        super().__init__("imu", "imu_0")

        self._publishers = {
            "rotation": self.create_publisher(Imu, "/rotation", 10),
            "angular": self.create_publisher(Imu, "/angular", 10),
            "accel": self.create_publisher(Imu, "/accel", 10),
        }
        self._imu_pub = self.create_publisher(Imu, "/imu/data/corrected", 10)

        self.imu_subscription = self.create_subscription(
            Imu, "/imu/data", self.imu_listener_callback, 10
        )

        self.R_imu_to_base = None

        self.num_calibration_quaternions = 0
        self.calibration_quaternions = np.zeros((NUM_CALIBRATION_QUATERNIONS, 4))
        self.calibrated = False

        self._latest_msg = None

    def imu_listener_callback(self, msg: Imu):
        # ---- PASSTHROUGH MODE ----
        if DISABLE_IMU_TRANSFORM:
            out = copy.deepcopy(msg)  # avoid aliasing / accidental mutation


            GYRO_BIAS = np.array([
                -0.006099891562248375,
                -0.0034746338098969654,
                0.00273246756079096
            ], dtype=float)

            w = np.array([out.angular_velocity.x, out.angular_velocity.y, out.angular_velocity.z], dtype=float)
            w = w - GYRO_BIAS
            out.angular_velocity.x, out.angular_velocity.y, out.angular_velocity.z = w.tolist()
            # Optional but recommended: enforce the frame you're using in TF/EKF
            # If your pipeline expects imu_link, set it here.
            if not out.header.frame_id:
                out.header.frame_id = "imu_link"   # or "base_link" if that's your convention

            # Defensive: drop obviously invalid quaternions (prevents EKF NaNs)
            q = np.array([out.orientation.x, out.orientation.y,
                        out.orientation.z, out.orientation.w], dtype=float)
            if (not np.all(np.isfinite(q))) or (np.linalg.norm(q) < 1e-6):
                self.get_logger().warn("Passthrough: invalid IMU quaternion; dropping msg")
                return

            # Normalize (cheap insurance)
            q /= np.linalg.norm(q)
            out.orientation.x, out.orientation.y, out.orientation.z, out.orientation.w = q.tolist()

            self._imu_pub.publish(out)
            return


        # ---- NORMAL MODE (your existing logic) ----
        if not self.calibrated:
            if self.num_calibration_quaternions == NUM_CALIBRATION_QUATERNIONS:
                self._calculate_transformation_matrix()
                self.calibrated = True
            else:
                q = np.array([
                    msg.orientation.x,
                    msg.orientation.y,
                    msg.orientation.z,
                    msg.orientation.w,
                ])
                q_matrix = R.from_quat(q).as_matrix()
                transformed_q_matrix = self.R_sensor_to_base @ q_matrix @ self.R_sensor_to_base.T
                transformed_q = R.from_matrix(transformed_q_matrix).as_quat()
                transformed_q /= np.linalg.norm(transformed_q)

                self.calibration_quaternions[self.num_calibration_quaternions] = transformed_q
                self.num_calibration_quaternions += 1
                return

        self._latest_msg = msg
        self.publish_sensor_data()

    def _calculate_transformation_matrix(self):
        avg_quat = average_quaternions(self.calibration_quaternions)
        r_imu = R.from_quat(avg_quat)
        r_base = R.from_quat([0, 0, 0, 1])
        self.r_imu_to_base = r_base * r_imu.inv()
        self.R_imu_to_base = self.r_imu_to_base.as_matrix()
        self.get_logger().info(f"Calibrated with:\n{self.R_imu_to_base}")

    def _transform_orientation(self, msg):
        q = np.array(
            [msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w]
        )
        q_matrix = R.from_quat(q).as_matrix()
        transformed_q_matrix = (
            self.R_imu_to_base @ self.R_sensor_to_base @ q_matrix @ self.R_sensor_to_base.T
        )
        transformed_q = R.from_matrix(transformed_q_matrix).as_quat()
        transformed_q /= np.linalg.norm(transformed_q)
        return transformed_q

    def _transform_angular_velocity(self, msg):
        angular_velocity = np.array(
            [msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z]
        )
        return self.R_sensor_to_base @ angular_velocity

    def _transform_linear_acceleration(self, msg):
        linear_acceleration = np.array(
            [msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z]
        )
        return self.R_sensor_to_base @ linear_acceleration

    def _cov3x3(self):
        """Flatten 3×3 covariance matrix from sensors.yaml into 9-element list."""
        if self.covariance is not None:
            return [float(v) for row in self.covariance for v in row]
        return [0.0] * 9

    def publish_sensor_data(self):
        msg = self._latest_msg
        if msg is None:
            return

        stamp = self.get_clock().now().to_msg()

        if self.is_active("rotation"):
            imu_rot = Imu()
            imu_rot.header.stamp = stamp
            imu_rot.header.frame_id = "base_link"

            q = self._transform_orientation(msg)
            imu_rot.orientation.x = q[0]
            imu_rot.orientation.y = q[1]
            imu_rot.orientation.z = q[2]
            imu_rot.orientation.w = q[3]

            imu_rot.orientation_covariance = self._cov3x3()
            imu_rot.angular_velocity_covariance = COVARIANCE_IGNORE
            imu_rot.linear_acceleration_covariance = COVARIANCE_IGNORE

            self._publishers["rotation"].publish(imu_rot)

        if self.is_active("angular"):
            imu_ang = Imu()
            imu_ang.header.stamp = stamp
            imu_ang.header.frame_id = "base_link"

            av = self._transform_angular_velocity(msg)
            imu_ang.angular_velocity.x = av[0]
            imu_ang.angular_velocity.y = av[1]
            imu_ang.angular_velocity.z = av[2]

            imu_ang.orientation_covariance = COVARIANCE_IGNORE
            imu_ang.angular_velocity_covariance = self._cov3x3()
            imu_ang.linear_acceleration_covariance = COVARIANCE_IGNORE

            self._publishers["angular"].publish(imu_ang)

        if self.is_active("accel"):
            imu_acc = Imu()
            imu_acc.header.stamp = stamp
            imu_acc.header.frame_id = "base_link"

            la = self._transform_linear_acceleration(msg)
            imu_acc.linear_acceleration.x = la[0]
            imu_acc.linear_acceleration.y = la[1]
            imu_acc.linear_acceleration.z = la[2]

            imu_acc.orientation_covariance = COVARIANCE_IGNORE
            imu_acc.angular_velocity_covariance = COVARIANCE_IGNORE
            imu_acc.linear_acceleration_covariance = self._cov3x3()

            self._publishers["accel"].publish(imu_acc)


def main(args=None):
    rclpy.init(args=args)
    imu = IMU()
    rclpy.spin(imu)
    imu.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
