#!/usr/bin/env python3
import time
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int16
from vision_msgs.msg import Detection3DArray

class SharkDropper(Node):
    """
    Listens to bottom camera detections and, upon reliably detecting the target label
    (default: "shark"), publishes a single command to the 'dropper' topic (Int16).

    Parameters:
      detections_topic (str):    input detections topic (default: 'bottom_cam/detections3d')
      target_label (str):        class label to trigger on (default: 'shark')
      score_threshold (float):   min confidence to count a detection (default: 0.6)
      required_streak (int):     consecutive frames required before firing (default: 3)
      dropper_topic (str):       output command topic (default: 'dropper')
      dropper_value (int):       Int16 value to publish when firing (default: 1)
      rearm_on_absence_sec (float): seconds without target to re-arm (default: 5.0)
    """

    def __init__(self):
        super().__init__('shark_dropper')

        # Params
        self.declare_parameter('detections_topic', 'bottom_cam/detections3d')
        self.declare_parameter('target_label', 'bin_shark')
        self.declare_parameter('score_threshold', 0.5)
        self.declare_parameter('required_streak', 3)
        self.declare_parameter('dropper_topic', 'dropper')
        self.declare_parameter('dropper_value', 1)
        self.declare_parameter('rearm_on_absence_sec', 5.0)

        self.detections_topic   = self.get_parameter('detections_topic').get_parameter_value().string_value
        self.target_label       = self.get_parameter('target_label').get_parameter_value().string_value
        self.score_threshold    = float(self.get_parameter('score_threshold').value)
        self.required_streak    = int(self.get_parameter('required_streak').value)
        self.dropper_topic      = self.get_parameter('dropper_topic').get_parameter_value().string_value
        self.dropper_value      = int(self.get_parameter('dropper_value').value)
        self.rearm_on_absence_s = float(self.get_parameter('rearm_on_absence_sec').value)

        # State
        self.streak = 0
        self.already_dropped = False
        self.last_seen_time = 0.0

        # IO
        self.dropper_pub = self.create_publisher(Int16, self.dropper_topic, 10)
        self.create_subscription(Detection3DArray, self.detections_topic, self._det_cb, 10)

        self.get_logger().info(
            f"SharkDropper listening on '{self.detections_topic}' "
            f"target_label='{self.target_label}', score>={self.score_threshold}, "
            f"streak={self.required_streak} -> publish {self.dropper_value} to '{self.dropper_topic}'"
        )

        # Timer to handle re-arming after absence
        self.create_timer(0.5, self._rearm_check)

    def _rearm_check(self):
        if self.already_dropped and (time.monotonic() - self.last_seen_time) >= self.rearm_on_absence_s:
            # Target has been absent long enough; allow re-trigger if desired
            self.already_dropped = False
            self.streak = 0
            self.get_logger().info("Re-armed dropper (target absent long enough).")

    def _det_cb(self, msg: Detection3DArray):
        # Look for target label with sufficient confidence
        seen_this_frame = False
        for det in msg.detections:
            if not det.results:
                continue
            hyp = det.results[0].hypothesis
            label = getattr(hyp, "class_id", "") or ""   # we set class_id as string label upstream
            score = float(getattr(hyp, "score", 0.0))
            if label == self.target_label and score >= self.score_threshold:
                seen_this_frame = True
                break

        # Update streak / timing
        if seen_this_frame:
            self.last_seen_time = time.monotonic()
            self.streak += 1
        else:
            # reset streak on a miss
            self.streak = 0

        # Fire once when conditions met
        if (not self.already_dropped) and seen_this_frame and (self.streak >= self.required_streak):
            self.get_logger().info(
                f"Detected '{self.target_label}' {self.streak}x consecutively (>= {self.required_streak}). "
                f"Sending dropper command ({self.dropper_value})."
            )
            self.dropper_pub.publish(Int16(data=self.dropper_value))
            self.already_dropped = True
            # keep last_seen_time to allow eventual re-arming if the target disappears

def main(args=None):
    rclpy.init(args=args)
    node = SharkDropper()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
