# detections3d_udp.py
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from vision_msgs.msg import Detection3DArray
import socket, json

class Detections3DUdp(Node):
    def __init__(self):
        super().__init__("detections3d_udp")

        # ---- params ----
        self.declare_parameter("topic", "detections3d")
        self.declare_parameter("udp_enable", True)
        self.declare_parameter("udp_host", "10.13.37.1")     # <-- set to your Mac IP
        self.declare_parameter("udp_port", 10000)            # <-- different from 9999
        self.declare_parameter("newline_delimited", True)    # JSONL for easy nc

        self.topic   = self.get_parameter("topic").get_parameter_value().string_value
        self.enabled = self.get_parameter("udp_enable").get_parameter_value().bool_value
        self.host    = self.get_parameter("udp_host").get_parameter_value().string_value
        self.port    = int(self.get_parameter("udp_port").get_parameter_value().integer_value)
        self.jsonl   = self.get_parameter("newline_delimited").get_parameter_value().bool_value

        # ---- UDP ----
        self._sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._dst  = (self.host, self.port)

        # ---- sub ----
        qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10,
        )
        self.sub = self.create_subscription(Detection3DArray, self.topic, self.cb, qos)
        self.get_logger().info(f"UDP bridge ON -> {self.host}:{self.port}, topic='{self.topic}'")

    def cb(self, msg: Detection3DArray):
        if not self.enabled or not msg.detections:
            return
        t = self.get_clock().now().nanoseconds / 1e9

        # Send one UDP packet per detection (JSON line)
        for i, det in enumerate(msg.detections):
            self.get_logger().info("good udp")
            label = ""
            score = 0.0
            if det.results:
                # class_id is a string in your publisher; score ∈ [0,1]
                label = det.results[0].hypothesis.class_id
                score = float(det.results[0].hypothesis.score)

            c = det.bbox.center.position
            s = det.bbox.size

            out = {
                "t": t, "idx": i,
                "label": label, "score": score,
                "center": {"x": float(c.x), "y": float(c.y), "z": float(c.z)},  # meters
                "size":   {"x": float(s.x), "y": float(s.y), "z": float(s.z)},  # meters
            }
            payload = json.dumps(out).encode("utf-8")
            if self.jsonl:
                payload += b"\n"
            try:
                self._sock.sendto(payload, self._dst)
            except Exception as e:
                self.get_logger().warn(f"UDP send failed: {e}")

def main():
    rclpy.init()
    node = Detections3DUdp()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.destroy_node()
        finally:
            rclpy.shutdown()

if __name__ == "__main__":
    main()
