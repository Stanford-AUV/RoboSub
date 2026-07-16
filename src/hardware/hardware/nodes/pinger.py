"""Pinger task-order publisher.

Runs the EE team's Daisy-Seed `ping` detection port (hardware/utils/tdoa,
copied verbatim from references/robosub-ee/research/tdoa) against the live
96 kHz hydrophone stream and turns its front/back verdict into the semifinals
task order on /pinger/task, which planning's branch item consumes.

Loop: record `window_sec` of audio from BOTH stream_audio Daisy boards ->
run_ping -> 'front'/'back' verdict -> map to a task string -> latch it. A
separate timer republishes the latched task at `publish_hz` so the branch
always sees the latest decision whenever it samples (it listens for a few
seconds and reads the most recent message).

Mapping is a parameter because the front/back -> torpedo/octagon assignment
is a mission decision, NOT something the detector knows. Confirm the defaults
match the course before a run; flip front_task/back_task if they are backwards.

Nothing needs to be sent to the boards to start them: stream_audio streams
continuously, so this node only reads.
"""
import threading
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from hardware.utils.tdoa import audio_source, detection
from hardware.utils.tdoa.fft_library import FFTLibrary


class Pinger(Node):
    def __init__(self):
        super().__init__("pinger")

        self.declare_parameter("topic", "/pinger/task")
        self.declare_parameter("window_sec", 10.0)   # audio recorded per cycle
        self.declare_parameter("rate", 96000)        # capture sample rate
        self.declare_parameter("publish_hz", 2.0)    # latched-verdict republish
        # front/back -> task order. See module docstring: verify per course.
        self.declare_parameter("front_task", "torpedo_first")
        self.declare_parameter("back_task", "octagon_first")

        gp = self.get_parameter
        self.topic = gp("topic").get_parameter_value().string_value
        self.window_sec = gp("window_sec").get_parameter_value().double_value
        self.rate = gp("rate").get_parameter_value().integer_value
        publish_hz = gp("publish_hz").get_parameter_value().double_value
        self.front_task = gp("front_task").get_parameter_value().string_value
        self.back_task = gp("back_task").get_parameter_value().string_value

        # depth 10, default (reliable/volatile) QoS to match the branch
        # subscription in planning/path_generator.py.
        self._pub = self.create_publisher(String, self.topic, 10)

        # Latest front/back -> task decision. Written by the detector thread,
        # published by the timer (both plain str reads/writes -> atomic in
        # CPython, no lock needed).
        self.latest_task = None

        self.get_logger().warning(
            f"Pinger mapping: front -> '{self.front_task}', "
            f"back -> '{self.back_task}'. VERIFY this matches the course "
            "(flip front_task/back_task params if backwards)."
        )

        # Republish the latched verdict so the branch always has a fresh
        # message to read whenever its listen window samples the topic.
        self.create_timer(1.0 / publish_hz, self._republish)

        self._thread = threading.Thread(target=self._detect_loop, daemon=True)
        self._thread.start()

    def _republish(self):
        if self.latest_task is not None:
            self._pub.publish(String(data=self.latest_task))

    def _detect_loop(self):
        listen_ms = int(self.window_sec * 1000)
        while rclpy.ok():
            try:
                # Blocking: records window_sec from both boards, then analyzes.
                chans, rate = audio_source.live_channels(
                    self.window_sec, rate=self.rate
                )
                fft = FFTLibrary(rate)
                verdict = detection.run_ping(chans, rate, fft, listen_ms)
            except SystemExit as exc:
                # audio_source/daisy_stream sys.exit() when no board is
                # connected -- do not let that kill the node; retry.
                self.get_logger().warning(f"pinger capture failed: {exc}; retry")
                time.sleep(1.0)
                continue
            except Exception as exc:  # noqa: BLE001 - never die on a bad window
                self.get_logger().warning(f"pinger cycle error: {exc}; retry")
                time.sleep(1.0)
                continue

            if verdict == "front":
                task = self.front_task
            elif verdict == "back":
                task = self.back_task
            else:
                # inconclusive / no valid ping: keep the previous latched
                # decision rather than clobber it with nothing.
                self.get_logger().info(f"pinger verdict '{verdict}': no update")
                continue

            self.latest_task = task
            self.get_logger().info(
                f"pinger verdict '{verdict}' -> publishing '{task}'"
            )


def main(args=None):
    rclpy.init(args=args)
    node = Pinger()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
