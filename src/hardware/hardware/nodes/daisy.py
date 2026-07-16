"""Pinger direction node: host-side port of the DaisySeed master_ros.cpp.

The two hydrophone boards run stream_audio.cpp (raw 96 kHz PCM over USB
CDC); this node decodes both streams, runs the exact firmware detection
(hardware/pinger/), and publishes:

  /pinger         std_msgs/String  "front"/"back" every 200 ms -- sticky,
                  exactly the firmware's hw.PrintLine mirror ("back" from
                  boot until the first ping decides otherwise); planning's
                  branch segments consume the codes as-is.
  /pinger/levels  msgs/PingerStamped  ~20 Hz -- per-channel PEAK normalized
                  level since the previous message + the latched direction;
                  sensors_plot's pinger panel and threshold calibration.

All four channels are also recorded to
<root>/data/semi_finals_audio/<session>/raw/ch<C>/<N>.wav (fragment <N>
increments per reconnect; whisper_ivc-compatible layout).

Boards are found by USB serial under /dev/serial/by-id (BOARDS in
hardware/pinger/daisy_stream.py) -- NEVER by bare /dev/ttyACM<n>: 07/13 a
re-enumeration shuffled the numbers. A missing board is retried forever; a
dead stream reconnects without touching the other board.
"""
import glob
import os
import queue
import threading
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from msgs.msg import PingerStamped
from hardware.pinger import detector
from hardware.pinger.daisy_stream import BOARDS, Stream
from hardware.pinger.pump import pump_batches
from hardware.pinger.recorder import SessionRecorder, resolve_root

RATE = int(detector.SAMPLE_RATE)   # 96000
BITS = 16                          # halves USB bandwidth vs the 24-bit boot
                                   # default; detection is float32 either way
RETRY_S = 3.0                      # board discovery/reconnect cadence
FLUSH_S = 2.0                      # WAV header patch interval
QUEUE_MAX = 8192                   # ~1.4 s of block events per board


class Daisy(Node):

    def __init__(self):
        super().__init__("daisy")
        self._pinger_pub = self.create_publisher(String, "/pinger", 10)
        self._levels_pub = self.create_publisher(
            PingerStamped, "/pinger/levels", 10)

        self._detector = detector.PingerDetector()
        self._events = queue.Queue(maxsize=QUEUE_MAX)
        self._peaks = [0.0, 0.0, 0.0, 0.0]   # consumer-thread state,
        self._peaks_lock = threading.Lock()  # read by the levels timer

        session = os.path.join(
            resolve_root(), "data", "semi_finals_audio",
            time.strftime("%Y_%m_%d_%H_%M_%S"))
        self._recorder = SessionRecorder(
            session, RATE, log=self.get_logger().error)
        self.get_logger().info(f"recording raw stream to {session}")

        self._stop = threading.Event()
        self._threads = [
            threading.Thread(target=self._consume, daemon=True),
        ]
        for serial_no, chans in BOARDS.items():
            self._threads.append(threading.Thread(
                target=self._board_loop, args=(serial_no, chans[0]),
                daemon=True))
        for t in self._threads:
            t.start()

        # Firmware mirror: latched direction every kPrintIntervalMs.
        self.create_timer(detector.kPrintIntervalMs / 1000.0,
                          self._publish_pinger)
        self.create_timer(0.05, self._publish_levels)   # 20 Hz
        self.create_timer(FLUSH_S, self._recorder._flush)

    # ---- board reader threads -------------------------------------------
    def _board_loop(self, serial_no, first_channel):
        while not self._stop.is_set():
            hits = glob.glob(f"/dev/serial/by-id/*{serial_no}*")
            if not hits:
                self.get_logger().warning(
                    f"hydrophone board {serial_no} (ch {first_channel}/"
                    f"{first_channel + 1}) not connected; retrying",
                    throttle_duration_sec=30.0)
                time.sleep(RETRY_S)
                continue
            stream = None
            try:
                stream = Stream(hits[0])
                stream.configure(rate=RATE, bits=BITS)
                self.get_logger().info(
                    f"board {serial_no}: streaming from {hits[0]}")
                pump_batches(
                    stream.batches(), first_channel,
                    on_levels=self._enqueue,
                    on_pcm=lambda pcm, fc=first_channel:
                        self._recorder.write(fc, pcm),
                    rate=RATE)
            except Exception as e:
                self.get_logger().error(
                    f"board {serial_no} stream died: {e}; reconnecting")
            finally:
                if stream is not None:
                    stream.close()
                self._recorder.next_fragment(first_channel)
            time.sleep(RETRY_S)

    def _enqueue(self, first_channel, pair_levels, t_us):
        try:
            self._events.put_nowait((first_channel, pair_levels, t_us))
        except queue.Full:
            try:                     # drop-oldest, keep the fresh event
                self._events.get_nowait()
                self._events.put_nowait((first_channel, pair_levels, t_us))
            except (queue.Empty, queue.Full):
                pass                 # racing the other reader: drop ours
            self.get_logger().warning(
                "pinger event queue overflow: detection falling behind",
                throttle_duration_sec=10.0)

    # ---- single consumer: the state machine stays single-threaded -------
    def _consume(self):
        while not self._stop.is_set():
            try:
                first, levels, t_us = self._events.get(timeout=0.5)
            except queue.Empty:
                continue
            self._detector.process_block_levels(first, levels, t_us)
            with self._peaks_lock:
                for i, lvl in zip((first, first + 1), levels):
                    if lvl > self._peaks[i]:
                        self._peaks[i] = lvl

    # ---- publishers ------------------------------------------------------
    def _publish_pinger(self):
        self._pinger_pub.publish(String(
            data="front" if self._detector.direction_front else "back"))

    def _publish_levels(self):
        msg = PingerStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        with self._peaks_lock:
            msg.levels = [float(p) for p in self._peaks]
            self._peaks = [0.0, 0.0, 0.0, 0.0]
        msg.front = self._detector.direction_front
        self._levels_pub.publish(msg)

    def destroy_node(self):
        self._stop.set()
        self._recorder.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    daisy = Daisy()
    try:
        rclpy.spin(daisy)
    finally:
        daisy.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
