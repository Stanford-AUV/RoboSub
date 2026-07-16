"""Pinger direction node: host-side port of the DaisySeed master_ros.cpp.

The two hydrophone boards run stream_audio.cpp (raw 96 kHz PCM over USB
CDC); this node decodes both streams, runs the exact firmware detection
(hardware/pinger/), and publishes:

  /pinger         std_msgs/String  ONE "front"/"back" message per DECIDED
                  ping (levels crossed the threshold and the collection
                  window closed); planning's branch tallies these during
                  its listen window and takes the majority.
  /pinger/levels  msgs/PingerStamped  ~20 Hz -- per-channel PEAK normalized
                  level since the previous message + the latched direction;
                  sensors_plot's pinger panel and threshold calibration.

All four channels are also recorded to
<root>/data/audio_pingers/<session>/raw/ch<C>/<N>.wav (fragment <N>
increments per reconnect; whisper_ivc-compatible layout).

Boards are found by USB serial under /dev/serial/by-id (BOARDS in
hardware/pinger/daisy_stream.py) -- NEVER by bare /dev/ttyACM<n>: 07/13 a
re-enumeration shuffled the numbers. A missing board is retried forever; a
dead stream reconnects without touching the other board.
"""
import collections
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
from hardware.pinger.debug_plot import render_debug
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

        self._session_dir = os.path.join(
            resolve_root(), "data", "audio_pingers",
            time.strftime("%Y_%m_%d_%H_%M_%S"))
        self._recorder = SessionRecorder(
            self._session_dir, RATE, log=self.get_logger().error)
        self.get_logger().info(f"recording raw stream to {self._session_dir}")

        # Debug plot of the branch listen window: planning publishes
        # "start"/"stop" on <decision>/listening; we keep a ring of recent
        # levels in BOARD SAMPLE-CLOCK time (the same clock the WAVs are
        # written with) and render session_dir/debug.png on "stop", so the
        # plot's x axis lines up with positions in raw/ch*/*.wav.
        self._dbg_lock = threading.Lock()
        self._dbg = {0: collections.deque(maxlen=90_000),   # ~60 s / board
                     2: collections.deque(maxlen=90_000)}
        self._last_t_us = {0: 0.0, 2: 0.0}
        self._listen_start = None
        self.create_subscription(
            String, "/pinger/listening", self._on_listening, 10)

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

        self._pub_seq = 0   # detector.decision_seq already published
        self.create_timer(0.05, self._publish_levels)   # 20 Hz
        self.create_timer(FLUSH_S, self._recorder._flush)

    # ---- board reader threads -------------------------------------------
    def _board_loop(self, serial_no, first_channel):
        # rclpy.ok() catches SIGINT/SIGTERM teardown even when
        # destroy_node never gets to run (launch kills us mid-shutdown);
        # without it this loop reconnected during teardown and recorded
        # junk 0.1 s fragments.
        while not self._stop.is_set() and rclpy.ok():
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
            # One /pinger message per decided ping: planning's branch
            # tallies these (threshold crossing up + down = one detection).
            if self._detector.decision_seq != self._pub_seq:
                self._pub_seq = self._detector.decision_seq
                self._pinger_pub.publish(String(
                    data="front" if self._detector.direction_front
                    else "back"))
            with self._peaks_lock:
                for i, lvl in zip((first, first + 1), levels):
                    if lvl > self._peaks[i]:
                        self._peaks[i] = lvl
            with self._dbg_lock:
                self._dbg[first].append((t_us, levels[0], levels[1]))
                self._last_t_us[first] = t_us

    # ---- branch listen-window debug plot ---------------------------------
    def _on_listening(self, msg):
        if msg.data == "start":
            with self._dbg_lock:
                self._listen_start = dict(self._last_t_us)
            self.get_logger().info("pinger listen window opened")
        elif msg.data == "stop":
            with self._dbg_lock:
                end = dict(self._last_t_us)
                # Missed "start" (e.g. node restarted mid-run): fall back
                # to the nominal listen length + slack.
                start = self._listen_start or {
                    b: max(0.0, t - 12e6) for b, t in end.items()}
                boards = {}
                for first, dq in self._dbg.items():
                    lo, hi = start.get(first, 0.0), end.get(first, 0.0)
                    pts = [p for p in dq if lo <= p[0] <= hi]
                    boards[first] = ([p[0] / 1e6 for p in pts],
                                     [p[1] for p in pts],
                                     [p[2] for p in pts])
                self._listen_start = None
            threading.Thread(target=self._render_debug, args=(boards,),
                             daemon=True).start()

    def _render_debug(self, boards):
        path = os.path.join(self._session_dir, "debug.png")
        try:
            render_debug(path, boards, detector.baseThreshold,
                         self._detector.direction_front)
            self.get_logger().info(f"pinger listen debug plot -> {path}")
        except Exception as e:
            self.get_logger().error(f"debug plot failed: {e}")

    # ---- publishers ------------------------------------------------------
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
