"""Crash-safe raw-stream recording for the pinger node.

Layout mirrors whisper_ivc's stream_transcribe sessions so the offline
tools read it directly: <session>/raw/ch<C>/<N>.wav, one mono 16-bit WAV
per channel, fragment <N> incremented per stream (re)connect.

WAV headers are written with placeholder sizes and patched in place on
every flush (the node flushes ~every 2 s), so a crash or power cut leaves
a valid file missing at most the last flush interval. Python's wave module
only fixes the header on close(), hence the manual RIFF writer.
"""
import os
import struct

import numpy as np


def resolve_root():
    """Repo root for data/: $ROBOSUB_DIR, else ~/RoboSub, else cwd (ros2
    launch starts nodes from $HOME) -- same rule as sensors_plot's PNG."""
    root = os.environ.get("ROBOSUB_DIR") or os.path.expanduser("~/RoboSub")
    return root if os.path.isdir(root) else os.getcwd()


class _WavWriter:
    """One mono 16-bit PCM WAV with an in-place patchable header."""

    _HEADER = struct.Struct("<4sI4s4sIHHIIHH4sI")

    def __init__(self, path, rate):
        self._f = open(path, "wb")
        self._rate = rate
        self._data_bytes = 0
        self._write_header()

    def _write_header(self):
        self._f.write(self._HEADER.pack(
            b"RIFF", 36 + self._data_bytes, b"WAVE",
            b"fmt ", 16, 1, 1, self._rate, self._rate * 2, 2, 16,
            b"data", self._data_bytes))

    def write(self, samples_i16):
        self._f.write(np.asarray(samples_i16, "<i2").tobytes())
        self._data_bytes += len(samples_i16) * 2

    def flush(self):
        pos = self._f.tell()
        self._f.seek(4)
        self._f.write(struct.pack("<I", 36 + self._data_bytes))
        self._f.seek(40)
        self._f.write(struct.pack("<I", self._data_bytes))
        self._f.seek(pos)
        self._f.flush()

    def close(self):
        self.flush()
        self._f.close()


class SessionRecorder:
    """Writes both channels of each board; disables itself on I/O error."""

    def __init__(self, session_dir, rate, log=print):
        self._dir = session_dir
        self._rate = rate
        self._log = log
        self.enabled = True
        self._writers = {}      # channel -> _WavWriter
        self._fragment = {}     # channel -> next fragment index

    def _writer(self, ch):
        if ch not in self._writers:
            frag = self._fragment.get(ch, 0)
            d = os.path.join(self._dir, "raw", f"ch{ch}")
            os.makedirs(d, exist_ok=True)
            self._writers[ch] = _WavWriter(
                os.path.join(d, f"{frag}.wav"), self._rate)
            self._fragment[ch] = frag + 1
        return self._writers[ch]

    def write(self, first_channel, pcm):
        """pcm: (N, 2) float32 in [-1, 1] from the frame decoder."""
        if not self.enabled:
            return
        try:
            # Inverse of daisy_stream._decode's /32768: round-trips the
            # original 16-bit samples bit-exactly; saturates out-of-range.
            i16 = np.clip(np.round(np.asarray(pcm, np.float32) * 32768.0),
                          -32768, 32767).astype("<i2")
            for idx in (0, 1):
                self._writer(first_channel + idx).write(i16[:, idx])
        # ValueError covers writes on a closed file (not an OSError).
        except (OSError, ValueError) as e:
            self.enabled = False
            self._log(f"pinger recording disabled: {e}")

    def next_fragment(self, first_channel):
        """Close the board's writers; next write opens fragment N+1."""
        for idx in (0, 1):
            w = self._writers.pop(first_channel + idx, None)
            if w is not None:
                try:
                    w.close()
                except (OSError, ValueError):
                    pass

    def _flush(self):
        """Patch all headers in place (called ~every 2 s by the node)."""
        if not self.enabled:
            return
        try:
            for w in self._writers.values():
                w.flush()
        except (OSError, ValueError) as e:
            self.enabled = False
            self._log(f"pinger recording disabled: {e}")

    def close(self):
        for w in list(self._writers.values()):
            try:
                w.close()
            except (OSError, ValueError):
                pass
        self._writers.clear()
