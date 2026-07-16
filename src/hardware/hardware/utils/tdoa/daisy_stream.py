"""Shared reader for the Daisy stream_audio firmware's USB PCM stream.

Wire format (little-endian):
    [0xDA][0x7A][seq u8][fmt u8][N x (ch0, ch1)]
    fmt: bit7 = 24-bit samples, bit6 = 64 samples/frame (else 32),
         bits0-3 = decimation from 96 kHz.
         0x00 = legacy firmware (16 kHz / 16-bit, 32 samples).
    Samples are s16, or s24 packed in 3 bytes when the 24-bit flag is set.
    64-sample frames align 1:1 with the detection firmwares' FFT blocks.

The firmware accepts config lines on the same port:
    "rate 96000|48000|32000|24000|16000", "bits 16|24", "reboot".
"""
import fcntl
import glob
import os
import subprocess
import sys
import termios

import numpy as np
import serial

MAGIC = b"\xda\x7a"
DEBUG_GAPS = bool(os.environ.get("DAISY_DEBUG_GAPS"))
HEADER_BYTES = 4
CODEC_RATE = 96000
RATES = (96000, 48000, 32000, 24000, 16000)
DEFAULT_RATE = 96000            # firmware boot default
DEFAULT_BITS = 24

# board USB serial -> the two hydrophone channels wired to its codec
BOARDS = {
    "376C36533433": (0, 1),  # ex-master board: hydrophones 0/1
    "376C36573433": (2, 3),  # ex-slave board: hydrophones 2/3
}


def _fmt_info(fmt):
    """(rate_hz, bits, samples, frame_bytes) for a frame's fmt byte."""
    if fmt == 0:                # legacy firmware: 16 kHz / 16-bit
        return 16000, 16, 32, HEADER_BYTES + 32 * 2 * 2
    bits = 24 if fmt & 0x80 else 16
    samples = 64 if fmt & 0x40 else 32
    dec = fmt & 0x0F
    if dec not in (1, 2, 3, 4, 6):
        return None
    return (CODEC_RATE // dec, bits, samples,
            HEADER_BYTES + samples * 2 * bits // 8)


def port_for_channel(channel):
    """Serial port of the board that carries this hydrophone channel."""
    for serial_no, chans in BOARDS.items():
        if channel in chans:
            hits = glob.glob(f"/dev/serial/by-id/*{serial_no}*")
            if not hits:
                sys.exit(f"board {serial_no} (hydrophone {channel}) "
                         "is not connected")
            return hits[0]
    sys.exit(f"no board carries channel {channel}")


def _streams_frames(port, probe_s=1.5):
    """True if this port is emitting stream_audio's binary frames."""
    try:
        with serial.Serial(port, 115200, timeout=probe_s) as ser:
            return MAGIC in ser.read(1024)
    except (OSError, serial.SerialException):
        return False


def find_port():
    ports = glob.glob("/dev/serial/by-id/usb-Electrosmith_Daisy_Seed*")
    if len(ports) == 1:
        return ports[0]
    if not ports:
        sys.exit("no Daisy serial port found")
    # Multiple boards: pick the one running stream_audio (binary frames);
    # other firmware (e.g. master_level) prints text lines instead
    streaming = [p for p in ports if _streams_frames(p)]
    if len(streaming) == 1:
        print(f"auto-selected streaming board: {streaming[0]}")
        return streaming[0]
    sys.exit(f"{len(streaming)} of {len(ports)} Daisy ports are streaming "
             f"audio frames ({ports}) — use --port to choose")


def _decode(payload, bits, samples):
    """Payload bytes -> (samples, 2) float32 in [-1, 1]."""
    if bits == 16:
        pcm = np.frombuffer(payload, dtype="<i2").astype(np.float32) / 32768.0
    else:
        b = np.frombuffer(payload, dtype=np.uint8).astype(np.int32)
        v = b[0::3] | (b[1::3] << 8) | (b[2::3] << 16)
        v = (v ^ 0x800000) - 0x800000          # sign-extend 24 -> 32 bit
        pcm = v.astype(np.float32) / 8388608.0
    return pcm.reshape(samples, 2)


class Stream:
    """Owns the serial port: yields decoded frames and sends config.

    Reading is delegated to a `cat` subprocess feeding a 1 MB pipe: a
    same-process reader thread shares the GIL with numpy/UI work, and any
    stall > ~110 ms overflows the kernel tty buffer, where the cdc-acm
    throttle path loses AND reorders chunks (seen as seq jumping forward
    then backward). `cat` never touches the GIL, and the pipe absorbs
    ~1.7 s of host stall at 96 kHz/24-bit.
    """

    def __init__(self, port):
        self.port = port
        # write side (config commands) + sets the tty raw via termios
        self.ser = serial.Serial(port, 115200, timeout=None)
        # pyserial always leaves the tty at VMIN=0 (it waits via select on
        # its own fd), under which cat's blocking read() returns 0 on a
        # momentarily-empty tty -- cat takes that as EOF and exits. VMIN=1
        # makes reads wait for at least one byte. Per-tty setting, so it
        # covers cat's separate fd; pyserial writes are unaffected.
        attrs = termios.tcgetattr(self.ser.fd)
        attrs[6][termios.VMIN] = 1
        attrs[6][termios.VTIME] = 0
        termios.tcsetattr(self.ser.fd, termios.TCSANOW, attrs)
        tty_fd = os.open(port, os.O_RDONLY | os.O_NOCTTY)
        try:
            self.proc = subprocess.Popen(
                ["cat"], stdin=tty_fd, stdout=subprocess.PIPE, bufsize=0)
        finally:
            os.close(tty_fd)
        try:
            F_SETPIPE_SZ = 1031            # Linux-specific fcntl
            fcntl.fcntl(self.proc.stdout.fileno(), F_SETPIPE_SZ, 1 << 20)
        except OSError:
            pass                           # default 64 kB pipe still works

    def configure(self, rate=None, bits=None):
        """Ask the firmware for a sample rate (Hz) and/or bit depth."""
        if rate is not None:
            if rate not in RATES:
                raise ValueError(f"rate must be one of {RATES}")
            self.ser.write(f"rate {rate}\n".encode())
        if bits is not None:
            if bits not in (16, 24):
                raise ValueError("bits must be 16 or 24")
            self.ser.write(f"bits {bits}\n".encode())

    def close(self):
        self.proc.terminate()
        self.ser.close()

    def batches(self, stats=None):
        """Yield (pcm, rate, bits) with pcm = (N, 2) float32 in [-1, 1].

        Every yield decodes ALL complete frames currently buffered into one
        array: at 96 kHz a board emits 1500 frames/s, and per-frame Python
        work in the consumer is exactly what lets the kernel serial buffer
        overflow (host-side drops). A batch never mixes formats — a frame
        with a different fmt byte starts the next batch.

        stats, if given, is a dict updated in place with 'samples' (per
        channel), 'dropped' (sequence gaps), 'rate' and 'bits' (most recent).
        """
        buf = bytearray()
        last_seq = None
        pipe_fd = self.proc.stdout.fileno()
        while True:
            # Blocking read from the cat pipe (releases the GIL).
            data = os.read(pipe_fd, 1 << 18)
            if not data:                   # cat exited: board disconnected
                raise serial.SerialException(f"{self.port}: stream ended")
            buf += data
            # Parse with an index -- `del buf[:n]` per frame is a memmove
            # of the whole remaining buffer, which goes quadratic the
            # moment the reader falls behind and buf grows (the thread
            # then pegs a core and NEVER catches up).
            payloads, cur_fmt = [], None
            pos, n = 0, len(buf)
            while True:
                start = buf.find(MAGIC, pos)
                if start < 0:
                    pos = n - 1 if n else 0
                    break
                pos = start
                if n - pos < HEADER_BYTES:
                    break
                fmt = buf[pos + 3]
                info = _fmt_info(fmt)
                if info is None:            # corrupt fmt: resync past magic
                    pos += 2
                    continue
                rate, bits, samples, frame_bytes = info
                # The magic can occur INSIDE audio payload: after any true
                # byte loss, locking onto a false magic mis-parses garbage
                # frames (random seq -> phantom "drops") until luck restores
                # sync. Only accept a frame once the NEXT frame's magic is
                # visible right behind it; the stream is continuous, so a
                # real boundary always gets its lookahead eventually.
                if n - pos < frame_bytes + 2:
                    break
                if buf[pos + frame_bytes:pos + frame_bytes + 2] != MAGIC:
                    pos += 2                # false magic: keep scanning
                    continue
                if cur_fmt is not None and fmt != cur_fmt:
                    yield self._flush(payloads, cur_fmt)
                    payloads = []
                cur_fmt = fmt
                seq = buf[pos + 2]
                if stats is not None:
                    if last_seq is not None and seq != (last_seq + 1) % 256:
                        stats["dropped"] = stats.get("dropped", 0) + 1
                        if DEBUG_GAPS:
                            import time as _t
                            print(f"[gap] {self.port[-20:]} "
                                  f"seq {last_seq}->{seq} "
                                  f"(delta {(seq - last_seq) % 256}) "
                                  f"buf={n} t={_t.time():.3f}",
                                  flush=True)
                    stats["samples"] = stats.get("samples", 0) + samples
                    stats["rate"] = rate
                    stats["bits"] = bits
                last_seq = seq
                payloads.append(bytes(buf[pos + HEADER_BYTES:
                                          pos + frame_bytes]))
                pos += frame_bytes
            del buf[:pos]
            if payloads:
                yield self._flush(payloads, cur_fmt)

    @staticmethod
    def _flush(payloads, fmt):
        rate, bits, samples, _ = _fmt_info(fmt)
        pcm = _decode(b"".join(payloads), bits, samples * len(payloads))
        return pcm, rate, bits

    def frames(self, stats=None):
        """Back-compat: yield one (64, 2) frame at a time (slow path)."""
        for pcm, rate, bits in self.batches(stats):
            n = 64 if len(pcm) % 64 == 0 else 32
            for i in range(0, len(pcm), n):
                yield pcm[i:i + n], rate, bits


def frames(port, stats=None):
    """Back-compat wrapper: yield only the (32, 2) float32 pcm arrays."""
    for pcm, _rate, _bits in Stream(port).frames(stats):
        yield pcm
