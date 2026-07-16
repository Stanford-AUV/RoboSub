"""Hydrophone sample sources for the TDOA ports.

Feeds the ported detection algorithms the same per-channel float samples
the firmware saw, either from the raw hi-fi WAVs that stream_transcribe
records (one mono file per channel under a session's raw/ch<C>/0.wav) or
captured live from the boards.

IMPORTANT gain note: the stream_audio firmware bakes kGain = 100 into every
sample — the same x100 `multiplier` the detection firmwares apply in their
audio callbacks. Samples from these sources are therefore already
"processedSample" values; the ports must NOT multiply by 100 again.
"""
import argparse
import os
import sys
import time
import wave

import numpy as np

from . import daisy_stream  # colocated in this package (see tdoa/__init__)


def _read_mono_wav(path):
    """-> (mono float32 in [-1, 1], rate). Inverts daisy_stream scaling."""
    with wave.open(path, "rb") as w:
        rate = w.getframerate()
        width = w.getsampwidth()
        raw = w.readframes(w.getnframes())
    if width == 2:
        pcm = np.frombuffer(raw, "<i2").astype(np.float32) / 32768.0
    elif width == 3:
        b = np.frombuffer(raw, np.uint8).astype(np.int32)
        v = b[0::3] | (b[1::3] << 8) | (b[2::3] << 16)
        v = (v ^ 0x800000) - 0x800000
        pcm = v.astype(np.float32) / 8388608.0
    else:
        sys.exit(f"{path}: unsupported sample width {width}")
    return pcm, rate


def raw_dir_channels(raw_dir):
    """-> ({hydrophone: mono float32}, rate) from a session's raw/ layout.

    Reads every raw/ch<C>/0.wav (mono, native rate/bit-depth) that
    stream_transcribe writes. `raw_dir` may be the session directory or its
    raw/ subdirectory."""
    import glob
    import re

    if os.path.isdir(os.path.join(raw_dir, "raw")):
        raw_dir = os.path.join(raw_dir, "raw")
    paths = sorted(glob.glob(os.path.join(raw_dir, "ch*", "*.wav")))
    if not paths:
        sys.exit(f"no ch*/*.wav found under {raw_dir}")
    chans, rate = {}, None
    for path in paths:
        m = re.search(r"ch(\d+)", os.path.basename(os.path.dirname(path)))
        if not m:
            continue
        pcm, r = _read_mono_wav(path)
        if rate is not None and r != rate:
            sys.exit(f"sample rate mismatch: {rate} vs {r} ({path})")
        rate = r
        chans[int(m.group(1))] = pcm
    if not chans:
        sys.exit(f"no channel WAVs decoded under {raw_dir}")
    return chans, rate


def live_channels(seconds, rate=96000):
    """Record `seconds` from every connected stream_audio board, then
    return ({hydrophone: mono float32}, rate). Record-then-analyze keeps
    the analysis loop identical to the offline path."""
    from tqdm import tqdm

    found = []
    for serial_no, hydro in daisy_stream.BOARDS.items():
        import glob
        hits = glob.glob(f"/dev/serial/by-id/*{serial_no}*")
        if hits:
            found.append((daisy_stream.Stream(hits[0]), hydro))
    if not found:
        sys.exit("no stream_audio board connected")

    for stream, _ in found:
        stream.configure(rate=rate, bits=16)

    chans = {}
    import threading

    def record(stream, hydro):
        got = []
        target = int(seconds * rate)
        bar = tqdm(total=target, desc=f"hydrophones {hydro}", unit="smp")
        for pcm, r, _bits in stream.frames():
            if r != rate:        # frames from before the rate switch
                continue
            got.append(pcm)
            bar.update(len(pcm))
            if sum(len(g) for g in got) >= target:
                break
        bar.close()
        cat = np.concatenate(got)[:target]
        for i, ch in enumerate(hydro):
            chans[ch] = cat[:, i]

    threads = [threading.Thread(target=record, args=f) for f in found]
    for t in threads:
        t.start()
    for t in threads:
        t.join()
    return chans, rate


def add_source_args(parser):
    parser.add_argument("--raw-dir", dest="raw_dir",
                        help="session dir (or its raw/) with ch<C>/0.wav")
    parser.add_argument("--live", type=float, metavar="SECONDS",
                        help="record this many seconds from the boards")
    parser.add_argument("--rate", type=int, default=96000,
                        help="live capture sample rate (default 96000)")


def channels_from_args(args):
    if args.live:
        return live_channels(args.live, args.rate)
    if getattr(args, "raw_dir", None):
        return raw_dir_channels(args.raw_dir)
    sys.exit("no input given (use --raw-dir or --live)")
