"""Consolidated exact Python port of the Daisy Seed detection firmwares.

One entry point, three modes — each a line-faithful port of the matching
gen2/daisyseed_firmware/master_*.cpp, using the firmware's **competition**
settings (the values that ship commented-out as "Competition Configuration";
the boards run the looser "Testing" values by default):

    level   master_level.cpp   64-pt FFT @ 35 kHz    per-block level log (ms)
    ping    master_ping.cpp    64-pt FFT @ 25 kHz    front/back verdict
    tdoa    master_tdoa.cpp    1024-pt FFT @ 1.76 kHz threshold edge times (us)

Usage:
    python master.py level --raw-dir ../whisper_ivc/data/<session>
    python master.py ping  --live 10
    python master.py tdoa  --raw-dir ../whisper_ivc/data/<session>

Same FFT settings and implementation for every mode (fft_library.py:
recursive radix-2 Cooley-Tukey, Hanning window, bin-window magnitude sum,
float32 throughout). Samples come from audio_source (the per-channel raw
hi-fi WAVs stream_transcribe records, or a live capture).

Differences forced by running on the Orin instead of the boards, all exact:
- stream_audio bakes kGain = 100 into every sample — the same x100 the
  firmwares apply in their audio callbacks — so it is NOT reapplied here;
- hydrophones 2/3 arrive on the vehicle as analog levels over the slave's
  DAC -> master's ADC. Here they are computed straight from the recorded
  channels with slave.cpp's own pipeline. slave.cpp runs one fixed config no
  matter which master program is loaded, so its settings (SLAVE below) are
  shared by every mode — which is why in level/tdoa the slave detects 25 kHz
  while the master detects a different target;
- time is the sample clock (sample_index / rate), not System::GetUs()
  polling, so timestamps are sample-accurate rather than loop-jittered;
- consecutive non-overlapping FFT blocks are processed; the firmware skips
  whatever arrives while it is busy.
"""
import argparse

from tqdm import tqdm

from . import audio_source
from .fft_library import FFTLibrary

# --- Competition configuration (the firmwares' commented "Competition" blocks)

# slave.cpp drives hydrophones 2/3 with ONE fixed config regardless of which
# master program is running, so it is shared across every mode.
SLAVE = dict(fft=64, target=25000.0, tol=0.01, maxes=(3.5, 3.5))

LEVEL = dict(fft=64, target=35000.0, tol=0.01, maxes=(4.0, 4.0), print_ms=1)
PING = dict(fft=64, target=25000.0, tol=0.01, maxes=(4.0, 4.0),
            base_threshold=0.04, listen_ms=10000, off_ms=1000, within_us=3000)
TDOA = dict(fft=1024, target=1760.0, tol=0.01, maxes=(9.0, 12.0),
            base_threshold=0.5)


def _level(fft, buf, cfg, ch):
    """Normalized master-channel level: min(mag, max) / max."""
    m = fft.getFrequencyMagnitude(buf, cfg["fft"], cfg["target"], cfg["tol"])
    mx = cfg["maxes"][ch]
    return min(m, mx) / mx


def _slave(fft, buf, idx):
    """Normalized slave-channel level (hydrophone 2 -> idx 0, 3 -> idx 1)."""
    m = fft.getFrequencyMagnitude(buf, SLAVE["fft"], SLAVE["target"],
                                  SLAVE["tol"])
    mx = SLAVE["maxes"][idx]
    return min(m, mx) / mx


# ------------------------------------------------------------------ level mode
def run_level(chans, rate, fft):
    cfg = LEVEL
    n_samples = min(len(c) for c in chans.values())
    n_blocks = n_samples // cfg["fft"]

    print("TDOA Frequency Detection Ready")
    print(f"Continuous monitoring: printing levels every "
          f"{cfg['print_ms']} ms")

    last_print = 0.0
    levels = [0.0, 0.0, 0.0, 0.0]

    for b in tqdm(range(n_blocks), unit="blk"):
        s, e = b * cfg["fft"], (b + 1) * cfg["fft"]

        if 0 in chans:
            levels[0] = _level(fft, chans[0][s:e], cfg, 0)
        if 1 in chans:
            levels[1] = _level(fft, chans[1][s:e], cfg, 1)
        if 2 in chans:
            levels[2] = _slave(fft, chans[2][s:e], 0)
        if 3 in chans:
            levels[3] = _slave(fft, chans[3][s:e], 1)

        current = e * 1000.0 / rate
        if current - last_print >= cfg["print_ms"]:
            tqdm.write(f"hydrophone_log: Mic0 reads{levels[0]:.3f} "
                       f"Mic1 reads{levels[1]:.3f} "
                       f"Mic2 reads{levels[2]:.3f} "
                       f"Mic3 reads{levels[3]:.3f}")
            last_print = current


# ------------------------------------------------------------------- ping mode
def run_ping(chans, rate, fft, listen_ms):
    cfg = PING
    print(f"localization for {cfg['target']:.3f} Hz starting!! "
          f"(wait {listen_ms} ms)")

    front_counter = 0
    back_counter = 0
    start_ms = 0.0
    most_recent_ping_ms = start_ms
    can_be_measured = False
    received_us = [0, 0, 0, 0]
    was_above = [False, False, False, False]
    levels = [0.0, 0.0, 0.0, 0.0]

    n_samples = min(len(c) for c in chans.values())
    n_blocks = min(n_samples // cfg["fft"],
                   int(listen_ms / 1000.0 * rate) // cfg["fft"])

    for b in tqdm(range(n_blocks), unit="blk"):
        s, e = b * cfg["fft"], (b + 1) * cfg["fft"]
        current_us = e * 1_000_000.0 / rate
        current_ms = current_us / 1000.0

        for ch in (0, 1):
            if ch in chans:
                levels[ch] = _level(fft, chans[ch][s:e], cfg, ch)
        for idx, ch in enumerate((2, 3)):
            if ch in chans:
                levels[ch] = _slave(fft, chans[ch][s:e], idx)

        # Threshold rising edges (only timestamped once the ping is "off"
        # long enough that a fresh arrival can be measured).
        for ch in range(4):
            is_above = levels[ch] >= cfg["base_threshold"]
            if is_above and not was_above[ch]:
                if can_be_measured:
                    received_us[ch] = current_us
                most_recent_ping_ms = current_ms
            was_above[ch] = is_above

        # Once all four hydrophones have an arrival, measure the TDOA.
        if all(t != 0 for t in received_us):
            if max(received_us) - min(received_us) < cfg["within_us"]:
                smallest = min(range(4), key=lambda i: received_us[i])
                second = min((i for i in range(4) if i != smallest),
                             key=lambda i: received_us[i])
                for idx in (smallest, second):
                    if idx in (0, 2):
                        tqdm.write("front detected")
                        front_counter += 1
                    else:
                        tqdm.write("back detected")
                        back_counter += 1
            received_us = [0, 0, 0, 0]
            can_be_measured = False

        if current_ms - most_recent_ping_ms >= cfg["off_ms"]:
            can_be_measured = True

    if front_counter > back_counter:
        verdict = "front"
    elif front_counter < back_counter:
        verdict = "back"
    elif front_counter == 0 and back_counter == 0:
        verdict = "no valid ping detected"
    else:
        verdict = "inconclusive"
    print(f"hydrophone:{verdict}")
    return verdict


# ------------------------------------------------------------------- tdoa mode
def run_tdoa(chans, rate, fft):
    cfg = TDOA
    print("TDOA Frequency Detection Ready")

    levels = [0.0, 0.0, 0.0, 0.0]
    was_above = [False, False, False, False]

    # The firmware fills a 1024-sample buffer for 0/1 while the slave's
    # 64-sample pipeline updates 2/3 sixteen times as often. Step on the
    # slave cadence; 0/1 refresh whenever a full 1024 block completes.
    n_samples = min(len(c) for c in chans.values())
    n_steps = n_samples // SLAVE["fft"]

    for step in tqdm(range(n_steps), unit="blk"):
        e = (step + 1) * SLAVE["fft"]
        t_us = e * 1_000_000.0 / rate

        if e % cfg["fft"] == 0:
            s = e - cfg["fft"]
            if 0 in chans:
                levels[0] = _level(fft, chans[0][s:e], cfg, 0)
            if 1 in chans:
                levels[1] = _level(fft, chans[1][s:e], cfg, 1)

        s = e - SLAVE["fft"]
        for idx, ch in enumerate((2, 3)):
            if ch in chans:
                levels[ch] = _slave(fft, chans[ch][s:e], idx)

        for ch in range(4):
            is_above = levels[ch] >= cfg["base_threshold"]
            if is_above and not was_above[ch]:
                tqdm.write(f"hydrophone_log: Mic{ch} reads {int(t_us)}")
            was_above[ch] = is_above


MODES = {"level": run_level, "ping": run_ping, "tdoa": run_tdoa}


def main():
    parser = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("mode", choices=MODES,
                        help="which detection firmware to run")
    parser.add_argument("--listen-ms", type=int, default=PING["listen_ms"],
                        help="ping mode: listen window (default 10000)")
    audio_source.add_source_args(parser)
    args = parser.parse_args()

    chans, rate = audio_source.channels_from_args(args)
    fft = FFTLibrary(rate)

    if args.mode == "ping":
        run_ping(chans, rate, fft, args.listen_ms)
    else:
        MODES[args.mode](chans, rate, fft)


if __name__ == "__main__":
    main()
