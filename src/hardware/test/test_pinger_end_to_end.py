"""Synthetic 96 kHz PCM (1046 Hz bursts, the active Testing config) through
the real FFT pipeline + state machine."""
import numpy as np

from hardware.pinger import detector

RATE = 96000
BLOCK = detector.kFftSize          # 64
BLOCK_US = BLOCK / RATE * 1e6


def synth(total_s, bursts):
    """4 channels of silence with 1046 Hz bursts.

    bursts: list of (channel, start_s, dur_s). Amplitude 0.9 -> level well
    above baseThreshold (sanity-checked in the test).
    """
    n = int(total_s * RATE)
    chans = np.zeros((4, n), np.float32)
    t = np.arange(n) / RATE
    for ch, start, dur in bursts:
        m = (t >= start) & (t < start + dur)
        chans[ch][m] = (0.9 * np.sin(
            2 * np.pi * detector.targetFrequency * t[m])).astype(np.float32)
    return chans


def run(chans):
    det = detector.PingerDetector()
    n_blocks = chans.shape[1] // BLOCK
    blocks = chans[:, :n_blocks * BLOCK].reshape(4, n_blocks, BLOCK)
    lv = np.stack([detector.compute_levels(blocks[c], c) for c in range(4)])
    for b in range(n_blocks):
        t_us = (b + 1) * BLOCK_US
        det.process_block_levels(0, (lv[0, b], lv[1, b]), t_us)
        det.process_block_levels(2, (lv[2, b], lv[3, b]), t_us)
    return det


def test_burst_level_clears_threshold():
    chans = synth(0.01, [(0, 0.0, 0.01)])
    lv = detector.compute_levels(chans[0][None, :64], 0)
    assert lv[0] >= detector.baseThreshold


def test_front_first_ping_yields_front():
    # ch0 and ch3 (front) lead by two blocks; ch1 trails inside the window.
    d = 5e-3
    det = run(synth(1.5, [(0, 1.2, d), (3, 1.2, d),
                          (1, 1.2 + 2 * BLOCK / RATE, d)]))
    assert det.direction_front is True


def test_back_only_ping_yields_back():
    d = 5e-3
    det = run(synth(1.5, [(1, 1.2, d), (2, 1.2, d)]))
    assert det.direction_front is False


def test_two_pings_second_flips_decision():
    d = 5e-3
    det = run(synth(3.0, [
        (0, 1.2, d), (3, 1.2, d),                # ping 1: front
        (1, 2.5, d), (2, 2.5, d),                # ping 2 (1.3 s later): back
    ]))
    assert det.direction_front is False


def test_silence_keeps_boot_value():
    det = run(synth(1.5, []))
    assert det.direction_front is False
