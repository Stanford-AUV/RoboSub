"""pump_batches: batch slicing, remainder carry, sample clock, rate skip."""
import numpy as np

from hardware.pinger import detector
from hardware.pinger.pump import pump_batches

RATE = 96000
BLOCK_US = 64 / RATE * 1e6


def tone(n, amp=0.5):
    t = np.arange(n) / RATE
    return (amp * np.sin(2 * np.pi * detector.targetFrequency * t)) \
        .astype(np.float32)


def batch(n, amp=0.5):
    pcm = np.zeros((n, 2), np.float32)
    pcm[:, 0] = tone(n, amp)
    return (pcm, RATE, 16)


def test_blocks_and_sample_clock():
    calls = []
    pump_batches([batch(128)], 0,
                 lambda fc, lv, t: calls.append((fc, lv, t)))
    assert len(calls) == 2
    assert calls[0][2] == BLOCK_US          # end of block 1
    assert calls[1][2] == 2 * BLOCK_US
    assert all(fc == 0 for fc, _, _ in calls)
    # levels match compute_levels on the same slices
    blocks = np.stack([tone(128)[:64], tone(128)[64:]])
    expect = detector.compute_levels(blocks, 0)
    got = [lv[0] for _, lv, _ in calls]
    np.testing.assert_array_equal(got, expect)


def test_remainder_carries_across_batches():
    calls = []
    pump_batches([batch(96), batch(96)], 2,
                 lambda fc, lv, t: calls.append(t))
    # 192 samples total -> 3 blocks, timed on the global sample count
    assert calls == [BLOCK_US, 2 * BLOCK_US, 3 * BLOCK_US]


def test_wrong_rate_batches_skipped():
    calls = []
    pcm = np.zeros((64, 2), np.float32)
    pump_batches([(pcm, 16000, 16), batch(64)], 0,
                 lambda fc, lv, t: calls.append(t))
    assert calls == [BLOCK_US]              # 16 kHz leftovers don't count


def test_on_pcm_sees_every_kept_batch():
    seen = []
    pump_batches([batch(96), batch(32)], 0,
                 lambda *a: None, on_pcm=lambda p: seen.append(len(p)))
    assert seen == [96, 32]
