"""compute_levels: batched magnitude -> clamp -> normalize, vs the scalar
FFT path and the firmware's clamping semantics."""
import numpy as np

from hardware.pinger import detector
from hardware.pinger.fft_library import FFTLibrary


def tone_block(freq=None, amp=0.5, rate=96000.0, n=64):
    freq = detector.targetFrequency if freq is None else freq
    t = np.arange(n) / rate
    return (amp * np.sin(2 * np.pi * freq * t)).astype(np.float32)


def test_constants_match_firmware_test_config():
    # Structural constants from master_ros.cpp, verbatim. targetFrequency
    # and baseThreshold are live calibration knobs ("WE CAN CHANGE"), so
    # only require them to be sane (any pinger frequency below Nyquist).
    assert 0.0 < detector.targetFrequency < detector.SAMPLE_RATE / 2
    assert detector.frequencyTolerance == 0.01
    # baseThreshold is a live calibration knob ("WE CAN CHANGE"), tuned in
    # the water — only require it to be a sane normalized level.
    assert 0.0 < detector.baseThreshold < 1.0
    assert detector.kFftSize == 64
    assert detector.offThresholdMs == 1000
    assert detector.withinThresholdUs == 3000
    assert detector.kPrintIntervalMs == 200
    assert detector.HYDROPHONE_MAXES == (4.0, 4.0, 4.0, 4.0)
    assert detector.SAMPLE_RATE == 96000.0
    assert detector.FRONT_CHANNELS == frozenset({0, 3})


def test_level_matches_scalar_pipeline():
    blocks = np.stack([tone_block(amp=0.4), tone_block(amp=0.05)])
    levels = detector.compute_levels(blocks, channel=0)
    lib = FFTLibrary(detector.SAMPLE_RATE)
    for lvl, blk in zip(levels, blocks):
        mag = lib.getFrequencyMagnitude(
            blk, detector.kFftSize, detector.targetFrequency,
            detector.frequencyTolerance)
        assert lvl == np.float32(min(mag, 4.0) / 4.0)


def test_level_clamps_to_one():
    blocks = tone_block(amp=200.0)[None, :]   # absurd amplitude
    assert detector.compute_levels(blocks, 0)[0] == 1.0


def test_silence_is_zero():
    blocks = np.zeros((3, 64), np.float32)
    np.testing.assert_array_equal(detector.compute_levels(blocks, 2), 0.0)


def test_off_target_tone_stays_low():
    # A tone far outside the tolerance window must not register.
    blocks = tone_block(freq=6000.0, amp=0.4)[None, :]
    on = detector.compute_levels(tone_block(amp=0.4)[None, :], 0)[0]
    off = detector.compute_levels(blocks, 0)[0]
    assert off < on / 3
