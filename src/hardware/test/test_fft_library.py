"""Parity: hardware.pinger.fft_library vs a literal scalar transliteration
of the DaisySeed fft_library.cpp, plus a numpy.fft sanity cross-check."""
import cmath
import math

import numpy as np
import pytest

from hardware.pinger.fft_library import FFTLibrary


# ---- literal scalar transliteration of the C++ (slow, reference only) ----
def cpp_fft(signal):
    N = len(signal)
    if N <= 1:
        return signal
    even = cpp_fft(signal[0::2])
    odd = cpp_fft(signal[1::2])
    out = [0j] * N
    for k in range(N // 2):
        t = cmath.exp(-2j * math.pi * k / N) * odd[k]
        out[k] = even[k] + t
        out[k + N // 2] = even[k] - t
    return out


def cpp_get_frequency_magnitude(buf, buffer_size, target, tol, rate):
    sig = [complex(float(np.float32(x)), 0.0) for x in buf[:buffer_size]]
    sig += [0j] * (buffer_size - len(sig))
    N = buffer_size
    for i in range(N):
        w = 0.5 * (1.0 - math.cos(2.0 * math.pi * i / (N - 1)))
        sig[i] *= w
    sig = cpp_fft(sig)
    lower_bin = int(target * (1.0 - tol) * N / rate)
    upper_bin = int(target * (1.0 + tol) * N / rate)
    lower_bin = min(lower_bin, N // 2 - 1)
    upper_bin = min(upper_bin, N // 2 - 1)
    return sum(abs(sig[b]) for b in range(lower_bin, upper_bin + 1))


@pytest.mark.parametrize(
    "size, target",
    [(64, 1046.0),      # active Testing config target
     (64, 25000.0),     # commented Competition config target
     (64, 14080.0)],
)
def test_parity_vs_cpp_reference(size, target):
    rng = np.random.default_rng(0)
    lib = FFTLibrary(96000.0)
    tol = 0.01
    for _ in range(20):
        t = np.arange(size) / 96000.0
        buf = (np.sin(2 * np.pi * target * t)
               + 0.3 * rng.standard_normal(size)).astype(np.float32)
        got = lib.getFrequencyMagnitude(buf, size, target, tol)
        ref = cpp_get_frequency_magnitude(buf, size, target, tol, 96000.0)
        assert abs(got - ref) / max(abs(ref), 1e-9) < 1e-4


def test_fft_matches_numpy():
    rng = np.random.default_rng(1)
    lib = FFTLibrary(96000.0)
    x = rng.standard_normal(1024).astype(np.float32).astype(np.complex64)
    ours = lib.fft(x)
    ref = np.fft.fft(x.astype(np.complex128))
    rel = np.max(np.abs(ours - ref)) / np.max(np.abs(ref))
    assert rel < 1e-4


def test_batch_equals_scalar_exactly():
    rng = np.random.default_rng(2)
    lib = FFTLibrary(96000.0)
    blocks = rng.standard_normal((50, 64)).astype(np.float32)
    batch = lib.getFrequencyMagnitudeBatch(blocks, 64, 1046.0, 0.01)
    scalar = np.array(
        [lib.getFrequencyMagnitude(b, 64, 1046.0, 0.01) for b in blocks],
        dtype=np.float32,
    )
    assert batch.dtype == np.float32
    assert batch.shape == (50,)
    # Same float32 op graph elementwise -> bitwise-equal results.
    assert np.array_equal(batch, scalar)


def test_batch_throughput():
    # 1 second of 4-channel audio = 6000 blocks; must be far faster than
    # real time. Generous bound: 2 s on the Orin.
    import time

    lib = FFTLibrary(96000.0)
    blocks = np.random.default_rng(3).standard_normal((6000, 64)) \
        .astype(np.float32)
    t0 = time.monotonic()
    lib.getFrequencyMagnitudeBatch(blocks, 64, 1046.0, 0.01)
    assert time.monotonic() - t0 < 2.0
