"""Exact Python port of gen2/daisyseed_firmware/library/fft_library.cpp.

Same algorithm, same settings, same numeric type (float32 / complex64):
- recursive Cooley-Tukey radix-2 DIT FFT (not numpy's FFT),
- Hanning window with the 1/(N-1) denominator,
- getFrequencyMagnitude: sum of bin magnitudes over the tolerance window,
  with the C++ size_t truncation and half-spectrum clamping,
- detectPitch: peak + parabolic interpolation, matching the C++ edge cases.

The recursion is transliterated with vectorized numpy per stage — the
operation graph and float32 precision match the C++; only the loop over k
is done as an array op.
"""
import numpy as np


class FFTLibrary:
    def __init__(self, sample_rate):
        self.m_sampleRate = np.float32(sample_rate)

    # Recursive Cooley-Tukey Radix-2 FFT implementation.
    def fft(self, signal):
        """signal: complex64 array; returns transformed complex64 array."""
        N = len(signal)
        if N <= 1:
            return signal
        # Split into even and odd parts, recurse
        even = self.fft(signal[0::2])
        odd = self.fft(signal[1::2])
        # Combine: t = polar(1, -2*pi*k/N) * odd[k]
        k = np.arange(N // 2, dtype=np.float64)
        ang = (-2.0 * np.pi * k / N).astype(np.float32)
        t = (np.cos(ang) + 1j * np.sin(ang)).astype(np.complex64) * odd
        return np.concatenate([even + t, even - t]).astype(np.complex64)

    # Apply Hanning window (C++: 0.5f * (1 - cosf(2*pi*i/(N-1))))
    @staticmethod
    def applyHanningWindow(signal):
        N = len(signal)
        i = np.arange(N, dtype=np.float64)
        ang = (2.0 * np.pi * i / (N - 1)).astype(np.float32)
        window = (np.float32(0.5) * (np.float32(1.0) - np.cos(ang))) \
            .astype(np.float32)
        return (signal * window).astype(np.complex64)

    # Level detection - magnitude at a specific frequency with tolerance
    def getFrequencyMagnitude(self, audio_buffer, buffer_size, target_freq,
                              tolerance=0.05):
        sig = np.zeros(buffer_size, np.complex64)
        n = min(len(audio_buffer), buffer_size)
        sig[:n] = np.asarray(audio_buffer[:n], dtype=np.float32)

        sig = self.applyHanningWindow(sig)
        sig = self.fft(sig)

        # Frequency bounds with tolerance (float32 math, size_t truncation)
        lower_freq = np.float32(target_freq) * np.float32(1.0 - tolerance)
        upper_freq = np.float32(target_freq) * np.float32(1.0 + tolerance)
        lower_bin = int(np.float32(lower_freq * buffer_size)
                        / self.m_sampleRate)
        upper_bin = int(np.float32(upper_freq * buffer_size)
                        / self.m_sampleRate)

        # Ensure we're within valid range (first half of FFT)
        if lower_bin >= buffer_size // 2:
            lower_bin = buffer_size // 2 - 1
        if upper_bin >= buffer_size // 2:
            upper_bin = buffer_size // 2 - 1

        # Sum the magnitudes within the frequency range (inclusive)
        mags = np.abs(sig[lower_bin:upper_bin + 1]).astype(np.float32)
        return float(np.float32(mags.sum(dtype=np.float32)))

    # Pitch detection: peak bin + parabolic interpolation
    def detectPitch(self, audio_buffer, buffer_size):
        sig = np.zeros(buffer_size, np.complex64)
        n = min(len(audio_buffer), buffer_size)
        sig[:n] = np.asarray(audio_buffer[:n], dtype=np.float32)
        sig = self.applyHanningWindow(sig)
        sig = self.fft(sig)
        return self.findInterpolatedFrequency(sig, float(self.m_sampleRate))

    @staticmethod
    def findInterpolatedFrequency(fft_data, sample_rate):
        N = len(fft_data)
        # Peak of |z|^2 over bins 1 .. N/2-2 (C++ loop bounds)
        power = np.abs(fft_data) ** 2
        max_mag = 0.0
        max_index = 0
        for i in range(1, N // 2 - 1):
            if power[i] > max_mag:
                max_mag = power[i]
                max_index = i

        if max_index < 1 or max_index >= N // 2 - 1:
            return float(max_index) * sample_rate / N

        mag0 = np.sqrt(power[max_index - 1])
        mag1 = np.sqrt(power[max_index])
        mag2 = np.sqrt(power[max_index + 1])

        denom = mag0 - 2.0 * mag1 + mag2
        with np.errstate(divide="ignore", invalid="ignore"):
            peak_shift = 0.5 * (mag0 - mag2) / denom
        if np.isnan(peak_shift):
            return float(max_index) * sample_rate / N
        return (float(max_index) + float(peak_shift)) * sample_rate / N
