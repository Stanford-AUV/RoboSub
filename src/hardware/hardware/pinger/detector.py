"""Exact host-side port of gen2/daisyseed_firmware/master_ros.cpp.

The boards run stream_audio.cpp (raw 96 kHz PCM over USB); this module does
what master_ros.cpp did on-board: per-64-sample-block FFT magnitude at the
target frequency -> clamp -> normalize -> rising-edge arrival ordering ->
front/back majority vote with a sticky latched decision.

Forced deviations from the firmware, both documented in the spec
(docs/superpowers/specs/2026-07-16-pinger-detection-design.md):
- GAIN: stream_audio bakes kGain = 100 into every sample -- the exact x100
  `multiplier` the firmware applies in its audio callback -- so samples
  arrive already multiplied and it is NOT applied again here.
- CLOCK: System::GetNow()/GetUs() polling becomes the per-board sample
  clock (samples_processed / rate), sample-accurate within a board; the two
  boards' clocks are merged without cross-board offset correction (each
  board carries one front and one back hydrophone).
"""
import numpy as np

from hardware.pinger.fft_library import FFTLibrary

# ////////////////////////////// Competition Configuration (WE CAN CHANGE) /////////////////////////////////////////
# # Hydrophone normalization (manually calibrate)
# hydrophone_0_max = 4.0
# hydrophone_1_max = 4.0
# hydrophone_2_max = 4.0          # ch 2/3 from slave.cpp: the host computes
# hydrophone_3_max = 4.0          # them from raw PCM, so they need maxes too
#
# # FFT
# kFftSize = 64                   # Higher = better frequency resolution
# kBlockSize = 64                 # Block size for audio processing
#
# # RMS
# multiplier = 100                # Amplification of signal (per sample)
#
# # Frequency Detection
# targetFrequency = 25000.0       # Target frequency to detect
# frequencyTolerance = 0.01       # Tolerance for frequency detection
# baseThreshold = 0.04            # Base threshold for frequency detection
#
# # Ping Detection (pinger fires a few ms every ~2 s, periodic)
# offThresholdMs = 1000           # Silence gap (ms) that re-arms a measurement
# withinThresholdUs = 3000        # Collection-window length (us) after the first detection

# ////////////////////////////// Testing Configuration (WE CAN CHANGE) /////////////////////////////////////////
# Hydrophone normalization (manually calibrate)
hydrophone_0_max = 4.0
hydrophone_1_max = 4.0
hydrophone_2_max = 4.0            # ch 2/3 from slave.cpp: the host computes
hydrophone_3_max = 4.0            # them from raw PCM, so they need maxes too

# FFT
kFftSize = 64                     # Higher = better frequency resolution
kBlockSize = 64                   # Block size for audio processing

# RMS
multiplier = 100                  # Baked into stream_audio samples (kGain) -- NOT reapplied here

# Frequency Detection
targetFrequency = 1046.0          # Target frequency to detect
frequencyTolerance = 0.01         # Tolerance for frequency detection
baseThreshold = 0.1               # Base threshold for frequency detection

# Ping Detection (pinger fires a few ms every ~2 s, periodic)
offThresholdMs = 1000             # Silence gap (ms) that re-arms a measurement
withinThresholdUs = 3000          # Collection-window length (us) after the first detection

# Output
kPrintIntervalMs = 200            # Sticky front/back is published every this many ms

# ////////////////////////////// Internal (DO NOT CHANGE) ///////////////////////////////////
SAMPLE_RATE = 96000.0             # stream_audio codec rate the node requests
HYDROPHONE_MAXES = (hydrophone_0_max, hydrophone_1_max,
                    hydrophone_2_max, hydrophone_3_max)
# Wiring: front = hydrophones 0 and 3, back = 1 and 2 (isFront in the cpp).
FRONT_CHANNELS = frozenset({0, 3})

_fft = FFTLibrary(SAMPLE_RATE)


def compute_levels(blocks, channel):
    """Normalized 0..1 levels for one channel's (B, 64) float32 blocks.

    Firmware equivalent: getFrequencyMagnitude -> clamp to the channel max
    -> divide by it (master_ros.cpp lines 189-204).
    """
    mags = _fft.getFrequencyMagnitudeBatch(
        blocks, kFftSize, targetFrequency, frequencyTolerance)
    mx = np.float32(HYDROPHONE_MAXES[channel])
    return np.minimum(mags, mx) / mx
