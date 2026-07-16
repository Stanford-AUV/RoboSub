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
baseThreshold = 0.2               # Base threshold for frequency detection

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


class PingerDetector:
    """Line-faithful port of master_ros.cpp's main-loop state machine.

    One call per 64-sample block per board; each board covers two channels
    (first_channel 0 or 2) and supplies its own sample-clock time in us.
    The pinger fires a few ms every ~2 s: the FIRST channel to cross
    threshold opens a collection window; over withinThresholdUs we record
    which channels fire and in what order, then vote front {0,3} vs back
    {1,2}, tie broken by the earliest arrival. The decision is sticky and
    starts as back (False), exactly like the firmware.
    """

    def __init__(self):
        self.direction_front = False   # latched decision (False = back)
        self.levels = [0.0, 0.0, 0.0, 0.0]
        self._measuring = False        # inside a collection window
        self._armed = False            # quiet long enough to measure
        self._collect_start_us = 0.0
        self._fired = [False, False, False, False]
        self._order = []               # channel indices in arrival order
        self._was_above = [False, False, False, False]
        # cpp: lastCrossMs = System::GetNow() at boot -> arming needs
        # offThresholdMs of quiet from stream start too.
        self._last_cross_us = 0.0

    def process_block_levels(self, first_channel, pair_levels, t_us):
        chans = (first_channel, first_channel + 1)
        for ch, lvl in zip(chans, pair_levels):
            self.levels[ch] = float(lvl)

        # Rising-edge detection (cpp loops i=0..3 each cycle; only this
        # board's two channels can have changed, and within one block the
        # lower channel index is checked first, like the cpp loop order).
        for ch in chans:
            is_above = self.levels[ch] >= baseThreshold
            if is_above and not self._was_above[ch]:
                self._last_cross_us = t_us
                if self._armed and not self._measuring:
                    # First detection of a new ping -> start collecting
                    self._measuring = True
                    self._armed = False
                    self._collect_start_us = t_us
                    self._fired = [False, False, False, False]
                    self._order = []
                if self._measuring and not self._fired[ch]:
                    self._fired[ch] = True
                    self._order.append(ch)
            self._was_above[ch] = is_above

        # Close the collection window and decide direction
        if self._measuring and (t_us - self._collect_start_us
                                >= withinThresholdUs):
            front = sum(1 for i in range(4)
                        if self._fired[i] and i in FRONT_CHANNELS)
            back = sum(1 for i in range(4)
                       if self._fired[i] and i not in FRONT_CHANNELS)
            if front > back:
                self.direction_front = True
            elif back > front:
                self.direction_front = False
            else:   # tie -> earliest arrival
                self.direction_front = self._order[0] in FRONT_CHANNELS
            self._measuring = False   # disarmed until quiet again

        # Re-arm once the array has been quiet long enough
        if (not self._measuring and not self._armed
                and t_us - self._last_cross_us >= offThresholdMs * 1000.0):
            self._armed = True
