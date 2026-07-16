"""Feed one board's decoded PCM batches through the level pipeline.

Bridges Stream.batches() (arbitrary-length (N, 2) float32 batches) to the
detector's per-64-sample-block events, keeping the board sample clock:
t_us for a block is the count of 96 kHz samples consumed up to and
including that block (the firmware's System::GetUs equivalent,
sample-accurate within the board)."""
import numpy as np

from hardware.pinger import detector


def pump_batches(batches, first_channel, on_levels, on_pcm=None,
                 rate=96000):
    block = detector.kFftSize
    rem = np.zeros((0, 2), np.float32)
    samples_done = 0
    for pcm, batch_rate, _bits in batches:
        if batch_rate != rate:      # frames from before the rate switch
            continue
        if on_pcm is not None:
            on_pcm(pcm)
        pcm = np.concatenate([rem, pcm]) if len(rem) else pcm
        n_blocks = len(pcm) // block
        rem = pcm[n_blocks * block:]
        if not n_blocks:
            continue
        cut = pcm[:n_blocks * block]
        blocks_a = np.ascontiguousarray(cut[:, 0]).reshape(n_blocks, block)
        blocks_b = np.ascontiguousarray(cut[:, 1]).reshape(n_blocks, block)
        lv_a = detector.compute_levels(blocks_a, first_channel)
        lv_b = detector.compute_levels(blocks_b, first_channel + 1)
        for b in range(n_blocks):
            samples_done += block
            t_us = samples_done / rate * 1e6
            on_levels(first_channel, (float(lv_a[b]), float(lv_b[b])), t_us)
