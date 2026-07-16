"""PingerDetector state machine vs master_ros.cpp semantics.

Timeline helpers speak microseconds. BLOCK_US is one 64-sample block at
96 kHz (~667 us) -- the granularity real events arrive at.
"""
from hardware.pinger.detector import PingerDetector

BLOCK_US = 64 / 96000.0 * 1e6
HI = 0.5      # above baseThreshold = 0.1
LO = 0.0


def quiet(det, start_us, ms=1100.0, step_us=10_000.0):
    """Feed silent events until start_us + ms; returns the next t_us."""
    t = start_us
    while t < start_us + ms * 1000.0:
        det.process_block_levels(0, (LO, LO), t)
        det.process_block_levels(2, (LO, LO), t)
        t += step_us
    return t


def edge(det, ch, t_us, level=HI):
    """Rising edge on one channel (its pair partner stays low)."""
    first = 0 if ch in (0, 1) else 2
    pair = (level, LO) if ch % 2 == 0 else (LO, level)
    det.process_block_levels(first, pair, t_us)


def drop(det, t_us):
    """All four channels back below threshold."""
    det.process_block_levels(0, (LO, LO), t_us)
    det.process_block_levels(2, (LO, LO), t_us)


def test_boot_direction_is_back():
    assert PingerDetector().direction_front is False


def test_edge_before_armed_is_ignored():
    det = PingerDetector()
    edge(det, 0, 500_000.0)             # 0.5 s in: not armed yet
    drop(det, 501_000.0)
    quiet(det, 502_000.0)
    assert det.direction_front is False  # nothing was measured


def test_front_majority():
    det = PingerDetector()
    t = quiet(det, 0.0)
    edge(det, 0, t)                                   # front
    edge(det, 3, t + BLOCK_US)                        # front
    edge(det, 1, t + 2 * BLOCK_US)                    # back
    drop(det, t + 4 * BLOCK_US)
    quiet(det, t + 5 * BLOCK_US)                      # close + settle
    assert det.direction_front is True


def test_back_majority():
    det = PingerDetector()
    t = quiet(det, 0.0)
    edge(det, 1, t)
    edge(det, 2, t + BLOCK_US)
    drop(det, t + 4 * BLOCK_US)
    quiet(det, t + 5 * BLOCK_US)
    assert det.direction_front is False


def test_tie_breaks_by_earliest_arrival():
    det = PingerDetector()
    t = quiet(det, 0.0)
    edge(det, 1, t)                                   # back arrives first
    edge(det, 0, t + BLOCK_US)                        # front second
    drop(det, t + 4 * BLOCK_US)
    quiet(det, t + 5 * BLOCK_US)
    assert det.direction_front is False

    t = quiet(det, t + 6_000_000.0)
    edge(det, 3, t)                                   # front arrives first
    edge(det, 2, t + BLOCK_US)
    drop(det, t + 4 * BLOCK_US)
    quiet(det, t + 5 * BLOCK_US)
    assert det.direction_front is True


def test_four_way_tie_cpp_example():
    # cpp comment: order 2 1 0 3 -> back (2 front vs 2 back, earliest = 2)
    det = PingerDetector()
    t = quiet(det, 0.0)
    for i, ch in enumerate((2, 1, 0, 3)):
        edge(det, ch, t + i * BLOCK_US)
    drop(det, t + 5 * BLOCK_US)
    quiet(det, t + 6 * BLOCK_US)
    assert det.direction_front is False


def test_single_channel_ping():
    det = PingerDetector()
    t = quiet(det, 0.0)
    edge(det, 0, t)                                   # only ch0 (front)
    drop(det, t + BLOCK_US)
    quiet(det, t + 2 * BLOCK_US)
    assert det.direction_front is True


def test_edge_at_window_close_is_included():
    # cpp records the iteration's edges BEFORE checking the window close,
    # so an edge in the same event that crosses 3000 us still counts.
    det = PingerDetector()
    t = quiet(det, 0.0)
    edge(det, 1, t)                                   # back opens window
    edge(det, 0, t + 3500.0)                          # front, past 3000 us
    edge(det, 3, t + 3500.0)                          # would be front #2...
    drop(det, t + 10_000.0)
    quiet(det, t + 20_000.0)
    # ch0's event both records the edge and closes the window; ch3's edge
    # arrives after the close and is NOT part of the measurement.
    # fired = {1, 0} -> 1v1 tie -> earliest = ch1 -> back.
    assert det.direction_front is False


def test_no_retrigger_until_quiet():
    det = PingerDetector()
    t = quiet(det, 0.0)
    edge(det, 1, t)                                   # back ping decides
    drop(det, t + 4 * BLOCK_US)
    # Close the window: like the cpp's continuous polling, real streams
    # deliver events every ~667 us; tests must supply one past 3000 us.
    drop(det, t + 5000.0)
    # 200 ms later (NOT quiet for 1000 ms): a front burst must be ignored
    t2 = t + 200_000.0
    edge(det, 0, t2)
    edge(det, 3, t2 + BLOCK_US)
    drop(det, t2 + 4 * BLOCK_US)
    det.process_block_levels(0, (LO, LO), t2 + 300_000.0)
    assert det.direction_front is False
    # ...and because that burst refreshed lastCross, arming needs another
    # full quiet second; a ping right after it is also ignored.
    edge(det, 0, t2 + 900_000.0 + 4 * BLOCK_US)
    drop(det, t2 + 910_000.0)
    quiet(det, t2 + 920_000.0)
    assert det.direction_front is False


def test_sticky_across_silence():
    det = PingerDetector()
    t = quiet(det, 0.0)
    edge(det, 0, t)
    drop(det, t + BLOCK_US)
    t = quiet(det, t + 2 * BLOCK_US)
    assert det.direction_front is True
    quiet(det, t, ms=30_000.0)                        # 30 s of nothing
    assert det.direction_front is True


def test_levels_property_tracks_latest():
    det = PingerDetector()
    det.process_block_levels(0, (0.2, 0.3), 0.0)
    det.process_block_levels(2, (0.4, 0.5), 0.0)
    assert det.levels == [0.2, 0.3, 0.4, 0.5]
