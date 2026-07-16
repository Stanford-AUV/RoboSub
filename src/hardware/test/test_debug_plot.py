"""render_debug writes a readable PNG spanning exactly the listen window."""
import numpy as np

from hardware.pinger.debug_plot import render_debug


def test_render_debug_writes_png(tmp_path):
    t0 = np.linspace(100.0, 110.0, 500)   # 10 s of board-0 stream time
    t2 = np.linspace(99.5, 109.5, 500)    # board 2 clock is offset slightly
    boards = {
        0: (t0.tolist(), (0.02 + 0.8 * (np.abs(t0 - 105) < 0.05)).tolist(),
            [0.02] * 500),
        2: (t2.tolist(), [0.03] * 500, [0.01] * 500),
    }
    out = str(tmp_path / "debug.png")
    assert render_debug(out, boards, threshold=0.2, decision_front=True) == out
    header = open(out, "rb").read(8)
    assert header == b"\x89PNG\r\n\x1a\n"


def test_render_debug_empty_window(tmp_path):
    # No samples during the window (e.g. boards disconnected) must still
    # produce a file, not raise.
    out = str(tmp_path / "debug.png")
    render_debug(out, {0: ([], [], []), 2: ([], [], [])},
                 threshold=0.2, decision_front=False)
    assert open(out, "rb").read(8) == b"\x89PNG\r\n\x1a\n"


def test_render_debug_labels_detections(tmp_path):
    t = np.linspace(10.0, 20.0, 200)
    boards = {0: (t.tolist(), [0.02] * 200, [0.02] * 200)}
    out = str(tmp_path / "debug.png")
    render_debug(out, boards, threshold=0.2, decision_front=True,
                 detections=[(12.5, True), (15.0, False), (18.2, True)])
    assert open(out, "rb").read(8) == b"\x89PNG\r\n\x1a\n"
