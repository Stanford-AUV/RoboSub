"""Render the pinger branch-listen debug plot.

Saved into the audio session dir (next to raw/ch*/N.wav) by the daisy node
when planning closes its listen window. The x axis is the BOARD SAMPLE
CLOCK in seconds -- the same clock the recorder writes the WAVs with -- so
a spike at x = 12.34 s sits at sample 12.34 * 96000 of that channel's
current WAV fragment.
"""
import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt

_COLORS = {0: "b", 1: "r", 2: "m", 3: "c"}


def render_debug(path, boards, threshold, decision_front,
                 front_channels=frozenset({0, 3}), detections=()):
    """boards: {first_channel: (times_s, levels_a, levels_b)}, already
    sliced to the listen window, times in that board's sample clock.
    detections: [(t_s, front_bool)] -- every decided ping in the window,
    marked and labeled FRONT/BACK. Returns path."""
    fig, ax = plt.subplots(figsize=(12, 6))
    tmin, tmax = None, None
    for first in sorted(boards):
        t, la, lb = boards[first]
        for ch, ys in ((first, la), (first + 1, lb)):
            side = "front" if ch in front_channels else "back"
            ax.plot(t, ys, _COLORS[ch] + "-", linewidth=0.9,
                    label=f"ch{ch} ({side})")
        if len(t):
            tmin = t[0] if tmin is None else min(tmin, t[0])
            tmax = t[-1] if tmax is None else max(tmax, t[-1])
    ax.axhline(threshold, color="k", linestyle="--", linewidth=0.8,
               label="threshold")
    # One labeled marker per decided ping (same logic as /pinger).
    for t, front in detections:
        color = "#0a0" if front else "#d00"
        ax.axvline(t, color=color, linewidth=1.2, alpha=0.7)
        ax.text(t, 1.005, "FRONT" if front else "BACK",
                transform=ax.get_xaxis_transform(), color=color,
                fontsize=9, fontweight="bold", ha="center", va="bottom",
                clip_on=False)
    if tmin is not None and tmax > tmin:
        ax.set_xlim(tmin, tmax)
    ax.set_ylim(-0.05, 1.05)
    ax.set_xlabel("Time into board stream (s) = position in raw/ch*/*.wav")
    ax.set_ylabel("Normalized level")
    ax.set_title("Pinger branch listen window -> "
                 + ("FRONT" if decision_front else "BACK"))
    ax.grid(True)
    ax.legend(fontsize="small")
    fig.tight_layout()
    fig.savefig(path, dpi=90)
    plt.close(fig)
    return path
