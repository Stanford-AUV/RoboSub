"""Sliding-window series store and matplotlib drawing for the debug plots.

No ROS in here: a node feeds samples in with `Window.append()` and calls one
of the draw helpers on a timer. Keeps the plot nodes down to a topic, a field
extractor and a list of series names.
"""

import matplotlib

matplotlib.use("TkAgg")  # these are bench tools; they want a window
import matplotlib.pyplot as plt  # noqa: E402  (must follow matplotlib.use)

COLORS = ("r-", "g-", "b-", "m-", "c-", "y-")


class Window:
    """Time plus N parallel series.

    `seconds=None` keeps the whole run (what a trajectory plot wants);
    a number prunes to that trailing window.
    """

    def __init__(self, labels, seconds=10.0):
        self.labels = list(labels)
        self.seconds = seconds
        self.time = []
        self.series = [[] for _ in self.labels]

    def append(self, t, values):
        if len(values) != len(self.series):
            raise ValueError(
                f"expected {len(self.series)} values, got {len(values)}"
            )
        self.time.append(t)
        for col, v in zip(self.series, values):
            col.append(v)
        if self.seconds is None:
            return
        cutoff = t - self.seconds
        while self.time and self.time[0] < cutoff:
            self.time.pop(0)
            for col in self.series:
                col.pop(0)

    def means(self):
        """Mean of each series over the current window, or None if empty."""
        if not self.time:
            return None
        return [sum(col) / len(col) for col in self.series]

    def column(self, label):
        """The series stored under `label`."""
        return self.series[self.labels.index(label)]


def _label(ax, title, ylabel, xlabel):
    ax.set_title(title)
    ax.set_xlabel(xlabel)
    ax.set_ylabel(ylabel)
    ax.grid(True)


def make_axes(title, ylabel, xlabel="Time (s)"):
    """Interactive single-axes figure, shown non-blocking."""
    plt.ion()
    fig, ax = plt.subplots(figsize=(12, 8))
    _label(ax, title, ylabel, xlabel)
    plt.show(block=False)
    return fig, ax


def make_figure(figsize=(15, 10)):
    """Interactive empty figure for callers laying out their own subplots."""
    plt.ion()
    fig = plt.figure(figsize=figsize)
    return fig


def show(fig):
    plt.show(block=False)
    return fig


def redraw(fig, ax, window, title, ylabel, xlabel="Time (s)", show_means=False):
    """Clear and replot every series of `window` onto one axes."""
    ax.cla()
    for col, label, color in zip(window.series, window.labels, COLORS):
        ax.plot(window.time, col, color, label=label)
    if show_means:
        means = window.means()
        if means:
            for mean, label, color in zip(means, window.labels, COLORS):
                ax.axhline(
                    mean,
                    color=color[0],
                    linestyle="--",
                    label=f"avg {label}: {mean:.2f}",
                )
    _label(ax, title, ylabel, xlabel)
    if window.time:
        ax.legend()
    plt.tight_layout()
    fig.canvas.draw()
    fig.canvas.flush_events()


def draw_panel(ax, time, values, ylabel, color="r-", ylim=None):
    """One series on its own axes - for multi-panel layouts."""
    ax.cla()
    ax.plot(time, values, color)
    ax.set_xlabel("Time (s)")
    ax.set_ylabel(ylabel)
    if ylim is not None:
        ax.set_ylim(list(ylim))
    ax.grid(True)


def make_3d_axes(fig, pos, title, lim):
    """3D axes with equal, fixed limits."""
    ax = fig.add_subplot(pos, projection="3d")
    ax.set_title(title)
    set_3d_limits(ax, lim)
    return ax


def set_3d_limits(ax, lim):
    ax.set_xlabel("X")
    ax.set_ylabel("Y")
    ax.set_zlabel("Z")
    ax.set_xlim(list(lim))
    ax.set_ylim(list(lim))
    ax.set_zlim(list(lim))


def draw_trajectory(ax, xs, ys, zs, title, lim):
    """Path through 3-space with a marker on the newest point."""
    ax.cla()
    ax.plot(xs, ys, zs, "b-")
    if xs:
        ax.scatter(xs[-1], ys[-1], zs[-1], color="red", marker="o")
    ax.set_title(title)
    set_3d_limits(ax, lim)


def draw_unit_sphere(ax):
    """Faint reference wireframe for orientation plots. Draw once."""
    import numpy as np

    u, v = np.mgrid[0 : 2 * np.pi : 20j, 0 : np.pi : 10j]
    ax.plot_wireframe(
        np.cos(u) * np.sin(v),
        np.sin(u) * np.sin(v),
        np.cos(v),
        color="gray",
        alpha=0.2,
    )


def draw_frame_axes(ax, rotation_matrix, previous=None):
    """Draw a frame's X/Y/Z basis vectors as arrows.

    `rotation_matrix` is 3x3, columns are the basis vectors in world
    coordinates. Pass the previous return value back in to clear the old
    arrows (quiver artists cannot be updated in place).
    """
    for artist in previous or ():
        artist.remove()
    arrows = []
    for col, color, name in zip(range(3), "rgb", ("X", "Y", "Z")):
        arrows.append(
            ax.quiver(
                0,
                0,
                0,
                *rotation_matrix[:, col],
                color=color,
                label=f"{name}_base",
            )
        )
    return arrows
