#!/usr/bin/env python3
"""Allan variance analysis of a static IMU rosbag2 recording.

Reads sensor_msgs/Imu from a bag, computes the overlapping Allan deviation
for each gyro and accel axis, extracts the standard noise parameters, and
prints EKF-ready covariance numbers.

Extracted per axis:
  N  angle/velocity random walk (white noise density), slope -1/2, read at tau=1s
  B  bias instability, minimum of the curve * 0.664
  K  rate/acceleration random walk, slope +1/2, read at tau=3s on the fit

Usage:
  python allan_analysis.py data/imu_static_YYYYmmdd_HHMMSS [--topic /imu/data]

Outputs (next to the bag):
  <bag>_allan_gyro.png, <bag>_allan_accel.png, <bag>_allan_results.txt
"""

import argparse
import os
import sys

import numpy as np
import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
from tqdm import tqdm


# ---------------------------------------------------------------- bag reading

def read_imu_bag(bag_path: str, topic: str):
    """Return (t, gyro Nx3, accel Nx3) arrays from a rosbag2 directory."""
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from sensor_msgs.msg import Imu

    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=bag_path, storage_id=""),
        rosbag2_py.ConverterOptions("", ""),
    )
    reader.set_filter(rosbag2_py.StorageFilter(topics=[topic]))

    ts, gyro, accel = [], [], []
    with tqdm(desc="reading bag", unit=" msgs") as bar:
        while reader.has_next():
            _, data, t_ns = reader.read_next()
            msg = deserialize_message(data, Imu)
            # prefer header stamp; fall back to bag receive time
            t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
            if t == 0.0:
                t = t_ns * 1e-9
            ts.append(t)
            gyro.append(
                (msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z)
            )
            accel.append(
                (
                    msg.linear_acceleration.x,
                    msg.linear_acceleration.y,
                    msg.linear_acceleration.z,
                )
            )
            bar.update(1)

    if not ts:
        sys.exit(f"no messages on {topic} in {bag_path}")
    return np.asarray(ts), np.asarray(gyro), np.asarray(accel)


# ---------------------------------------------------------------- allan math

def overlapping_allan_deviation(x: np.ndarray, fs: float, taus: np.ndarray):
    """Overlapping Allan deviation of rate signal x sampled at fs.

    Uses the cumulative-sum (theta) formulation:
      AVAR(tau) = < (theta[k+2m] - 2 theta[k+m] + theta[k])^2 > / (2 tau^2)
    """
    theta = np.concatenate(([0.0], np.cumsum(x) / fs))  # integrated signal
    n = len(theta)
    adev = np.empty(len(taus))
    for i, tau in enumerate(tqdm(taus, desc="allan", leave=False)):
        m = int(round(tau * fs))
        avar = np.mean(
            (theta[2 * m :] - 2.0 * theta[m : n - m] + theta[: n - 2 * m]) ** 2
        ) / (2.0 * tau * tau)
        adev[i] = np.sqrt(avar)
    return adev


def make_taus(fs: float, duration: float, points: int = 200):
    """Log-spaced averaging times from 2 samples up to duration/9,
    snapped to integer sample counts."""
    m = np.unique(
        np.round(
            np.logspace(np.log10(2), np.log10(duration * fs / 9.0), points)
        ).astype(int)
    )
    return m / fs


def fit_params(taus: np.ndarray, adev: np.ndarray):
    """Extract N (slope -1/2), B (minimum * 0.664), K (slope +1/2)."""
    logt, loga = np.log10(taus), np.log10(adev)

    def line_at(target_slope, tau_ref):
        # local slope of the curve; pick region closest to the target slope
        slope = np.gradient(loga, logt)
        idx = np.argmin(np.abs(slope - target_slope))
        # least-squares line with fixed slope through a window around idx
        lo, hi = max(0, idx - 5), min(len(taus), idx + 6)
        b = np.mean(loga[lo:hi] - target_slope * logt[lo:hi])
        return 10.0 ** (target_slope * np.log10(tau_ref) + b), idx

    N, n_idx = line_at(-0.5, 1.0)  # white noise: adev = N/sqrt(tau), read at tau=1s
    K, k_idx = line_at(+0.5, 3.0)  # random walk: adev = K*sqrt(tau/3), read at tau=3s
    i_min = int(np.argmin(adev))
    B = adev[i_min] * 0.664
    return {
        "N": N,
        "B": B,
        "K": K,
        "tau_min": taus[i_min],
        "n_idx": n_idx,
        "k_idx": k_idx,
        "i_min": i_min,
    }


# ---------------------------------------------------------------- plotting

def plot_sensor(taus, adevs, params, labels, unit, title, out_png):
    fig, ax = plt.subplots(figsize=(9, 6))
    colors = ["C0", "C1", "C2"]
    for adev, p, lab, c in zip(adevs, params, labels, colors):
        ax.loglog(taus, adev, c, label=f"{lab}: N={p['N']:.3e} B={p['B']:.3e}")
        ax.loglog(taus, p["N"] / np.sqrt(taus), c, ls="--", lw=0.8, alpha=0.6)
        ax.loglog(taus[p["i_min"]], adev[p["i_min"]], c + "o", ms=5)
    ax.set_xlabel("averaging time tau [s]")
    ax.set_ylabel(f"Allan deviation [{unit}]")
    ax.set_title(title)
    ax.grid(True, which="both", alpha=0.3)
    ax.legend(fontsize=8)
    fig.tight_layout()
    fig.savefig(out_png, dpi=150)
    plt.close(fig)


# ---------------------------------------------------------------- main

def analyze(t, gyro, accel, out_prefix):
    dt = np.diff(t)
    fs = 1.0 / np.median(dt)
    duration = t[-1] - t[0]
    gaps = int(np.sum(dt > 3.0 * np.median(dt)))
    lines = [
        f"samples: {len(t)}   rate: {fs:.2f} Hz   duration: {duration/3600:.2f} h",
        f"timestamp gaps (>3x median dt): {gaps}",
        "",
    ]
    if duration < 3600:
        lines.append("WARNING: <1h of data; bias instability estimates will be poor.")

    taus = make_taus(fs, duration)
    results = {}
    for name, data, unit, nu, bu in [
        ("gyro", gyro, "rad/s", "rad/s/sqrt(Hz)", "rad/s"),
        ("accel", accel, "m/s^2", "m/s^2/sqrt(Hz)", "m/s^2"),
    ]:
        adevs, params = [], []
        for ax_i, lab in enumerate("xyz"):
            adev = overlapping_allan_deviation(data[:, ax_i], fs, taus)
            p = fit_params(taus, adev)
            adevs.append(adev)
            params.append(p)
        results[name] = params
        plot_sensor(
            taus, adevs, params, ["x", "y", "z"], unit,
            f"Allan deviation — {name}", f"{out_prefix}_allan_{name}.png",
        )
        lines.append(f"=== {name} ===")
        for lab, p in zip("xyz", params):
            lines.append(
                f"  {lab}: N={p['N']:.4e} {nu}   "
                f"B={p['B']:.4e} {bu} (tau_min={p['tau_min']:.0f}s)   "
                f"K={p['K']:.4e}"
            )
        lines.append("")

    # EKF-ready numbers: worst axis, discrete per-sample variance = N^2 * fs
    g_N = max(p["N"] for p in results["gyro"])
    a_N = max(p["N"] for p in results["accel"])
    g_B = max(p["B"] for p in results["gyro"])
    lines += [
        "=== EKF-ready (worst axis) ===",
        f"gyro noise density  N = {g_N:.4e} rad/s/sqrt(Hz)",
        f"accel noise density N = {a_N:.4e} m/s^2/sqrt(Hz)",
        f"gyro bias instability B = {g_B:.4e} rad/s",
        "",
        f"discrete per-sample variances at fs={fs:.1f} Hz (variance = N^2 * fs):",
        f"  angular velocity variance: {g_N**2 * fs:.4e}  (rad/s)^2   -> sensors.yaml angular cov diag",
        f"  linear accel    variance: {a_N**2 * fs:.4e}  (m/s^2)^2  -> sensors.yaml accel cov diag",
        "",
        "static gyro bias (mean over whole log) [rad/s]:",
        f"  {np.mean(gyro, axis=0).tolist()}   -> GYRO_BIAS in imu.py",
        "static accel mean [m/s^2] (includes gravity):",
        f"  {np.mean(accel, axis=0).tolist()}",
    ]
    report = "\n".join(lines)
    with open(f"{out_prefix}_allan_results.txt", "w") as f:
        f.write(report + "\n")
    return report


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("bag", help="rosbag2 directory")
    ap.add_argument("--topic", default="/imu/data")
    args = ap.parse_args()

    t, gyro, accel = read_imu_bag(args.bag, args.topic)
    prefix = os.path.join(args.bag.rstrip("/"))
    report = analyze(t, gyro, accel, prefix)
    print(report)
    print(f"\nplots: {prefix}_allan_gyro.png, {prefix}_allan_accel.png")


if __name__ == "__main__":
    main()
