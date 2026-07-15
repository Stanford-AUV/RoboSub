"""Lock-on gate tests for DetectionFilter (pure python, no ROS)."""

import numpy as np

from perception.utils.object_world import DetectionFilter


SUB = np.zeros(3)


def make_filter(**kw):
    defaults = dict(window=5, alpha=0.3, max_jump_m=3.0, stale_sec=2.0,
                    confirm_hits=10, confirm_window_sec=1.0,
                    lock_radius_min_m=0.05, lock_radius_frac=0.03)
    defaults.update(kw)
    return DetectionFilter(**defaults)


def feed(f, points, t0=0.0, dt=0.05, sub_pos=SUB):
    t = t0
    for p in points:
        f.update(p, t, sub_pos)
        t += dt
    return t


def test_flicker_never_locks():
    f = make_filter()
    # 3-frame blips at scattered spots, spaced over several seconds
    rng = np.random.default_rng(0)
    t = 0.0
    for _ in range(6):
        spot = rng.uniform(-3, 3, 3)
        for _ in range(3):
            f.update(spot, t, SUB)
            t += 0.05
        t += 1.5  # gap: window empties between blips
    assert not f.locked
    assert f.get(t) is None


def test_steady_stream_locks():
    f = make_filter()
    goal = np.array([2.0, 0.0, -1.0])
    rng = np.random.default_rng(1)
    pts = [goal + rng.uniform(-0.02, 0.02, 3) for _ in range(10)]
    t_end = feed(f, pts)
    assert f.locked
    got = f.get(t_end)
    assert got is not None
    assert np.linalg.norm(got - goal) < 0.05


def test_locked_ignores_far_burst():
    f = make_filter()
    goal = np.array([2.0, 0.0, -1.0])
    t_end = feed(f, [goal] * 10)
    assert f.locked
    # burst of false positives 1 m away must not drag the goal
    t_end = feed(f, [goal + np.array([1.0, 0.0, 0.0])] * 5, t0=t_end)
    got = f.get(t_end)
    assert np.linalg.norm(got - goal) < 0.05


def test_rejected_outliers_do_not_refresh_staleness():
    f = make_filter()
    goal = np.array([2.0, 0.0, -1.0])
    t_end = feed(f, [goal] * 10)  # ends at 0.5
    # only scattered outliers for the next 3 s -> they are rejected while
    # locked and too inconsistent to re-confirm, so the lock must go stale
    rng = np.random.default_rng(3)
    outliers = [goal + rng.uniform(-3, 3, 3) + np.array([1.5, 0, 0])
                for _ in range(60)]
    feed(f, outliers, t0=t_end)
    assert f.get(t_end + 3.0) is None


def test_stale_requires_reconfirmation():
    f = make_filter()
    goal = np.array([2.0, 0.0, -1.0])
    t_end = feed(f, [goal] * 10)
    t = t_end + 5.0  # > stale_sec
    assert f.get(t) is None
    assert not f.locked
    # a single new detection must not immediately publish again
    f.update(goal, t, SUB)
    assert f.get(t) is None
    # full re-confirmation relocks
    t_end2 = feed(f, [goal] * 10, t0=t + 0.05)
    assert f.locked
    assert f.get(t_end2) is not None


def test_range_scaled_radius():
    rng = np.random.default_rng(2)
    # at 4 m range, radius = 0.12 m -> +/-4 cm jitter locks
    f = make_filter()
    goal_far = np.array([4.0, 0.0, 0.0])
    pts = [goal_far + rng.uniform(-0.04, 0.04, 3) for _ in range(12)]
    feed(f, pts)
    assert f.locked
    # at 0.5 m range, radius floor = 0.05 m -> +/-8 cm jitter must NOT lock
    f2 = make_filter()
    goal_near = np.array([0.5, 0.0, 0.0])
    pts2 = [goal_near + rng.uniform(-0.08, 0.08, 3) for _ in range(12)]
    feed(f2, pts2)
    assert not f2.locked


def test_update_backcompat_without_sub_pos():
    # sub_pos omitted -> radius floor applies; steady stream still locks
    f = make_filter()
    goal = np.array([2.0, 0.0, -1.0])
    t = 0.0
    for _ in range(10):
        f.update(goal, t)
        t += 0.05
    assert f.locked
