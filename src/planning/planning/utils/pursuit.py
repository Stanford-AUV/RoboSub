"""Pure pursuit-stepping math for go_to_object segments (no ROS)."""

import numpy as np


def pursuit_target(goal, from_pos, standoff):
    """Point `standoff` meters short of goal along the approach line, and the
    yaw (deg) that faces the goal. If already inside the standoff sphere,
    hold position (backing off isn't worth the thrash) but still face it."""
    goal = np.asarray(goal, dtype=float)
    from_pos = np.asarray(from_pos, dtype=float)
    d = goal - from_pos
    yaw = float(np.degrees(np.arctan2(d[1], d[0])))
    dist = float(np.linalg.norm(d))
    if dist <= standoff or dist < 1e-9:
        return from_pos.copy(), yaw
    return goal - d / dist * standoff, yaw


def step_toward(current, target, v_max, dt):
    """Advance current toward target by at most v_max*dt, never overshooting."""
    current = np.asarray(current, dtype=float)
    target = np.asarray(target, dtype=float)
    d = target - current
    dist = float(np.linalg.norm(d))
    max_step = v_max * dt
    if dist <= max_step or dist < 1e-9:
        return target.copy()
    return current + d / dist * max_step


def arrival_pos(ekf_pos, cmd_pos):
    """Position to judge stage arrival from: the sub's real (EKF) position
    when odometry is alive, else the commanded position so a dead EKF can't
    wedge the run."""
    return cmd_pos if ekf_pos is None else ekf_pos


def arrived(current, target, tol):
    return float(np.linalg.norm(np.asarray(target) - np.asarray(current))) <= tol
