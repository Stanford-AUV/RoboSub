# RoboSub IMU / Localization Debug Log

## Session 2026-07-01 — IMU "yaw/pitch drift" investigation

### Code changes made (by Claude)

1. **`src/hardware/hardware/nodes/imu.py` — orientation zeroing rewritten (Euler → quaternion)**
   - Before: converted the transformed orientation to Euler `xyz`, captured per-axis
     roll/pitch/yaw offsets on the first frame, and subtracted them each frame.
   - After: capture the inverse of the initial rotation once
     (`self.rot_init_inv = rot_final.inv()`), then `rot_zeroed = self.rot_init_inv * rot_final`.
   - Why: per-axis Euler subtraction is only valid near zero roll/pitch; when the
     vehicle is tilted it cross-couples roll/pitch into apparent yaw. Quaternion
     composition has no gimbal coupling and is memoryless (removes a constant, cannot
     itself ramp).
   - Also renamed the unused `self.q_init_inv` → `self.rot_init_inv` in `__init__`.

2. **`src/hardware/hardware/nodes/imu.py` — sensor→base transform corrected (gravity → Z)**
   - Before: `rot_base = R_sensor_to_base * rot_sensor`, then `rotate_quaternion`
     (90° about Y + flip), then a 180° about Z. Net effect: frame mis-rotated ~90°,
     gravity/heading landing on the **pitch** axis.
   - After: a single remount rotation
     `rot_final = rot_sensor * R.from_matrix(R_sensor_to_base).inv()`
     (`world_from_base = world_from_sensor * sensor_from_base`).
   - Verified numerically against the accel ground-truth: gravity → base **Z**
     (`[0.8, 0.1, 9.8]`), **heading → yaw**, tilt axes → roll/pitch.
   - Removed the now-unused `rotate_quaternion` method.
   - **Still to confirm physically:** which horizontal axis is forward (roll vs pitch)
     and the signs — the data pins vertical, not forward. Run the 60 s axis test below.

3. **`launch_sub.sh` — new full-stack bring-up script (repo root)**
   - Launches `ros2 launch main` for **hardware → localization → control → planning**
     (2 s stagger so hardware/localization come up first).
   - On **Ctrl+C**: SIGINT each launch, wait for them to exit, then send neutral PWM
     `1500` × 8 **directly to the Arduino serial port** (`/dev/ttyACM0` @ 9600) so the
     thrusters shut off even after the ROS nodes are gone (the arduino node holds the
     port exclusively, so the script waits for it to release before writing).
   - Sources ROS Jazzy + workspace `install/setup.bash`; wraps the ROS setup sourcing
     in `set +u` (ROS setup scripts reference unbound vars).

### Config changes
- `src/hardware/hardware/sensors.yaml`: `imu_0.accel` `0 → 123` (user; enable `/accel` so the
  acceleration panel populates in `sensors_plot`).
- `src/hardware/hardware/sensors.yaml`: `imu_0.R_sensor_to_base` rotated **+90° about base-Z**
  → `[[-1,0,0],[0,0,1],[0,1,0]]`. The physical axis test showed roll/pitch swapped ("inverted
  cardinalities") — vertical/yaw were correct but forward was 90° off (unobservable from a
  static sub). This is now the single mounting knob. If roll/pitch read sign-inverted, use the
  −90° variant `[[1,0,0],[0,0,-1],[0,1,0]]` (both keep gravity→Z, heading→yaw).

### Root-cause diagnosis

**Hardware:** Xsens **MTi-200 VRU** — no heading reference; yaw is unreferenced,
gyro-integrated, and drifts by design. The DVL (Wayfinder) reports **no heading**
(velocity + altitude only). The MTi-200 filter is **VRU-only** — it cannot produce an
onboard magnetometer/north-referenced heading.

**The frame is rotated.** The IMU is physically mounted rotated relative to `base_link`,
and the transform chain in `imu.py` (`rotate_quaternion`: 90° about Y + flip, then a
180° about Z) leaves the output frame mis-rotated ~90°, so **gravity/vertical lands on
the output Y (pitch) axis instead of Z (yaw)**.
- Confirmed from the accel panel: gravity reads **~9.8 on Y** (should be on Z), with a
  ~0.8 leak onto Z (a ~5° frame/mount tilt). `|specific force| ≈ g` throughout → there is
  **no real acceleration**; the "weird Z acceleration" is gravity projected onto a tilted axis.
- Simulation using the mounting derived from the accel reading: a 6° **heading** rotation
  comes out as ~6° of output **pitch**. So the unreferenced **heading drift appears on the
  PITCH channel**, which is why "yaw" looked stable while "pitch" crept.

**Ramp vs. offset (the ramp→level experiment):**
- The **constant** part (static IMU-frame rotation) → a fixed pitch **offset**.
- The **ramp** appears **only when the sub is tilted**: a fixed tilt combined with slowly
  drifting heading sweeps the tilt vector through the body pitch axis
  (ramp rate ≈ tilt_angle × heading_drift_rate). Observed pitch ramp ≈ 0.05 rad / 700 s
  with a ~5° tilt ⇒ implied heading drift ≈ **~2.8°/min** (a plausible VRU rate).
- Taking the sub off the ramp removed the tilt (Z-accel 0.8 → 0) and the pitch ramp went
  **flat** — confirming the ramp is tilt-coupled, not a free-running pitch drift.

**Summary:** it is the *same* heading drift throughout; the rotated IMU frame routes it
onto the pitch label, and vehicle tilt is what couples it into a visible ramp.

### Correction (after roll/pitch swap + yaw→roll/pitch coupling reported)
- **Root cause of the coupling:** zeroing did `rot_zeroed = rot_init_inv * rot_final` (zero
  ALL axes to the startup attitude). A non-level init makes the reference frame tilted, so a
  real yaw bleeds into roll/pitch. **Fixed:** yaw-only zeroing — capture initial heading
  `yaw0 = rot_final.as_euler("ZYX")[0]`, `rot_init_inv = R.from_euler("z", -yaw0)`; roll/pitch
  now stay absolute (gravity-referenced), only heading is zeroed.
- **Reverted** the `R_sensor_to_base` +90° guess back to `[[0,0,1],[1,0,0],[0,1,0]]` — the roll/pitch
  swap observations were contaminated by the coupling; determine forward cleanly AFTER the
  coupling fix. Build is `--symlink-install`, so `.py`/`.yaml` edits are live on node **restart**
  (no full rebuild needed).

### Final mounting matrix
- Axis test (post coupling-fix): channels correct (no swap) but roll AND pitch signs were
  inverted vs right-hand FLU. C and D are the 180°-about-Z pair (same channel mapping, opposite
  roll/pitch sign) — the channel test can't distinguish them, the sign test does. Switched
  `R_sensor_to_base` C → **D = `[[0,0,-1],[-1,0,0],[0,1,0]]`**. Verified: det +1, gravity → base
  +Z, roll/pitch signs flipped, yaw sign unchanged. This also points base +X (forward) the
  correct way (C had forward reversed, which is why both signs were wrong).

### Physical axis-verification test (run after rebuilding)
With the sub roughly level and still, watch the EKF rotation panel while doing each move,
one at a time, returning to rest between:
1. **Rotate ~90° in heading (spin flat).** Expect **YAW** to move ~90°, roll/pitch ~0.
2. **Nose down / pitch it.** Expect **PITCH** to move, roll/yaw ~0.
3. **Roll it left/right.** Expect **ROLL** to move, pitch/yaw ~0.
Check the **sign** too (nose-down should be one consistent sign, etc.). If an axis is
swapped or inverted, adjust `R_sensor_to_base` in `sensors.yaml` (it is the single knob
now) and re-test.

### Proposed / not yet applied
- **Magnetometer heading pipeline** (calibration node + tilt-compensated `/imu/mag` heading,
  fused into the EKF as absolute yaw, paired with the gyro). Scoped, not yet built. Now that
  the frame is corrected, heading lives on the yaw axis where this pipeline targets it.
