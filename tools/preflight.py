#!/usr/bin/env python3
"""Poolside pre-flight check for the AUV: tests every sensor, motor and device
and prints a GO / NO-GO status report before the sub goes in the water.

Run it on the Orin through the wrapper (activates conda + sources the
workspace):

    ./preflight.sh                  # full HARDWARE check (stack must be STOPPED)
    ./preflight.sh --live           # stack already running: topic/rate/sanity check
    ./preflight.sh --yes            # no operator prompts (thrusters judged by current draw)
    ./preflight.sh --only teensy,thrusters
    ./preflight.sh --skip cameras,hydrophones
    ./preflight.sh --no-thrusters   # never spin a motor
    ./preflight.sh --fire-torpedoes # also fire both torpedoes (default: re-center only)

Two modes, because the hardware ports are exclusive (one process per serial
port):

  HARDWARE mode (default) talks to every device directly, without the stack:
    system      workspace built / not stale, disk, temperatures, env
    config      sensors.yaml / thrusters.yaml / cameras.yaml sanity + known gotchas
    usb         udev symlinks, permissions, xsens_mt module, hydrophone boards
    teensy      serial link, telemetry rate, battery, current, leak (humidity),
                internal temps, depth/pressure, water temp, BNO085s
    lights      light PWM command + operator visual confirmation
    torpedoes   servo re-center ack (full fire only with --fire-torpedoes)
    thrusters   each of the 8 thrusters pulsed fwd + rev; verified by current
                draw and (unless --yes) operator confirmation
    imu         brings up the Xsens driver + hardware/imu briefly, checks rates,
                quaternion validity, level, gyro bias
    dvl         Wayfinder connect, setup readback, ping for a few seconds,
                built-in-test errors, input voltage, bottom lock
    hydrophones both DaisySeed boards stream audio; rate, drops, levels, clipping
    cameras     OAK (and RealSense if configured) enumerate + grab RGB/depth

  LIVE mode (--live) is for after `./launch_sub.sh --no-thrust`: checks every
  key topic is publishing at its expected rate, that exactly one publisher owns
  /odometry/filtered and /wrench, that every expected node is up, and sanity of
  the values (telemetry thresholds, EKF finite/agreeing with the IMU).

A report is written to logs/preflight_<timestamp>.{txt,json}.
Exit code: 0 = GO, 1 = NO-GO (at least one FAIL), 2 = GO with warnings.
"""
import argparse
import datetime as _dt
import glob
import json
import math
import os
import re
import shutil
import signal
import subprocess
import sys
import time
import traceback
from pathlib import Path

REPO = Path(__file__).resolve().parent.parent
SRC = REPO / "src"

# Make the in-tree packages importable even without a sourced workspace (the
# DVL SDK and the Daisy stream reader are plain Python).
for p in (SRC / "hardware", SRC / "perception"):
    if str(p) not in sys.path:
        sys.path.append(str(p))

# =============================================================================
# Thresholds -- edit here. Values are for a 4S LiPo (thrusters.py assumes a
# 14.8 V nominal pack).
# =============================================================================
TH = {
    "battery_fail_low": 14.0,     # V  (3.5 V/cell) -- do not launch
    "battery_warn_low": 15.2,     # V  (3.8 V/cell) -- swap soon
    "battery_fail_high": 17.2,    # V  (wrong pack / bad reading)
    "idle_current_warn": 5.0,     # A at neutral PWM
    "humidity_warn": 60.0,        # %  -- condensation / slow leak
    "humidity_fail": 75.0,        # %  -- LEAK, do not launch
    "humidity_rise_warn": 2.0,    # % rise within one sample window
    "int_temp_warn": 50.0,        # C
    "int_temp_fail": 65.0,        # C
    "water_temp_min": 0.0,        # C plausibility
    "water_temp_max": 40.0,
    "surface_depth_warn": 0.25,   # m  |depth| out of the water
    "depth_noise_warn": 0.05,     # m  std at rest
    "telemetry_min_hz": 5.0,
    "thruster_current_delta": 0.25,  # A rise that proves a thruster spun
    "spin_pwm_delta": 60,         # us from neutral for the thruster pulse
    "spin_seconds": 0.8,          # per direction (keep short: props dry)
    "imu_min_hz": 100.0,          # /filter/quaternion
    "imu_level_warn_deg": 10.0,   # roll/pitch on the stand
    "imu_gyro_bias_warn": 0.05,   # rad/s at rest
    "dvl_min_hz": 2.0,
    "disk_free_warn_gb": 10.0,
    "orin_temp_warn": 75.0,       # C
    "hydro_clip": 0.98,           # |sample| considered clipped
    "hydro_dead_rms": 1e-5,       # RMS below this = dead channel
}

TEENSY_PORT = "/dev/ttyACM_teensy"
TEENSY_BAUD = 9600
THRUSTER_COUNT = 8
NEUTRAL = 1500
PWM_MIN, PWM_MAX = 1250, 1750     # same clip as hardware/arduino.py
IMU_LINK = "/dev/ttyUSB_imu"
DVL_LINK = "/dev/ttyUSB_dvl"
DVL_VID_PID = (0x0403, 0x6001)    # FTDI FT232R on the Wayfinder
THRUSTER_NAMES = [
    "top front left", "top front right", "top back left", "top back right",
    "bottom front left", "bottom front right", "bottom back left",
    "bottom back right",
]
ALL_SECTIONS = ["system", "config", "usb", "teensy", "lights", "torpedoes",
                "thrusters", "imu", "dvl", "hydrophones", "cameras"]

# =============================================================================
# Reporting
# =============================================================================
PASS, WARN, FAIL, SKIP, INFO = "PASS", "WARN", "FAIL", "SKIP", "INFO"
_COLOR = {PASS: "\033[32m", WARN: "\033[33m", FAIL: "\033[31;1m",
          SKIP: "\033[90m", INFO: "\033[36m"}
_RESET = "\033[0m"
USE_COLOR = sys.stdout.isatty()


def c(status, text=None):
    text = status if text is None else text
    return f"{_COLOR[status]}{text}{_RESET}" if USE_COLOR else text


class Report:
    def __init__(self):
        self.results = []          # dicts: section, name, status, detail
        self.section = "?"

    def begin(self, section, title):
        self.section = section
        print()
        print(c(INFO, f"== {title} " + "=" * max(0, 66 - len(title))))

    def add(self, status, name, detail=""):
        self.results.append(dict(section=self.section, name=name,
                                 status=status, detail=str(detail)))
        print(f"  [{c(status, f'{status:4}')}] {name}" +
              (f" -- {detail}" if detail else ""))
        return status

    def ok(self, name, detail=""):
        return self.add(PASS, name, detail)

    def warn(self, name, detail=""):
        return self.add(WARN, name, detail)

    def fail(self, name, detail=""):
        return self.add(FAIL, name, detail)

    def skip(self, name, detail=""):
        return self.add(SKIP, name, detail)

    def info(self, name, detail=""):
        return self.add(INFO, name, detail)

    def counts(self):
        out = {k: 0 for k in (PASS, WARN, FAIL, SKIP, INFO)}
        for r in self.results:
            out[r["status"]] += 1
        return out

    def verdict(self):
        n = self.counts()
        if n[FAIL]:
            return "NO-GO", 1
        if n[WARN]:
            return "GO (with warnings)", 2
        return "GO", 0

    def summary(self, mode, started):
        n = self.counts()
        verdict, code = self.verdict()
        lines = []
        lines.append("=" * 72)
        lines.append(f" AUV PRE-FLIGHT REPORT   {started:%Y-%m-%d %H:%M:%S}   "
                     f"mode: {mode}")
        lines.append("=" * 72)
        sections = []
        for r in self.results:
            if r["section"] not in sections:
                sections.append(r["section"])
        for s in sections:
            rs = [r for r in self.results if r["section"] == s]
            worst = (FAIL if any(r["status"] == FAIL for r in rs) else
                     WARN if any(r["status"] == WARN for r in rs) else
                     PASS if any(r["status"] == PASS for r in rs) else
                     SKIP)
            lines.append(f" {s:<12} {worst}")
        problems = [r for r in self.results if r["status"] in (FAIL, WARN)]
        if problems:
            lines.append("-" * 72)
            lines.append(" Needs attention:")
            for r in sorted(problems, key=lambda r: r["status"] != FAIL):
                lines.append(f"  {r['status']:4} [{r['section']}] {r['name']}"
                             + (f": {r['detail']}" if r["detail"] else ""))
        lines.append("-" * 72)
        lines.append(f" {n[PASS]} pass, {n[WARN]} warn, {n[FAIL]} fail, "
                     f"{n[SKIP]} skipped")
        lines.append(f" VERDICT: {verdict}")
        lines.append("=" * 72)
        return lines, verdict, code


R = Report()


def section(name, title, args):
    """Decorator-ish guard: returns False if the section should not run."""
    if args.only and name not in args.only:
        return False
    if name in args.skip:
        return False
    R.begin(name, title)
    return True


def run_safely(fn, *a, **kw):
    """A crashing check must never abort the whole report."""
    try:
        return fn(*a, **kw)
    except KeyboardInterrupt:
        raise
    except Exception as e:  # noqa: BLE001
        R.fail(f"{fn.__name__} crashed", f"{type(e).__name__}: {e}")
        if os.environ.get("PREFLIGHT_DEBUG"):
            traceback.print_exc()
        return None


def ask(args, question, default=None):
    """y/n/s operator prompt. Returns True/False, or None (skip / --yes)."""
    if args.yes:
        return None
    hint = "[y/n/s=skip]"
    while True:
        try:
            ans = input(f"      ? {question} {hint} ").strip().lower()
        except EOFError:
            return None
        if ans in ("y", "yes"):
            return True
        if ans in ("n", "no"):
            return False
        if ans in ("s", "skip", ""):
            return default


def stats(xs):
    xs = [x for x in xs if x is not None and math.isfinite(x)]
    if not xs:
        return None, None
    m = sum(xs) / len(xs)
    sd = math.sqrt(sum((x - m) ** 2 for x in xs) / len(xs))
    return m, sd


def load_yaml(path):
    import yaml
    with open(path) as f:
        return yaml.safe_load(f)


def sh(cmd, timeout=10):
    try:
        p = subprocess.run(cmd, shell=True, capture_output=True, text=True,
                           timeout=timeout)
        return p.returncode, (p.stdout + p.stderr).strip()
    except subprocess.TimeoutExpired:
        return 124, "timeout"


# =============================================================================
# Stack detection -- hardware mode must not fight running nodes for ports
# =============================================================================
STACK_PATTERNS = ["ros2 launch main", "install/lib/hardware",
                  "xsens_mti_node", "ekf_node", "install/lib/perception"]


def stack_processes():
    procs = []
    for pat in STACK_PATTERNS:
        rc, out = sh(f"pgrep -af '{pat}'")
        if rc == 0:
            for line in out.splitlines():
                if "preflight" not in line and "pgrep" not in line:
                    procs.append(line.strip())
    return sorted(set(procs))


def port_holders(port):
    if not os.path.exists(port) or not shutil.which("fuser"):
        return ""
    rc, out = sh(f"fuser {port}")
    return out.split(":", 1)[-1].strip() if rc == 0 else ""


# =============================================================================
# SYSTEM
# =============================================================================
def check_system(args):
    # Workspace built and not stale (same rule launch_sub.sh enforces).
    ws = REPO / "install" / "setup.bash"
    if not ws.exists():
        R.fail("workspace built", f"{ws} missing -- run colcon build")
    else:
        ws_m = ws.stat().st_mtime
        stale = [p for p in SRC.rglob("*.py")
                 if "/launch/" not in str(p).replace("\\", "/")
                 and p.stat().st_mtime > ws_m]
        if stale:
            R.fail("build up to date",
                   f"{len(stale)} .py newer than last build, e.g. "
                   f"{stale[0].relative_to(REPO)} -- colcon build --symlink-install")
        else:
            R.ok("build up to date")

    env = os.environ.get("CONDA_DEFAULT_ENV")
    if env == "robosub":
        R.ok("conda env", "robosub")
    else:
        R.warn("conda env", f"active env is {env!r}, expected 'robosub'")

    if os.environ.get("NO_THRUST", "0") not in ("", "0"):
        R.warn("NO_THRUST is set",
               "a launch from this shell will NOT spin thrusters")

    prof = os.environ.get("FASTRTPS_DEFAULT_PROFILES_FILE",
                          str(REPO / "config" / "fastdds_async.xml"))
    if Path(prof).exists():
        R.ok("FastDDS async profile", Path(prof).name)
    else:
        R.warn("FastDDS async profile", f"{prof} missing (publish() may block "
               "on tether drop)")

    try:
        free = shutil.disk_usage(REPO).free / 1e9
        (R.ok if free >= TH["disk_free_warn_gb"] else R.warn)(
            "disk free (bags/logs)", f"{free:.1f} GB")
    except OSError as e:
        R.warn("disk free", str(e))

    zones = sorted(glob.glob("/sys/class/thermal/thermal_zone*/temp"))
    temps = {}
    for z in zones:
        try:
            t = int(Path(z).read_text().strip()) / 1000.0
            name = Path(z).with_name("type").read_text().strip()
            if -40 < t < 150:
                temps[name] = t
        except (OSError, ValueError):
            pass
    if temps:
        hot = max(temps.values())
        detail = ", ".join(f"{k}={v:.0f}C" for k, v in sorted(temps.items()))
        (R.ok if hot < TH["orin_temp_warn"] else R.warn)("Orin temperatures",
                                                         detail)
    else:
        R.skip("Orin temperatures", "no thermal zones (not on the Orin?)")

    try:
        import serial  # noqa: F401
        R.ok("pyserial importable")
    except ImportError:
        R.fail("pyserial importable", "python -m pip install pyserial")

    if args.gpu:
        rc, out = sh("python -c \"import torch;print(torch.cuda.is_available(),"
                     "torch.cuda.get_device_name(0) if torch.cuda.is_available() "
                     "else '')\"", timeout=90)
        if rc == 0 and out.startswith("True"):
            R.ok("GPU torch", out)
        else:
            R.warn("GPU torch", out[-200:])
    else:
        R.skip("GPU torch", "pass --gpu to test (slow import)")


# =============================================================================
# CONFIG
# =============================================================================
def check_config(args):
    hw = SRC / "hardware" / "hardware"
    try:
        sy = load_yaml(hw / "sensors.yaml")
        missing = [k for k in ("imu_0", "dvl_0", "depth_0") if k not in sy]
        if missing:
            R.fail("sensors.yaml", f"missing sections {missing}")
        else:
            R.ok("sensors.yaml", f"sections: {', '.join(sy)}")
        for name, cfg in sy.items():
            m = cfg.get("R_sensor_to_base")
            if m:
                import numpy as np
                M = np.array(m, dtype=float)
                if not np.allclose(M @ M.T, np.eye(3), atol=1e-3):
                    R.fail(f"{name} R_sensor_to_base", "not a rotation matrix")
    except Exception as e:  # noqa: BLE001
        R.fail("sensors.yaml", e)

    try:
        ty = load_yaml(hw / "thrusters.yaml")
        n = len([k for k in ty if k.startswith("thruster_")])
        (R.ok if n == THRUSTER_COUNT else R.fail)(
            "thrusters.yaml", f"{n} thrusters (expect {THRUSTER_COUNT})")
    except Exception as e:  # noqa: BLE001
        R.fail("thrusters.yaml", e)

    # Known gotcha documented in src/hardware/README.md: the thrusters node
    # listens to relative 'sensors' while arduino publishes /arduino/sensors,
    # so the thrust->PWM curve never sees the real battery voltage.
    try:
        txt = (hw / "nodes" / "thrusters.py").read_text()
        if re.search(r'SensorsStamped,\s*"sensors"', txt):
            remap = "arduino/sensors" in (SRC / "main" / "launch" /
                                          "hardware.py").read_text()
            if not remap:
                R.warn("thrusters voltage topic",
                       "thrusters node subscribes 'sensors' (nothing publishes "
                       "it) -> PWM curve fixed at 14.8 V")
    except OSError:
        pass

    try:
        cy = load_yaml(SRC / "perception" / "perception" / "cameras.yaml")
        R.ok("cameras.yaml", f"cameras: {', '.join(cy)}")
    except Exception as e:  # noqa: BLE001
        R.warn("cameras.yaml", e)

    try:
        oy = load_yaml(SRC / "perception" / "perception" / "object_tracking.yaml")
        p = next(iter(oy.values()))["ros__parameters"]
        if (all(v == 0 for v in p.get("cam_translation", [0]))
                and all(v == 0 for v in p.get("cam_rpy_deg", [0]))):
            R.warn("camera extrinsics",
                   "object_tracking.yaml cam_translation/cam_rpy_deg still "
                   "identity -- measure before a detection run")
        else:
            R.ok("camera extrinsics")
    except Exception as e:  # noqa: BLE001
        R.warn("object_tracking.yaml", e)


# =============================================================================
# USB / udev
# =============================================================================
def check_usb(args):
    for link, what, required in ((TEENSY_PORT, "Teensy", True),
                                 (IMU_LINK, "Xsens IMU", True),
                                 (DVL_LINK, "DVL", False)):
        if os.path.exists(link):
            real = os.path.realpath(link)
            if os.access(link, os.R_OK | os.W_OK):
                R.ok(f"{what} device", f"{link} -> {real}")
            else:
                R.fail(f"{what} device", f"{link} not read/writable "
                       "(sudo ./ports.sh or add user to dialout)")
            holders = port_holders(link)
            if holders:
                R.warn(f"{what} port busy", f"held by PID {holders}")
        elif required:
            R.fail(f"{what} device", f"{link} missing (unplugged / no power / "
                   "udev rule)")
        else:
            dvl = find_dvl_port()
            if dvl:
                R.warn("DVL device", f"no {DVL_LINK} udev symlink; found FTDI "
                       f"at {dvl} (see ORIN_SETUP.md DVL udev)")
            else:
                R.fail("DVL device", "no FTDI 0403:6001 serial device found")

    if Path("/sys/module/xsens_mt").exists():
        R.ok("xsens_mt kernel module loaded")
    else:
        R.fail("xsens_mt kernel module", "not loaded -- bash "
               "drivers/xsens_mt/install.sh (needed after kernel updates)")

    try:
        from hardware.pinger.daisy_stream import BOARDS
    except Exception:  # noqa: BLE001
        BOARDS = {"376636603433": (0, 1), "376C36573433": (2, 3)}
    for serial_no, chans in BOARDS.items():
        hits = glob.glob(f"/dev/serial/by-id/*{serial_no}*")
        if hits:
            R.ok(f"hydrophone board ch{chans[0]}/{chans[1]}",
                 os.path.realpath(hits[0]))
        else:
            R.fail(f"hydrophone board ch{chans[0]}/{chans[1]}",
                   f"serial {serial_no} not under /dev/serial/by-id")


def find_dvl_port():
    if os.path.exists(DVL_LINK):
        return DVL_LINK
    try:
        from serial.tools import list_ports
    except ImportError:
        return None
    imu_real = os.path.realpath(IMU_LINK) if os.path.exists(IMU_LINK) else None
    for p in list_ports.comports():
        if (p.vid, p.pid) == DVL_VID_PID and os.path.realpath(p.device) != imu_real:
            return p.device
    return None


# =============================================================================
# TEENSY (telemetry, lights, torpedoes, thrusters)
# =============================================================================
class Teensy:
    """Minimal re-implementation of the arduino node's serial protocol.

    Every PWM line ("1500 1500 ... x8\\n") is answered with one telemetry
    line "...> pressure: X depth: Y ...". Other commands ("light N",
    "t_left", "t_right", "t_zero") are answered with one ack line.
    """

    def __init__(self, port=TEENSY_PORT, baud=TEENSY_BAUD):
        import serial
        self.serial_mod = serial
        self.ser = serial.Serial(port, baudrate=baud, timeout=1, exclusive=True)
        time.sleep(0.3)
        self.ser.reset_input_buffer()
        self.pwms = [NEUTRAL] * THRUSTER_COUNT

    @staticmethod
    def parse(line):
        if "> " not in line:
            return None
        body = line.split("> ", 1)[1].strip()
        tokens = body.split()
        data = {}
        for i in range(0, len(tokens) - 1, 2):
            try:
                data[tokens[i].rstrip(":")] = float(tokens[i + 1])
            except ValueError:
                pass
        # firmware typo: 'internal_temperatur2'
        if "internal_temperatur2" in data and "internal_temperature2" not in data:
            data["internal_temperature2"] = data["internal_temperatur2"]
        return data or None

    def send_pwms(self, pwms=None):
        if pwms is not None:
            self.pwms = list(pwms)
        line = " ".join(str(int(min(max(p, PWM_MIN), PWM_MAX)))
                        for p in self.pwms)
        self.ser.write((line + "\n").encode())
        raw = self.ser.readline().decode(errors="replace").strip()
        return self.parse(raw), raw

    def command(self, cmd):
        self.ser.write((cmd + "\n").encode())
        return self.ser.readline().decode(errors="replace").strip()

    def stream(self, seconds, pwms=None):
        """Hold the given PWMs for `seconds`, returning telemetry samples."""
        samples, raws = [], []
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            d, raw = self.send_pwms(pwms)
            if d:
                d["_t"] = time.monotonic()
                samples.append(d)
            elif raw:
                raws.append(raw)
        return samples, raws

    def neutral_burst(self):
        line = (" ".join([str(NEUTRAL)] * THRUSTER_COUNT) + "\n").encode()
        for _ in range(10):
            try:
                self.ser.write(line)
                self.ser.flush()
            except Exception:  # noqa: BLE001
                pass
            time.sleep(0.05)
        self.pwms = [NEUTRAL] * THRUSTER_COUNT

    def close(self):
        try:
            self.neutral_burst()
        finally:
            self.ser.close()


TEENSY = None  # shared across teensy/lights/torpedoes/thrusters sections


def open_teensy():
    global TEENSY
    if TEENSY is not None:
        return TEENSY
    if not os.path.exists(TEENSY_PORT):
        R.fail("open Teensy serial", f"{TEENSY_PORT} missing")
        return None
    try:
        TEENSY = Teensy()
        R.ok("open Teensy serial", f"{TEENSY_PORT} @ {TEENSY_BAUD}")
    except Exception as e:  # noqa: BLE001
        holders = port_holders(TEENSY_PORT)
        R.fail("open Teensy serial", f"{e}" + (f" (held by PID {holders})"
                                               if holders else ""))
    return TEENSY


def judge_telemetry(samples, label=""):
    """Shared threshold logic for Teensy telemetry (hardware + live mode)."""
    def col(k):
        return [s[k] for s in samples if k in s]

    keys = ("voltage", "current", "humidity", "internal_temperature1",
            "internal_temperature2", "external_temperature", "depth",
            "pressure")
    missing = [k for k in keys if not col(k)]
    if missing:
        R.warn(f"{label}telemetry fields", f"missing: {', '.join(missing)}")

    v, _ = stats(col("voltage"))
    if v is not None:
        cells = v / 4
        if v < TH["battery_fail_low"]:
            R.fail(f"{label}battery voltage", f"{v:.2f} V ({cells:.2f} V/cell) "
                   "-- CHARGE/SWAP")
        elif v > TH["battery_fail_high"]:
            R.fail(f"{label}battery voltage", f"{v:.2f} V -- implausible / "
                   "wrong pack")
        elif v < TH["battery_warn_low"]:
            R.warn(f"{label}battery voltage", f"{v:.2f} V ({cells:.2f} V/cell) "
                   "-- low, swap soon")
        else:
            R.ok(f"{label}battery voltage", f"{v:.2f} V ({cells:.2f} V/cell)")

    i, _ = stats(col("current"))
    if i is not None:
        (R.ok if i < TH["idle_current_warn"] else R.warn)(
            f"{label}current draw", f"{i:.2f} A")

    hum = col("humidity")
    if hum:
        h = sum(hum) / len(hum)
        rise = hum[-1] - hum[0]
        if h >= TH["humidity_fail"]:
            R.fail(f"{label}hull humidity (leak)", f"{h:.1f}% -- POSSIBLE LEAK, "
                   "open and inspect")
        elif h >= TH["humidity_warn"]:
            R.warn(f"{label}hull humidity (leak)", f"{h:.1f}% -- check seals / "
                   "desiccant")
        elif rise >= TH["humidity_rise_warn"]:
            R.warn(f"{label}hull humidity (leak)", f"{h:.1f}% but rose "
                   f"{rise:.1f}% during the check")
        else:
            R.ok(f"{label}hull humidity (leak)", f"{h:.1f}%")

    for k in ("internal_temperature1", "internal_temperature2"):
        t, _ = stats(col(k))
        if t is None:
            continue
        name = f"{label}{k.replace('_', ' ')}"
        if t >= TH["int_temp_fail"]:
            R.fail(name, f"{t:.1f} C -- overheating")
        elif t >= TH["int_temp_warn"]:
            R.warn(name, f"{t:.1f} C")
        else:
            R.ok(name, f"{t:.1f} C")

    wt, _ = stats(col("external_temperature"))
    if wt is not None:
        (R.ok if TH["water_temp_min"] < wt < TH["water_temp_max"] else R.fail)(
            f"{label}water temp (Bar02)", f"{wt:.1f} C")

    d, dsd = stats(col("depth"))
    if d is not None:
        if dsd is not None and dsd > TH["depth_noise_warn"]:
            R.warn(f"{label}depth sensor noise", f"std {dsd:.3f} m")
        else:
            R.ok(f"{label}depth sensor noise", f"std {dsd:.3f} m")
    p, _ = stats(col("pressure"))
    if p is not None:
        if not (p > 0 and math.isfinite(p)):
            R.fail(f"{label}pressure", f"{p}")
        else:
            R.info(f"{label}pressure / depth", f"{p:.1f} / "
                   + (f"{d:.3f} m" if d is not None else "n/a"))
    return d


def check_teensy(args):
    t = open_teensy()
    if t is None:
        return
    samples, junk = t.stream(3.0)
    if not samples:
        R.fail("telemetry", "no parseable reply to PWM lines" +
               (f" (got e.g. {junk[0]!r})" if junk else " (silent)"))
        return
    hz = len(samples) / (samples[-1]["_t"] - samples[0]["_t"] + 1e-9)
    (R.ok if hz >= TH["telemetry_min_hz"] else R.warn)(
        "telemetry rate", f"{hz:.1f} Hz ({len(samples)} samples)")
    if junk:
        R.warn("telemetry garbage lines", f"{len(junk)} unparsed, e.g. "
               f"{junk[0][:60]!r}")

    d = judge_telemetry(samples)
    if d is not None:
        if args.in_water:
            R.info("depth (in water)", f"{d:.3f} m")
        elif abs(d) > TH["surface_depth_warn"]:
            R.warn("depth at surface", f"{d:.3f} m (expect ~0 out of water; "
                   "re-zero / check Bar02)")
        else:
            R.ok("depth at surface", f"{d:.3f} m")

    # BNO085s forwarded by the Teensy (optional firmware feature)
    for n in (0, 1):
        keys = [f"imu{n}_{k}" for k in ("qw", "qx", "qy", "qz", "ax", "ay", "az")]
        rows = [s for s in samples if all(k in s for k in keys)]
        if not rows:
            R.skip(f"BNO085 #{n}", "not in telemetry (firmware does not send "
                   "imu fields yet)")
            continue
        s = rows[-1]
        qn = math.sqrt(sum(s[f"imu{n}_q{a}"] ** 2 for a in "wxyz"))
        an = math.sqrt(sum(s[f"imu{n}_a{a}"] ** 2 for a in "xyz"))
        ok = abs(qn - 1) < 0.05 and 8.8 < an < 10.8
        (R.ok if ok else R.fail)(f"BNO085 #{n}", f"|q|={qn:.3f} |a|={an:.2f} "
                                 "m/s^2")


def check_lights(args):
    t = open_teensy()
    if t is None:
        return
    ack = t.command("light 1500")
    t.stream(1.0)
    seen = ask(args, "Are the lights ON (dim/half)?")
    t.command("light 1100")
    t.stream(0.3)
    detail = f"ack {ack!r}" if ack else "no ack line"
    if seen is True:
        R.ok("lights", detail)
    elif seen is False:
        R.fail("lights", "operator saw no light; " + detail)
    else:
        (R.info if ack else R.warn)("lights", detail + " (not visually "
                                    "confirmed)")


def check_torpedoes(args):
    t = open_teensy()
    if t is None:
        return
    ack = t.command("t_zero")
    t.stream(0.5)
    if "zero" in ack.lower():
        R.ok("torpedo servo re-center", repr(ack))
    else:
        R.warn("torpedo servo re-center", f"unexpected ack {ack!r} "
               "(expect 'Torpedo zeroed')")
    if not args.fire_torpedoes:
        R.skip("torpedo fire", "pass --fire-torpedoes to fire both")
        return
    if ask(args, "FIRE both torpedoes now? Area clear?", default=False) is False:
        R.skip("torpedo fire", "operator declined")
        return
    for cmd, word in (("t_left", "left"), ("t_right", "right")):
        ack = t.command(cmd)
        (R.ok if word in ack.lower() else R.warn)(f"torpedo {word}", repr(ack))
        t.stream(2.0)
    t.command("t_zero")
    t.stream(0.5)


def check_thrusters(args):
    if args.no_thrusters:
        R.skip("thrusters", "--no-thrusters")
        return
    t = open_teensy()
    if t is None:
        return
    if not args.yes:
        print("      !! Each thruster will spin briefly forward then reverse.")
        print("      !! Keep hands, cables and tools clear of every prop.")
        if ask(args, "Ready to spin thrusters?", default=False) is not True:
            R.skip("thrusters", "operator declined")
            return
    dpwm = TH["spin_pwm_delta"]
    secs = TH["spin_seconds"]

    base, _ = t.stream(1.0)
    i0, _ = stats([s.get("current") for s in base])
    have_current = i0 is not None
    if have_current:
        R.info("idle current baseline", f"{i0:.2f} A")
    else:
        R.warn("current sensing", "no 'current' in telemetry -- thrusters "
               "can only be verified by eye")

    def pulse(idx, pwm):
        pw = [NEUTRAL] * THRUSTER_COUNT
        pw[idx] = pwm
        smp, _ = t.stream(secs, pw)
        t.stream(0.4, [NEUTRAL] * THRUSTER_COUNT)   # spin down
        tail = smp[len(smp) // 3:]                  # skip spin-up
        m, _ = stats([s.get("current") for s in tail])
        return None if m is None or i0 is None else m - i0

    try:
        for idx in range(THRUSTER_COUNT):
            name = f"thruster {idx} ({THRUSTER_NAMES[idx]})"
            print(f"      -> spinning {name}: fwd {NEUTRAL + dpwm}, "
                  f"rev {NEUTRAL - dpwm}")
            d_fwd = pulse(idx, NEUTRAL + dpwm)
            d_rev = pulse(idx, NEUTRAL - dpwm)
            seen = ask(args, f"Did {name} spin both directions?")
            cur = ""
            cur_ok = None
            if have_current and d_fwd is not None and d_rev is not None:
                cur = f"dI fwd {d_fwd:+.2f} A, rev {d_rev:+.2f} A"
                thr = TH["thruster_current_delta"]
                cur_ok = d_fwd >= thr and d_rev >= thr
            if seen is False:
                R.fail(name, "operator: did NOT spin both ways. " + cur)
            elif seen is True:
                if cur_ok is False:
                    R.warn(name, "spun (operator) but low current rise -- "
                           "check current sensor / ESC. " + cur)
                else:
                    R.ok(name, cur or "operator confirmed")
            else:  # --yes: current is the only evidence
                if cur_ok is True:
                    R.ok(name, cur)
                elif cur_ok is False:
                    R.fail(name, "no current rise -- ESC/motor/cable? " + cur)
                else:
                    R.warn(name, "unverified (no current data, no operator)")
    finally:
        t.neutral_burst()
    R.info("thrusters returned to neutral", f"{NEUTRAL} x {THRUSTER_COUNT}")


# =============================================================================
# ROS helpers (IMU check in hardware mode; everything in live mode)
# =============================================================================
class TopicProbe:
    """Subscribe to a set of topics, record arrival times + last messages."""

    def __init__(self, specs, node_name="preflight_probe"):
        import rclpy
        from rclpy.qos import qos_profile_sensor_data, QoSProfile
        self.rclpy = rclpy
        if not rclpy.ok():
            rclpy.init()
        self.node = rclpy.create_node(node_name)
        self.data = {}
        for topic, msg_type in specs.items():
            self.data[topic] = {"t": [], "msgs": []}
            for qos in (QoSProfile(depth=50), qos_profile_sensor_data):
                # subscribe with both reliable and best-effort so we match
                # either kind of publisher
                self.node.create_subscription(
                    msg_type, topic, self._cb(topic), qos)

    def _cb(self, topic):
        def f(msg):
            d = self.data[topic]
            now = time.monotonic()
            # dedupe the double subscription (same message twice within 0.2ms)
            if d["t"] and now - d["t"][-1] < 2e-4:
                return
            d["t"].append(now)
            d["msgs"].append(msg)
            if len(d["msgs"]) > 400:
                d["msgs"].pop(0)
        return f

    def spin(self, seconds):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            self.rclpy.spin_once(self.node, timeout_sec=0.05)

    def wait_first(self, topic, timeout):
        end = time.monotonic() + timeout
        while time.monotonic() < end and not self.data[topic]["t"]:
            self.rclpy.spin_once(self.node, timeout_sec=0.1)
        return bool(self.data[topic]["t"])

    def reset(self):
        for d in self.data.values():
            d["t"].clear()
            d["msgs"].clear()

    def rate(self, topic):
        ts = self.data[topic]["t"]
        if len(ts) < 2:
            return 0.0
        return (len(ts) - 1) / (ts[-1] - ts[0])

    def last(self, topic):
        m = self.data[topic]["msgs"]
        return m[-1] if m else None

    def close(self):
        self.node.destroy_node()


def quat_to_rpy(q):
    x, y, z, w = q.x, q.y, q.z, q.w
    roll = math.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))
    pitch = math.asin(max(-1.0, min(1.0, 2 * (w * y - z * x))))
    yaw = math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
    return tuple(math.degrees(a) for a in (roll, pitch, yaw))


def ros_available():
    try:
        import rclpy  # noqa: F401
        return True
    except ImportError:
        return False


def judge_imu(probe, quat_topic="/filter/quaternion"):
    hz = probe.rate(quat_topic)
    (R.ok if hz >= TH["imu_min_hz"] else R.warn)(
        f"{quat_topic} rate", f"{hz:.0f} Hz (expect ~400)")
    msgs = probe.data[quat_topic]["msgs"]
    if msgs:
        bad = 0
        for m in msgs:
            q = m.quaternion
            n = math.sqrt(q.x ** 2 + q.y ** 2 + q.z ** 2 + q.w ** 2)
            if not math.isfinite(n) or abs(n - 1) > 0.02:
                bad += 1
        (R.ok if bad == 0 else R.fail)("IMU quaternion valid",
                                       f"{bad}/{len(msgs)} invalid")
        r, p, y = quat_to_rpy(msgs[-1].quaternion)
        lvl = max(abs(r), abs(p))
        (R.ok if lvl < TH["imu_level_warn_deg"] else R.warn)(
            "IMU roll/pitch", f"roll {r:+.1f}  pitch {p:+.1f}  yaw {y:+.1f} deg"
            + ("" if lvl < TH["imu_level_warn_deg"] else
               " -- sub not level, or IMU alignment lost"))
    raw = probe.data.get("/imu/data", {}).get("msgs", [])
    if raw:
        gx = [m.angular_velocity for m in raw]
        g = max(math.sqrt(a.x ** 2 + a.y ** 2 + a.z ** 2) for a in gx)
        (R.ok if g < TH["imu_gyro_bias_warn"] else R.warn)(
            "IMU gyro at rest", f"max |w| {g:.3f} rad/s"
            + ("" if g < TH["imu_gyro_bias_warn"] else
               " -- is the sub moving? gyro bias?"))


# =============================================================================
# IMU (hardware mode): bring up the real driver briefly in an isolated domain
# =============================================================================
def check_imu(args):
    if not ros_available():
        R.skip("IMU functional test", "rclpy not importable -- run through "
               "./preflight.sh (sources the workspace)")
        return
    if not os.path.exists(IMU_LINK):
        R.fail("IMU functional test", f"{IMU_LINK} missing")
        return
    from geometry_msgs.msg import QuaternionStamped
    from sensor_msgs.msg import Imu

    log = REPO / "logs" / f"preflight_imu_{_dt.datetime.now():%H%M%S}.log"
    log.parent.mkdir(exist_ok=True)
    print(f"      starting Xsens driver + hardware/imu (log: {log.name}) ...")
    with open(log, "w") as lf:
        proc = subprocess.Popen(["ros2", "launch", "main", "imu.py"],
                                stdout=lf, stderr=subprocess.STDOUT,
                                stdin=subprocess.DEVNULL,
                                start_new_session=True, env=os.environ.copy())
    probe = None
    try:
        probe = TopicProbe({"/filter/quaternion": QuaternionStamped,
                            "/imu/data": Imu,
                            "/imu/orientation": Imu}, "preflight_imu")
        if not probe.wait_first("/filter/quaternion", 25):
            tail = log.read_text()[-400:] if log.exists() else ""
            hint = ("'No MTi device found' -- replug IMU / reload xsens_mt"
                    if "No MTi" in tail else tail.strip().splitlines()[-1:]
                    or "no output")
            R.fail("Xsens driver publishing", f"no /filter/quaternion in 25 s "
                   f"({hint})")
            return
        R.ok("Xsens driver publishing")
        probe.spin(1.0)
        probe.reset()
        probe.spin(args.imu_seconds)
        judge_imu(probe)
        hz = probe.rate("/imu/orientation")
        (R.ok if hz > 20 else R.fail)("hardware/imu -> /imu/orientation",
                                      f"{hz:.0f} Hz (expect ~100)")
    finally:
        if probe:
            probe.close()
        try:
            os.killpg(proc.pid, signal.SIGINT)
            proc.wait(timeout=8)
        except (subprocess.TimeoutExpired, ProcessLookupError):
            try:
                os.killpg(proc.pid, signal.SIGKILL)
            except ProcessLookupError:
                pass


# =============================================================================
# DVL
# =============================================================================
def check_dvl(args):
    port = find_dvl_port()
    if not port:
        R.fail("DVL port", "no /dev/ttyUSB_dvl and no FTDI 0403:6001 device")
        return
    try:
        from hardware.utils.dvl_utils.dvl_connect import Dvl
    except Exception as e:  # noqa: BLE001
        R.fail("DVL SDK import", e)
        return
    dvl = None
    try:
        dvl = Dvl(port, 115200)
        if not dvl.is_connected():
            R.fail("DVL connect", f"{port}: no response (powered? streaming? "
                   "a powered-but-idle DVL can read 0 bytes)")
            return
        R.ok("DVL connect", port)

        if dvl.enter_command_mode() and dvl.get_setup():
            sos = dvl.system_setup.speed_of_sound
            (R.ok if 1400 < sos < 1600 else R.warn)(
                "DVL speed of sound", f"{sos:.0f} m/s" +
                (" (seawater default -- dvl node fixes it on launch)"
                 if abs(sos - 1500) < 0.5 else ""))
        else:
            R.warn("DVL setup readback", "could not read system setup")

        got = []
        dvl.register_ondata_callback(lambda data, obj: got.append(data))
        if not dvl.exit_command_mode():
            R.fail("DVL start pinging", "exit_command_mode failed")
            return
        print(f"      pinging for {args.dvl_seconds:.0f} s ...")
        t0 = time.monotonic()
        time.sleep(args.dvl_seconds)
        dt = time.monotonic() - t0
        dvl.enter_command_mode()   # stop pinging

        valid = [d for d in got if d is not None and getattr(d, "is_valid", False)]
        hz = len(valid) / dt
        if not valid:
            R.fail("DVL data", "pinging but no valid ensembles received")
            return
        (R.ok if hz >= TH["dvl_min_hz"] else R.warn)("DVL ensemble rate",
                                                     f"{hz:.1f} Hz")
        last = valid[-1]
        bit = getattr(last, "bit_count", 0)
        (R.ok if bit == 0 else R.fail)("DVL built-in test",
                                       "no errors" if bit == 0 else
                                       f"{bit} BIT errors (code {last.bit_code})")
        volt = getattr(last, "voltage", None)
        if volt:
            (R.ok if volt > 10 else R.warn)("DVL input voltage", f"{volt:.1f} V")
        locked = [d for d in valid if d.is_velocity_valid()]
        frac = len(locked) / len(valid)
        if args.in_water:
            (R.ok if frac > 0.8 else R.fail)(
                "DVL bottom lock", f"{frac:.0%} of ensembles; mean range "
                f"{last.mean_range:.2f} m")
        else:
            R.info("DVL bottom lock", f"{frac:.0%} (none expected out of "
                   "water; rerun with --in-water once floating)")
    finally:
        if dvl is not None:
            try:
                dvl.disconnect()
            except Exception:  # noqa: BLE001
                pass


# =============================================================================
# HYDROPHONES (DaisySeed boards)
# =============================================================================
def _alarm(*_):
    raise TimeoutError("no audio")


def check_hydrophones(args):
    try:
        from hardware.pinger.daisy_stream import BOARDS, Stream
    except Exception as e:  # noqa: BLE001
        R.fail("Daisy stream import", e)
        return
    import numpy as np
    for serial_no, chans in BOARDS.items():
        label = f"hydrophones ch{chans[0]}/{chans[1]}"
        hits = glob.glob(f"/dev/serial/by-id/*{serial_no}*")
        if not hits:
            R.fail(label, f"board {serial_no} not connected")
            continue
        st = None
        try:
            st = Stream(hits[0])
            info = {}
            pcm_chunks = []
            end = time.monotonic() + args.hydro_seconds
            gen = st.batches(info)
            # batches() blocks on the pipe; guard with an alarm
            signal.signal(signal.SIGALRM, _alarm)
            signal.alarm(int(args.hydro_seconds) + 3)
            try:
                for pcm, rate, bits in gen:
                    pcm_chunks.append(pcm)
                    if time.monotonic() > end:
                        break
            finally:
                signal.alarm(0)
            pcm = np.concatenate(pcm_chunks) if pcm_chunks else np.zeros((0, 2))
            if not len(pcm):
                R.fail(label, "no audio frames")
                continue
            rate = info.get("rate", 0)
            drops = info.get("dropped", 0)
            R.ok(f"{label} stream", f"{rate} Hz / {info.get('bits')} bit, "
                 f"{len(pcm)} samples")
            if drops:
                R.warn(f"{label} dropped frames", f"{drops} sequence gaps")
            for k, ch in enumerate(chans):
                x = pcm[:, k]
                rms = float(np.sqrt(np.mean((x - x.mean()) ** 2)))
                clip = float(np.mean(np.abs(x) >= TH["hydro_clip"]))
                name = f"hydrophone {ch}"
                if rms < TH["hydro_dead_rms"]:
                    R.fail(name, f"dead / flat signal (rms {rms:.1e})")
                elif clip > 0.01:
                    R.warn(name, f"{clip:.1%} samples clipped (rms {rms:.4f})")
                else:
                    R.ok(name, f"rms {rms:.4f}, peak {np.abs(x).max():.3f}")
        except TimeoutError:
            R.fail(label, "opened but no audio within timeout (firmware?)")
        except Exception as e:  # noqa: BLE001
            R.fail(label, f"{type(e).__name__}: {e}")
        finally:
            if st:
                st.close()


# =============================================================================
# CAMERAS
# =============================================================================
def check_cameras(args):
    try:
        cy = load_yaml(SRC / "perception" / "perception" / "cameras.yaml")
    except Exception as e:  # noqa: BLE001
        R.fail("cameras.yaml", e)
        return
    oaks = {k: v["ros__parameters"] for k, v in cy.items() if k.startswith("oak")}
    rss = {k: v["ros__parameters"] for k, v in cy.items()
           if k.startswith("realsense")}

    if oaks:
        try:
            import depthai as dai
        except ImportError:
            R.fail("depthai import", "python -m pip install depthai")
            dai = None
        if dai:
            avail = {}
            for d in dai.Device.getAllAvailableDevices():
                did = d.getDeviceId() if hasattr(d, "getDeviceId") else d.getMxId()
                avail[str(did)] = d
            for key, cfg in oaks.items():
                mxid = str(cfg["camera"]["i_mx_id"])
                if mxid not in avail:
                    R.fail(f"{key} (OAK {mxid})", "not found; available: "
                           f"{', '.join(avail) or 'none'}")
                    continue
                run_safely(_grab_oak, dai, key, avail[mxid], cfg)

    for key, cfg in rss.items():
        sn = str(cfg["camera"]["i_serial_number"])
        try:
            import pyrealsense2 as rs
        except ImportError:
            R.warn(f"{key} (RealSense {sn})", "pyrealsense2 not installed -- "
                   "skipped (not in the flown stack)")
            continue
        sns = [d.get_info(rs.camera_info.serial_number)
               for d in rs.context().query_devices()]
        if sn in sns:
            R.ok(f"{key} (RealSense {sn})", "enumerated")
        else:
            R.warn(f"{key} (RealSense {sn})", f"not connected (found: "
                   f"{', '.join(sns) or 'none'})")


def _grab_oak(dai, key, info, cfg):
    import numpy as np
    w = cfg.get("rgb", {}).get("rgb_width", 640)
    h = cfg.get("rgb", {}).get("rgb_height", 480)
    device = dai.Device(info)
    try:
        speed = str(device.getUsbSpeed()).split(".")[-1]
        R.ok(f"{key} open", f"USB {speed}")
        pipeline = dai.Pipeline(device)
        cam = pipeline.create(dai.node.ColorCamera)
        cam.setPreviewSize(w, h)
        cam.setFps(10)
        cam.setInterleaved(False)
        q_rgb = cam.preview.createOutputQueue(maxSize=4, blocking=False)
        left = pipeline.create(dai.node.MonoCamera)
        right = pipeline.create(dai.node.MonoCamera)
        left.setBoardSocket(dai.CameraBoardSocket.LEFT)
        right.setBoardSocket(dai.CameraBoardSocket.RIGHT)
        for m in (left, right):
            m.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P)
            m.setFps(10)
        stereo = pipeline.create(dai.node.StereoDepth)
        left.out.link(stereo.left)
        right.out.link(stereo.right)
        q_depth = stereo.depth.createOutputQueue(maxSize=4, blocking=False)
        pipeline.start()
        rgb = depth = None
        end = time.monotonic() + 10
        n_rgb = 0
        while time.monotonic() < end and (rgb is None or depth is None
                                          or n_rgb < 5):
            f = q_rgb.tryGet()
            if f is not None:
                rgb = f.getCvFrame()
                n_rgb += 1
            g = q_depth.tryGet()
            if g is not None:
                depth = g.getFrame()
            time.sleep(0.02)
        pipeline.stop()
        if rgb is None:
            R.fail(f"{key} RGB frames", "no frames in 10 s (USB bandwidth?)")
        else:
            mean = float(rgb.mean())
            (R.ok if 5 < mean < 250 else R.warn)(
                f"{key} RGB frames", f"{rgb.shape[1]}x{rgb.shape[0]}, mean "
                f"brightness {mean:.0f}" + ("" if 5 < mean < 250 else
                                            " -- lens cap / dark / blown out?"))
        if depth is None:
            R.fail(f"{key} depth frames", "no depth frames in 10 s")
        else:
            valid = float(np.mean(depth > 0))
            (R.ok if valid > 0.2 else R.warn)(
                f"{key} depth frames", f"{valid:.0%} valid pixels")
    finally:
        device.close()


# =============================================================================
# LIVE mode -- the stack is running; inspect the ROS graph
# =============================================================================
def live_mode(args):
    if not ros_available():
        R.begin("live", "Live stack")
        R.fail("rclpy", "not importable -- run via ./preflight.sh")
        return
    from geometry_msgs.msg import (PoseWithCovarianceStamped, QuaternionStamped,
                                   TwistWithCovarianceStamped, WrenchStamped)
    from nav_msgs.msg import Odometry
    from sensor_msgs.msg import Image, Imu
    from msgs.msg import PingerStamped, PWMsStamped, SensorsStamped, ThrustsStamped

    # topic: (type, min_hz, required)  required=False -> WARN instead of FAIL
    topics = {
        "/filter/quaternion": (QuaternionStamped, TH["imu_min_hz"], True),
        "/imu/data": (Imu, 50, False),
        "/imu/orientation": (Imu, 20, True),
        "/arduino/sensors": (SensorsStamped, TH["telemetry_min_hz"], True),
        "/position": (PoseWithCovarianceStamped, 2, True),
        "/velocity": (TwistWithCovarianceStamped, TH["dvl_min_hz"],
                      args.in_water),
        "/odometry/filtered": (Odometry, 30, True),
        "/wrench": (WrenchStamped, 10, False),
        "/thrusts": (ThrustsStamped, 10, False),
        "pwms": (PWMsStamped, 50, False),
        "/pinger/levels": (PingerStamped, 10, True),
        "/desired/pose": (Odometry, 10, False),
        "/camera/oak_0/rgb": (Image, 5, False),
        "/heading_correction": (PoseWithCovarianceStamped, 0.5, False),
    }
    R.begin("live", "Live stack: nodes")
    probe = TopicProbe({t: v[0] for t, v in topics.items()}, "preflight_live")
    probe.spin(2.0)   # discovery
    node_list = probe.node.get_node_names()
    nodes = set(node_list)
    expected = {
        "xsens_mti_node": True, "imu": True, "dvl_ros_bridge": True,
        "arduino": True, "thrusters": True, "thrust_generator": True,
        "depth": True, "daisy": True, "ekf_filter_node": True,
        "pid_control": False, "path_generator": False,
        "oak_node": False, "heading_corrector": False,
    }
    for n, req in expected.items():
        if n in nodes:
            R.ok(f"node {n}")
        else:
            (R.fail if req else R.warn)(f"node {n}", "not running")
    for n in sorted(nodes):
        if n != "preflight_live" and node_list.count(n) > 1:
            R.fail(f"node {n}", "running more than once")

    # Substitution-point rule (README): exactly one publisher each.
    for t in ("/odometry/filtered", "/wrench"):
        k = probe.node.count_publishers(t)
        if k == 1:
            R.ok(f"single publisher {t}")
        elif k == 0:
            (R.fail if t == "/odometry/filtered" else R.warn)(
                f"single publisher {t}", "none")
        else:
            R.fail(f"single publisher {t}", f"{k} publishers -- a fake_localizer"
                   "/keyboard/sim node is doubled up with the real one")

    R.begin("live", "Live stack: topic rates")
    probe.reset()
    print(f"      sampling for {args.live_seconds:.0f} s ...")
    probe.spin(args.live_seconds)
    for t, (_, min_hz, req) in topics.items():
        hz = probe.rate(t)
        if hz >= min_hz:
            R.ok(t, f"{hz:.1f} Hz")
        elif hz == 0:
            (R.fail if req else R.warn)(t, "silent")
        else:
            (R.fail if req else R.warn)(t, f"{hz:.1f} Hz < {min_hz}")

    R.begin("live", "Live stack: values")
    tel = []
    for m in probe.data["/arduino/sensors"]["msgs"]:
        tel.append({k: getattr(m, k) for k in (
            "voltage", "current", "humidity", "internal_temperature1",
            "internal_temperature2", "external_temperature", "depth",
            "pressure")})
    if tel:
        judge_telemetry(tel)
    if probe.data["/filter/quaternion"]["msgs"]:
        judge_imu(probe)

    odo = probe.data["/odometry/filtered"]["msgs"]
    if odo:
        o = odo[-1]
        p = o.pose.pose.position
        vals = [p.x, p.y, p.z, o.twist.twist.linear.x, o.twist.twist.linear.y,
                o.twist.twist.linear.z]
        if not all(math.isfinite(v) for v in vals):
            R.fail("EKF output finite", "NaN/inf in /odometry/filtered -- "
                   "restart the stack")
        else:
            R.ok("EKF output finite", f"pos ({p.x:+.2f}, {p.y:+.2f}, "
                 f"{p.z:+.2f}) m")
        cov = list(o.pose.covariance)
        if cov and max(cov[0], cov[7], cov[14]) > 100:
            R.warn("EKF position covariance", f"diag {cov[0]:.1f}/{cov[7]:.1f}"
                   f"/{cov[14]:.1f} -- filter not converged")
        imu = probe.last("/imu/orientation")
        if imu:
            _, _, ye = quat_to_rpy(o.pose.pose.orientation)
            _, _, yi = quat_to_rpy(imu.orientation)
            dy = (ye - yi + 180) % 360 - 180
            (R.ok if abs(dy) < 10 else R.warn)("EKF yaw vs IMU",
                                               f"{dy:+.1f} deg")
        speed = math.sqrt(sum(v * v for v in vals[3:]))
        (R.ok if speed < 0.2 else R.warn)("EKF velocity at rest",
                                          f"{speed:.2f} m/s" + (
                                              "" if speed < 0.2 else
                                              " -- drifting while stationary"))

    pw = probe.last("pwms")
    if pw is not None:
        R.info("current PWMs", " ".join(str(x) for x in pw.pwms))
    if os.environ.get("NO_THRUST", "0") not in ("", "0"):
        R.info("NO_THRUST", "set in this shell (stack may be in bench mode)")

    lv = probe.data["/pinger/levels"]["msgs"]
    if lv:
        R.info("pinger levels", str(lv[-1])[:120])
    probe.close()


# =============================================================================
# main
# =============================================================================
def main():
    ap = argparse.ArgumentParser(
        description="AUV poolside pre-flight check -- see module docstring.")
    ap.add_argument("--live", action="store_true",
                    help="stack is running: check the ROS graph instead of "
                    "talking to devices directly")
    ap.add_argument("--only", type=lambda s: s.split(","), default=[],
                    help=f"comma list of sections: {','.join(ALL_SECTIONS)}")
    ap.add_argument("--skip", type=lambda s: s.split(","), default=[],
                    help="comma list of sections to skip")
    ap.add_argument("--yes", "-y", action="store_true",
                    help="no operator prompts (thrusters verified by current)")
    ap.add_argument("--no-thrusters", action="store_true",
                    help="never spin a thruster")
    ap.add_argument("--fire-torpedoes", action="store_true",
                    help="actually fire both torpedoes")
    ap.add_argument("--in-water", action="store_true",
                    help="sub is floating: expect DVL bottom lock, skip "
                    "surface-depth check")
    ap.add_argument("--gpu", action="store_true", help="also test GPU torch")
    ap.add_argument("--force", action="store_true",
                    help="run hardware mode even if stack processes are found")
    ap.add_argument("--imu-seconds", type=float, default=5.0)
    ap.add_argument("--dvl-seconds", type=float, default=5.0)
    ap.add_argument("--hydro-seconds", type=float, default=2.0)
    ap.add_argument("--live-seconds", type=float, default=8.0)
    args = ap.parse_args()
    bad = [s for s in args.only + args.skip if s not in ALL_SECTIONS]
    if bad and not args.live:
        ap.error(f"unknown section(s) {bad}; choose from {ALL_SECTIONS}")

    started = _dt.datetime.now()
    mode = "LIVE" if args.live else "HARDWARE"
    if not args.live:
        # The IMU check starts the real Xsens driver; keep it on a private DDS
        # domain so it can't collide with anyone else's ROS graph.
        os.environ["ROS_DOMAIN_ID"] = os.environ.get("PREFLIGHT_DOMAIN_ID", "77")
        os.environ["ROS_LOCALHOST_ONLY"] = "1"

    # Ctrl+C anywhere: thrusters to neutral, then still print the report.
    interrupted = False
    try:
        if args.live:
            live_mode(args)
        else:
            procs = stack_processes()
            if procs and not args.force:
                R.begin("system", "Stack check")
                R.fail("stack is running", f"{len(procs)} stack process(es), "
                       "e.g. " + procs[0][:80] + " -- stop it (./kill_sub.sh) "
                       "or use --live")
            else:
                steps = [
                    ("system", "System / workspace", check_system),
                    ("config", "Configuration files", check_config),
                    ("usb", "USB devices & drivers", check_usb),
                    ("teensy", "Teensy: telemetry, battery, leak, temps",
                     check_teensy),
                    ("lights", "Lights", check_lights),
                    ("torpedoes", "Torpedoes", check_torpedoes),
                    ("thrusters", "Thrusters (8x T200)", check_thrusters),
                    ("imu", "Xsens MTi-200 IMU", check_imu),
                    ("dvl", "Teledyne Wayfinder DVL", check_dvl),
                    ("hydrophones", "Hydrophones (DaisySeed)",
                     check_hydrophones),
                    ("cameras", "Cameras", check_cameras),
                ]
                for name, title, fn in steps:
                    if section(name, title, args):
                        run_safely(fn, args)
    except KeyboardInterrupt:
        interrupted = True
        print("\n  !! interrupted")
        R.fail("check interrupted", "Ctrl+C -- report is incomplete")
    finally:
        if TEENSY is not None:
            try:
                TEENSY.close()   # always leaves thrusters at neutral
            except Exception:  # noqa: BLE001
                pass

    lines, verdict, code = R.summary(mode, started)
    print()
    for ln in lines:
        colored = ln
        if ln.startswith(" VERDICT"):
            st = FAIL if code == 1 else WARN if code == 2 else PASS
            colored = c(st, ln)
        else:
            for st in (FAIL, WARN, PASS, SKIP):
                if ln.rstrip().endswith(" " + st) or ln.startswith(f"  {st}"):
                    colored = ln.replace(st, c(st), 1)
                    break
        print(colored)

    out = REPO / "logs"
    try:
        out.mkdir(exist_ok=True)
        stem = out / f"preflight_{started:%Y%m%d_%H%M%S}"
        stem.with_suffix(".txt").write_text("\n".join(lines) + "\n\nDetail:\n" +
                                            "\n".join(
            f"{r['status']:4} [{r['section']}] {r['name']}"
            + (f": {r['detail']}" if r["detail"] else "")
            for r in R.results) + "\n")
        stem.with_suffix(".json").write_text(json.dumps(dict(
            started=started.isoformat(), mode=mode, verdict=verdict,
            counts=R.counts(), thresholds=TH, results=R.results), indent=2))
        print(f" report saved: {stem.with_suffix('.txt').relative_to(REPO)}")
    except OSError as e:
        print(f" (could not save report: {e})")
    sys.exit(1 if interrupted else code)


if __name__ == "__main__":
    main()
