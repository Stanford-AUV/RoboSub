#!/usr/bin/env python3
"""One-time tool: burn the IMU mounting rotation (RotSensor alignment) into the
Xsens MTi so every output (quaternion, euler, gyro, accel, mag) arrives already
in the vehicle/base frame. Replaces the software remount in imu.py.

Wire protocol verified against the in-tree SDK
(lib/xspublic/xscontroller/mtibasedevice.cpp:115):
  SetAlignmentRotation = MID 0xEC, data = frame byte (0=sensor, 1=local)
                         + quaternion as 4 big-endian floats [w, x, y, z]
  ReqAlignmentRotation  = MID 0xEC with only the frame byte; reply 0xED.
Device must be in config mode (GoToConfig 0x30 / GoToMeasurement 0x10).
The setting persists in device flash.

Usage (driver must NOT be running - it owns the port):
  python set_alignment.py                    # show current alignment (both frames)
  python set_alignment.py --from-sensors-yaml  # burn R_sensor_to_base from sensors.yaml
  python set_alignment.py --identity           # reset to identity
  python set_alignment.py --quat W X Y Z       # burn explicit quaternion
  add --invert to burn the inverse (if live verify shows the wrong direction)
"""
import argparse
import struct
import sys
import time

import numpy as np
import serial
from scipy.spatial.transform import Rotation as R

PORT_DEFAULT = "/dev/ttyUSB_imu"
BAUD = 115200

MID_GOTOCONFIG = 0x30
MID_GOTOCONFIG_ACK = 0x31
MID_GOTOMEASUREMENT = 0x10
MID_GOTOMEASUREMENT_ACK = 0x11
MID_SET_ALIGNMENT = 0xEC
MID_SET_ALIGNMENT_ACK = 0xED
FRAME_SENSOR = 0
FRAME_LOCAL = 1

SENSORS_YAML = "/home/robosub/RoboSub/src/hardware/hardware/sensors.yaml"


def xbus_frame(mid: int, data: bytes = b"") -> bytes:
    assert len(data) < 255
    body = bytes([0xFF, mid, len(data)]) + data
    cks = (-sum(body)) & 0xFF
    return bytes([0xFA]) + body + bytes([cks])


def read_msg(ser: serial.Serial, want_mid: int, timeout: float = 2.0):
    """Scan the byte stream for a valid xbus frame with the wanted MID."""
    deadline = time.time() + timeout
    buf = bytearray()
    while time.time() < deadline:
        b = ser.read(1)
        if not b:
            continue
        buf += b
        # resync to preamble
        while buf and buf[0] != 0xFA:
            buf.pop(0)
        if len(buf) < 5:
            continue
        length = buf[3]
        total = 4 + length + 1
        if len(buf) < total:
            continue
        frame = bytes(buf[:total])
        del buf[:total]
        if sum(frame[1:]) & 0xFF != 0:  # checksum over BID..CKS must be 0
            continue
        mid = frame[2]
        if mid == want_mid:
            return frame[4 : 4 + length]
        # else keep scanning (measurement data floods the port pre-config)
    return None


def transact(ser, mid, data, ack_mid, tries=5, timeout=2.0):
    for _ in range(tries):
        ser.reset_input_buffer()
        ser.write(xbus_frame(mid, data))
        resp = read_msg(ser, ack_mid, timeout)
        if resp is not None:
            return resp
    return None


def get_alignment(ser, frame):
    resp = transact(ser, MID_SET_ALIGNMENT, bytes([frame]), MID_SET_ALIGNMENT_ACK)
    if resp is None or len(resp) < 17:
        return None
    w, x, y, z = struct.unpack(">4f", resp[1:17])
    return np.array([w, x, y, z])


def set_alignment(ser, frame, q_wxyz):
    data = bytes([frame]) + struct.pack(">4f", *q_wxyz)
    resp = transact(ser, MID_SET_ALIGNMENT, data, MID_SET_ALIGNMENT_ACK)
    return resp is not None


def quat_from_sensors_yaml():
    import yaml

    with open(SENSORS_YAML) as f:
        cfg = yaml.safe_load(f)
    m = np.array(cfg["imu_0"]["R_sensor_to_base"], dtype=float)
    q = R.from_matrix(m).as_quat()  # scipy: [x,y,z,w]
    return np.array([q[3], q[0], q[1], q[2]]), m  # -> [w,x,y,z]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", default=PORT_DEFAULT)
    ap.add_argument("--from-sensors-yaml", action="store_true")
    ap.add_argument("--identity", action="store_true")
    ap.add_argument("--quat", nargs=4, type=float, metavar=("W", "X", "Y", "Z"))
    ap.add_argument("--invert", action="store_true", help="burn the inverse quaternion")
    ap.add_argument("--frame", choices=["sensor", "local"], default="sensor")
    args = ap.parse_args()

    frame = FRAME_SENSOR if args.frame == "sensor" else FRAME_LOCAL

    q = None
    if args.from_sensors_yaml:
        q, m = quat_from_sensors_yaml()
        print(f"R_sensor_to_base from sensors.yaml:\n{m}")
    elif args.identity:
        q = np.array([1.0, 0.0, 0.0, 0.0])
    elif args.quat:
        q = np.array(args.quat, dtype=float)
        q = q / np.linalg.norm(q)

    if q is not None and args.invert:
        q = np.array([q[0], -q[1], -q[2], -q[3]])  # unit-quat inverse = conjugate

    ser = serial.Serial(args.port, BAUD, timeout=0.2, exclusive=True)
    try:
        print("-> GoToConfig")
        if transact(ser, MID_GOTOCONFIG, b"", MID_GOTOCONFIG_ACK) is None:
            print("ERROR: no GoToConfig ack (is the xsens driver still running?)")
            sys.exit(1)

        for name, fr in (("sensor", FRAME_SENSOR), ("local", FRAME_LOCAL)):
            cur = get_alignment(ser, fr)
            print(f"current alignment[{name}] (w,x,y,z) = {cur}")

        if q is not None:
            print(f"-> SetAlignmentRotation frame={args.frame} quat(w,x,y,z)={q}")
            if not set_alignment(ser, frame, q):
                print("ERROR: set failed (no ack)")
                sys.exit(1)
            rb = get_alignment(ser, frame)
            print(f"read-back[{args.frame}] = {rb}")
            if rb is None or not np.allclose(rb, q, atol=1e-6):
                print("ERROR: read-back does not match!")
                sys.exit(1)
            print("OK: alignment burned and verified (persists in device flash).")
        else:
            print("(read-only: no --from-sensors-yaml/--identity/--quat given)")
    finally:
        print("-> GoToMeasurement")
        transact(ser, MID_GOTOMEASUREMENT, b"", MID_GOTOMEASUREMENT_ACK)
        ser.close()


if __name__ == "__main__":
    main()
