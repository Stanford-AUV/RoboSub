#!/usr/bin/env python3
"""
Orin -> Teensy heartbeat ticker  (pure stdlib: no pyserial / conda needed).

Sends 'heartbeat <seq>\\n' to the Teensy once per second.  Survives the Teensy
being unplugged / replugged / reflashed: on a write failure it drops the stale
handle and keeps retrying every tick, re-opening whatever /dev/ttyACM* node
the Teensy re-enumerates as.  (The Teensy is powered from the vehicle rail,
not USB, so pulling the USB cable does NOT reboot it -- its heartbeat
watchdog stays armed and the red LED flashes until beats resume.)

This script only WRITES to the port, so you can watch the firmware's replies
in a SEPARATE terminal without byte contention (needs HB_ACK_REPLY=1 in the
firmware to see acks):

    stty -F /dev/ttyACM0 9600 raw -echo
    cat /dev/ttyACM0

Needs read/write access to the port (be in the 'dialout' group, or chmod it).
Usage:  python3 heartbeat.py [interval_s] [duration_s] [port]
        port defaults to auto-detect (first /dev/ttyACM*, re-checked on
        every reconnect)
"""
import glob
import os
import sys
import time
import subprocess

INTERVAL_S = float(sys.argv[1]) if len(sys.argv) > 1 else 1.0  # tick period
DURATION_S = float(sys.argv[2]) if len(sys.argv) > 2 else 5.0  # stop after 5 s -> watchdog fires
FORCED_PORT = sys.argv[3] if len(sys.argv) > 3 else None       # None = auto-detect


def find_port():
    """First ttyACM node present, or the forced one if it exists."""
    if FORCED_PORT:
        return FORCED_PORT if os.path.exists(FORCED_PORT) else None
    ports = sorted(glob.glob("/dev/ttyACM*"))
    return ports[0] if ports else None


def open_port(port):
    """Open write-only (a separate `cat` can own all the return traffic).
    Returns an fd, or None if the port vanished / isn't accessible."""
    # best-effort line config (Teensy is native USB CDC; harmless if it fails)
    subprocess.run(["stty", "-F", port, "9600", "raw", "-echo"], check=False)
    try:
        return os.open(port, os.O_WRONLY | os.O_NOCTTY)
    except FileNotFoundError:
        return None
    except PermissionError:
        print(f">>> Permission denied on {port}. Fix once with:\n"
              f"      sudo usermod -aG dialout $USER   (then re-login)\n"
              f"    or: sudo chmod a+rw {port}", flush=True)
        return None
    except OSError:
        # e.g. EBUSY: the ROS node holds the port with TIOCEXCL (exclusive=True)
        return None


def ros_node_on_port():
    """True while the ROS 'arduino' node is running. Its pyserial open is
    exclusive (TIOCEXCL), which blocks NEW opens but does not evict an fd we
    already hold -- so we must yield the port ourselves, every tick, not just
    at startup."""
    ps = subprocess.run(["ps", "-eo", "cmd"], capture_output=True, text=True).stdout
    return "hardware/arduino" in ps


def main():
    fd = None
    port = None
    seq = 0
    sent = 0
    was_down = False        # so we only print one reconnect banner per outage
    paused_for_ros = False  # ditto for the ROS-owns-the-port pause banner
    t_start = time.monotonic()
    t_end = t_start + DURATION_S
    next_tick = t_start
    print(f">>> heartbeat every {INTERVAL_S}s for {DURATION_S:.0f}s "
          f"(port auto-detected, reconnects on unplug/replug)")
    try:
        while time.monotonic() < t_end:
            seq += 1

            # --- guard: never fight the ROS stack for the port ---
            if ros_node_on_port():
                if fd is not None:
                    os.close(fd)
                    fd = None
                if not paused_for_ros:
                    print(">>> BUSY: ROS 'arduino' node owns the port; pausing "
                          "beats until it exits (red LED will flash)...", flush=True)
                    paused_for_ros = True
                    was_down = True
                next_tick += INTERVAL_S
                nap = next_tick - time.monotonic()
                if nap > 0:
                    time.sleep(nap)
                continue
            elif paused_for_ros:
                paused_for_ros = False
                print(">>> ROS 'arduino' node gone; resuming beats", flush=True)

            # (Re)open the port if we don't have a live handle.
            if fd is None:
                port = find_port()
                if port is not None:
                    fd = open_port(port)
                if fd is not None:
                    print(f">>> {'re' if was_down else ''}connected on {port}", flush=True)
                    was_down = False

            if fd is None:
                if not was_down:
                    print(">>> no /dev/ttyACM* port; retrying every tick...", flush=True)
                    was_down = True
            else:
                try:
                    os.write(fd, f"heartbeat {seq}\n".encode())
                    sent += 1
                    print(f"[orin -> teensy] heartbeat {seq}", flush=True)
                except OSError as e:
                    # Stale handle: the Teensy re-enumerated (unplug/reflash).
                    # Drop it; next tick re-opens whatever node it came back as.
                    print(f"!!! write failed on seq {seq} ({e}); reconnecting...",
                          flush=True)
                    os.close(fd)
                    fd = None
                    was_down = True

            next_tick += INTERVAL_S
            nap = next_tick - time.monotonic()
            if nap > 0:
                time.sleep(nap)
    except KeyboardInterrupt:
        print("\n>>> interrupted by user")
    finally:
        if fd is not None:
            os.close(fd)
        print(f">>> done: sent {sent} heartbeats in "
              f"{time.monotonic() - t_start:.1f}s, port released")


if __name__ == "__main__":
    main()
