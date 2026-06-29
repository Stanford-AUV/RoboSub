# `xsens_mt` — host kernel module for the Xsens MTi IMU

The Xsens **MTi-200 VRU** (USB `2639:0012`) is a **native-USB** device, **not** an
FTDI serial chip. Its USB descriptor exposes a control interface (`if00`) and a
**bulk data interface (`if01`)**; the correct Linux driver is the in-tree
`xsens_mt` usb-serial driver, which binds `if01` and presents a raw `/dev/ttyUSB*`
with **no** control/baud handshakes.

Our Jetson runs **L4T R36.5.0** (`5.15.x-tegra`), which is built with
`CONFIG_USB_SERIAL_XSENS_MT` **unset** — so the kernel has no driver for the IMU.
This directory ships that driver out-of-tree.

## Symptom this fixes

If the IMU is force-bound to `ftdi_sio` (e.g. via a `new_id`), every FTDI vendor
control transfer returns **`-32` (EPIPE/STALL)** — `"Unable to read/write latency
timer: -32"`, baud never sets, reads are garbage, `/imu/data` never publishes.
That STALL is the device *correctly rejecting* FTDI commands it doesn't implement,
because **it isn't an FTDI chip**. The fix is to use `xsens_mt`, not ftdi_sio.

## Install / rebuild

Run on the **host** (not inside the devcontainer). Re-run after any kernel update:

```bash
bash drivers/xsens_mt/install.sh
```

This builds `xsens_mt.ko` against `/lib/modules/$(uname -r)/build`, installs it to
`.../updates/`, runs `depmod`, enables auto-load (`/etc/modules-load.d/xsens_mt.conf`),
installs the udev rule, and verifies `/dev/ttyUSB_imu`.

## What you get

- `/dev/ttyUSB_imu` → the `xsens_mt` tty for `if01` (stable across reboots/replugs;
  matched by VID/PID, not by `ttyUSBn` number).
- The ROS driver is configured for it: `xsens_mti_node.yaml` has
  `scan_for_devices: false`, `port: /dev/ttyUSB_imu`.
- The devcontainer bind-mounts `/dev/ttyUSB_imu`; `ports.sh` chmods it.

Verify: `ros2 launch main hardware.py` → `ros2 topic hz /imu/data` (~400 Hz).

## Notes / caveats

- The module is **unsigned**; it loads because module-signature enforcement is off
  on this device (you'll see a one-line "tainting kernel" message — harmless).
- It is built for **one specific kernel version**. A kernel/L4T upgrade requires
  re-running `install.sh` (kept simple deliberately; switch to DKMS if upgrades
  become frequent).
- The DVL is a *real* FTDI **FT232R** (`0403:6001`) and correctly uses `ftdi_sio`.
  This driver only matches Xsens VID `2639`, so the two never conflict.
