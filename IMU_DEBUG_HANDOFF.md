# Xsens IMU `-32` debug — handoff

> ## ✅ RESOLVED (2026-06-28)
> **The diagnosis below was WRONG — it is NOT a tegra-xusb/kernel USB bug; no root-port move or L4T reflash is needed.**
>
> The MTi-200 (`2639:0012`) is a **native-USB device, not an FTDI chip** (descriptor: bDeviceClass=2, if00=interrupt control, if01=bulk data). L4T R36.5.0 ships with `CONFIG_USB_SERIAL_XSENS_MT` **unset**, so nothing claimed it; this debug session had force-bound it to `ftdi_sio` via `new_id`. ftdi_sio then sent FTDI vendor control requests to a non-FTDI device, which **correctly STALLs** them → the deterministic `-32`/EPIPE. (Standard control worked; only FTDI-vendor control stalled — proof it's the device rejecting FTDI commands, not the controller.)
>
> **Fix (done, verified `/imu/data` @ ~400 Hz):** build + install the in-kernel **`xsens_mt`** driver out-of-tree, add a udev rule for a stable `/dev/ttyUSB_imu`, point the yaml at it. All packaged in **[`drivers/xsens_mt/`](drivers/xsens_mt/README.md)** — run `bash drivers/xsens_mt/install.sh` (re-run after any kernel update).
>
> Everything below is the original (mis)diagnosis, kept for history only.

---

## Goal
Make the Xsens MTi-200 VRU IMU work so `ros2 launch main hardware.py` connects and `/imu/data` publishes. It worked on a previous Orin (now wiped & gone) on the SAME wiring/hub, so this is a kernel/software regression on the current Orin, not hardware.

## System / device
- Jetson AGX Orin, L4T **R36.5.0** (kernel `5.15.185-tegra`, built 2026-01-16), JetPack 6.
- IMU: Xsens MTi-200 VRU, USB **2639:0012**, uses an FTDI **FT2232C** (dual-channel, full-speed/USB1.1, ep0 maxpacket=8, exposes 2 interfaces → ttyUSB data=if00).
- USB tree: root(bus1,480M) port4 → onboard Realtek hub `0bda:5420` → [Teensy(ttyACM0) on port1; a SECOND added hub on port4 → DVL FT232R + camera]. DVL FT232R works fine.

## Symptom
`ftdi_sio` attaches ttyUSB0/1 but every FTDI control transfer returns **`-32` (EPIPE/STALL)**: "Unable to read/write latency timer: -32", set-baudrate / flow-control fail → baud never set → reads are garbage (no `FA FF` Xsens preamble).

## RULED OUT — do NOT redo
- Not the IMU hardware (worked on old Orin).
- Not the hub/TT: DVL FT232R works through the SAME second-hub TT where the MTi fails. MTi tested on onboard hub (`1-4.2`) AND second hub (`1-4.4.1`) — identical failure.
- Not ModemManager (masked), not autosuspend (off), not new_id binding (binds OK), not device-node placeholder (fixed).
- Not USB quirks: tested n/e/b/k + combos via `/sys/module/usbcore/parameters/quirks` → all `-32`.
- Not flaky: 12/12 re-enumeration retries all `-32` (deterministic).
- Not `ftdi_sio`: RAW usbfs control transfers (ftdi_sio unbound, libusb-style) reproduce `-32` on ALL FTDI **vendor** requests (0x0A,0x05,0x00,0x03) at wIndex 0/1/2, while a standard `GET_DESCRIPTOR` on the same ep0 SUCCEEDS. => standard control works; FTDI vendor control is STALLed; fault is BELOW the serial driver, at tegra-xusb/kernel.

## Leading diagnosis
tegra-xusb (xHCI) USB stack in R36.5.0 mishandles this FT2232C's vendor control transfers. Fix is a kernel/L4T change OR (untested) removing the TT by going to a true root port. Matches NVIDIA forum t/265551 (Xsens MTi+FTDI on AGX Orin, fixed only by changing L4T).

## NEXT ACTIONS (priority order)
1. **Cheapest, decisive, NOT yet done — true root port.** Plug MTi into a USB-C port on the Orin (the only free root port = bus1 port2) via a USB-A-female→USB-C-male OTG adapter. Confirm sysfs path is a **NO-DOT path** like `1-2` (currently `1-4.4.1`). Re-run the raw-control probe. If vendor requests succeed at root → fix = keep MTi on a direct root/USB-C port (no hub). If still `-32` → controller bug independent of TT.
2. **If root doesn't fix it: change the kernel.** Reflash to an older JetPack 6 L4T (try R36.4.3 or R36.3.0), or swap just the kernel Image + tegra-xusb module from an older L4T. (Old Orin's exact version unrecoverable.)
3. **Once USB works:** install udev rule, verify `/dev/ttyUSB_imu`, run `ros2 launch main hardware.py`, confirm `/imu/data`.

## Already done (config ready for when USB works)
- `src/xsens_mti_ros2_driver/param/xsens_mti_node.yaml`: `scan_for_devices: false`, `port: "/dev/ttyUSB0"` (if00 data interface).
- udev rule prepared (NOT installed): registers ftdi_sio new_id for 2639:0012 + `ttyUSB_imu` symlink. Was at scratchpad `99-xsens-ftdi.rules`; recreate and install: `sudo cp 99-xsens-ftdi.rules /etc/udev/rules.d/ && sudo udevadm control --reload`.

## How-to for the agent
- No passwordless sudo (user runs sudo manually). Agent is in `docker` group → do root sysfs/usbfs ops via a privileged container:
  `docker run --rm --privileged -v /sys:/sys -v /dev:/dev -v <dir>:/work vsc-robosub-224659a0d7e804d9a32ae7664c9d5a77772cc7ad67945b2abfbd527aa2984d58-uid:latest bash /work/script.sh`
- Raw control probe (USBDEVFS_CONTROL=0xC0185500): set `DEV=<usb-path>`, unbind ftdi_sio from `:1.0`/`:1.1`, ioctl GET_LATENCY(0xC0,0x0A,wLen1)/GET_DESCRIPTOR(0x80,0x06,0x0100,wLen18). See prior rawctrl.py logic.
- Find MTi path: `for d in /sys/bus/usb/devices/*/; do [ "$(cat $d/idVendor 2>/dev/null)" = 2639 ] && echo $(basename $d); done`
