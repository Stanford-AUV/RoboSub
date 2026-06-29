#!/bin/bash
#
# Build + install the xsens_mt kernel module so the Xsens MTi IMU works on this
# Jetson (L4T R36.5.0 ships CONFIG_USB_SERIAL_XSENS_MT unset). Idempotent:
# safe to re-run, and you MUST re-run it after any kernel/L4T update (the .ko
# is built for one specific kernel version).
#
# Run from the host (NOT inside the devcontainer):
#     bash drivers/xsens_mt/install.sh
# It will sudo for the privileged steps.
#
set -euo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
KREL="$(uname -r)"

echo "==> Building xsens_mt.ko for kernel ${KREL}"
if [[ ! -e "/lib/modules/${KREL}/build/Makefile" ]]; then
    echo "ERROR: no kernel build dir at /lib/modules/${KREL}/build" >&2
    echo "       Install the matching linux-headers / kernel-source first." >&2
    exit 1
fi
make -C "${HERE}" clean >/dev/null 2>&1 || true
make -C "${HERE}"

echo "==> Installing module to /lib/modules/${KREL}/updates/"
sudo install -d "/lib/modules/${KREL}/updates"
sudo cp -v "${HERE}/xsens_mt.ko" "/lib/modules/${KREL}/updates/xsens_mt.ko"
sudo depmod -a

echo "==> Auto-load at boot"
echo "xsens_mt" | sudo tee /etc/modules-load.d/xsens_mt.conf >/dev/null

echo "==> udev rule -> stable /dev/ttyUSB_imu"
sudo tee /etc/udev/rules.d/99-xsens-imu.rules >/dev/null <<'EOF'
# Xsens MTi-200 VRU -- native-USB device (VID 2639 / PID 0012).
# IMPORTANT: NOT an FTDI chip. Must use the in-kernel xsens_mt usb-serial
# driver (raw bulk), never ftdi_sio (its FTDI vendor ctrl transfers STALL -32).
# xsens_mt exposes only interface if01 (bulk data) as a tty, so idVendor+
# idProduct uniquely identify it. (All ATTRS in one udev rule must match the
# SAME parent device, so do NOT also match bInterfaceNumber here.)
SUBSYSTEM=="tty", ATTRS{idVendor}=="2639", ATTRS{idProduct}=="0012", SYMLINK+="ttyUSB_imu", MODE="0666", GROUP="dialout"
EOF

echo "==> Loading module + applying udev now"
# If ftdi_sio mistakenly holds the device (e.g. a stale new_id), reloading it
# drops any runtime-added id; xsens_mt then claims interface 1.
if [[ -d /sys/module/ftdi_sio ]]; then
    sudo modprobe -r ftdi_sio 2>/dev/null || true
    sudo modprobe ftdi_sio 2>/dev/null || true
fi
sudo modprobe xsens_mt 2>/dev/null || sudo insmod "/lib/modules/${KREL}/updates/xsens_mt.ko" 2>/dev/null || true
sudo udevadm control --reload-rules
sudo udevadm trigger --subsystem-match=tty --action=add
sudo udevadm settle --timeout=5 || true

echo "==> Result"
if [[ -e /dev/ttyUSB_imu ]]; then
    echo "OK: /dev/ttyUSB_imu -> $(readlink -f /dev/ttyUSB_imu)"
    T="$(basename "$(readlink -f /dev/ttyUSB_imu)")"
    echo "    driver: $(basename "$(readlink -f /sys/class/tty/${T}/device/driver 2>/dev/null || echo none)")"
    echo "    by-id : $(ls /dev/serial/by-id/ 2>/dev/null | grep -i xsens || echo '(none)')"
    echo "DONE. The IMU is on /dev/ttyUSB_imu (xsens_mt). Launch with: ros2 launch main hardware.py"
else
    echo "WARNING: /dev/ttyUSB_imu not present. Is the IMU plugged in?" >&2
    echo "Check: lsusb | grep 2639 ; dmesg | tail" >&2
    exit 1
fi
