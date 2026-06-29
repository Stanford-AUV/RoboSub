// SPDX-License-Identifier: GPL-2.0+
/*
 * Xsens MT USB driver
 *
 * Copyright (C) 2013 Xsens <info@xsens.com>
 *
 * Verbatim copy of the mainline Linux driver drivers/usb/serial/xsens_mt.c.
 * We ship it out-of-tree because L4T R36.5.0 (kernel 5.15.x-tegra) is built
 * with CONFIG_USB_SERIAL_XSENS_MT unset. The Xsens MTi-200 (USB 2639:0012)
 * is a NATIVE-USB device, NOT an FTDI chip: it exposes a control interface
 * (if00) and a bulk data interface (if01). It must bind this xsens_mt driver
 * (raw bulk, no control handshakes), never ftdi_sio -- ftdi_sio's FTDI vendor
 * control transfers STALL (-32/EPIPE) on this non-FTDI device.
 *
 * Build/install:  see install.sh in this directory.
 */

#include <linux/kernel.h>
#include <linux/tty.h>
#include <linux/module.h>
#include <linux/usb.h>
#include <linux/usb/serial.h>

#define XSENS_VID 0x2639

#define MTi_10_IMU_PID		0x0001
#define MTi_20_VRU_PID		0x0002
#define MTi_30_AHRS_PID		0x0003

#define MTi_100_IMU_PID		0x0011
#define MTi_200_VRU_PID		0x0012
#define MTi_300_AHRS_PID	0x0013

#define MTi_G_700_GPS_INS_PID	0x0014

static const struct usb_device_id id_table[] = {
	{ USB_DEVICE(XSENS_VID, MTi_10_IMU_PID) },
	{ USB_DEVICE(XSENS_VID, MTi_20_VRU_PID) },
	{ USB_DEVICE(XSENS_VID, MTi_30_AHRS_PID) },

	{ USB_DEVICE(XSENS_VID, MTi_100_IMU_PID) },
	{ USB_DEVICE(XSENS_VID, MTi_200_VRU_PID) },
	{ USB_DEVICE(XSENS_VID, MTi_300_AHRS_PID) },

	{ USB_DEVICE(XSENS_VID, MTi_G_700_GPS_INS_PID) },
	{ },
};
MODULE_DEVICE_TABLE(usb, id_table);

static int xsens_mt_probe(struct usb_serial *serial,
					const struct usb_device_id *id)
{
	if (serial->interface->cur_altsetting->desc.bInterfaceNumber == 1)
		return 0;

	return -ENODEV;
}

static struct usb_serial_driver xsens_mt_device = {
	.driver = {
		.owner =	THIS_MODULE,
		.name =		"xsens_mt",
	},
	.id_table =		id_table,
	.num_ports =		1,

	.probe =		xsens_mt_probe,
};

static struct usb_serial_driver * const serial_drivers[] = {
	&xsens_mt_device, NULL
};

module_usb_serial_driver(serial_drivers, id_table);

MODULE_AUTHOR("Frans Klaver <frans.klaver@xsens.com>");
MODULE_DESCRIPTION("Xsens MT USB driver");
MODULE_LICENSE("GPL v2");
