#!/usr/bin/env python3
# Utility to set the home position
# Date 		Who Description
# =============================================================================
# 09/29/20  NH  Initial commit, fixed args

from pymavlink import mavutil
import datetime as dt
import argparse
import time
import serial
import pynmea2
import utm
#input_position = (32.940244, -117.027927)

#endpoint='tcp:localhost:14777'


if __name__ == '__main__':
	parser = argparse.ArgumentParser(description="Sets the BlueROV home location")
	parser.add_argument('endpoint', type=str, help='Companion should be tcp:localhost:14777, GCS should be udpin:0.0.0.0:14550, surface assistant should be udpin:0.0.0.0:14550')
	group = parser.add_mutually_exclusive_group(required=True)
	group.add_argument('--gps_port', type=str)
	group.add_argument('--latlon', type=float, nargs=2)
	group.add_argument('--utm', type=str, nargs=4)
	
	args = parser.parse_args()
	endpoint=args.endpoint
	if args.gps_port is not None:
		# Use GPS NMEA
		port = serial.Serial(args.gps_port, baudrate=115200)
		while True:
			line = port.readline()
			try:
				msg = pynmea2.parse(line.decode())
			except ParseError:
				continue
			if msg.sentence_type == 'GGA':
				if msg.is_valid:
					input_position = (msg.latitude, msg.longitude)
					break
	elif args.latlon is not None:
		input_position = (args.latlon[0], args.latlon[1])
	elif args.utm is not None:
		input_position = utm.to_latlon(int(args.utm[2]), int(args.utm[3]), int(args.utm[0]), args.utm[1])
	print(input_position)
	print(endpoint)
	IGNORE_FLAG_ALL = (
		mavutil.mavlink.GPS_INPUT_IGNORE_FLAG_ALT |
		mavutil.mavlink.GPS_INPUT_IGNORE_FLAG_HDOP |
		mavutil.mavlink.GPS_INPUT_IGNORE_FLAG_VDOP |
		mavutil.mavlink.GPS_INPUT_IGNORE_FLAG_VEL_HORIZ |
		mavutil.mavlink.GPS_INPUT_IGNORE_FLAG_VEL_VERT |
		mavutil.mavlink.GPS_INPUT_IGNORE_FLAG_SPEED_ACCURACY |
		mavutil.mavlink.GPS_INPUT_IGNORE_FLAG_HORIZONTAL_ACCURACY |
		mavutil.mavlink.GPS_INPUT_IGNORE_FLAG_VERTICAL_ACCURACY)
	master = mavutil.mavlink_connection(endpoint)
	heartbeat = master.wait_heartbeat()
	master.mav.set_home_position_send(
		target_system=heartbeat.get_srcSystem(),
		latitude=int(input_position[0] * 1e7),
		longitude=int(input_position[1] * 1e7),
		altitude=0,
		x=0,
		y=0,
		z=0,
		q=[1,0,0,0],
		approach_x=0,
		approach_y=0,
		approach_z=1,
	)
	master.mav.set_gps_global_origin_send(
		target_system=heartbeat.get_srcSystem(),
		latitude=int(input_position[0] * 1e7),
		longitude=int(input_position[1] * 1e7),
		altitude=0
	)
	print("Success")
