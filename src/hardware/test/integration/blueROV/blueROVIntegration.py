#!/usr/bin/env python3
###############################################################################
# @internal
# @note
#     Copyright &copy; 2020 Teledyne RD Instruments
#     All rights reserved.
#
# @attention
#     MIT License
#
# Copyright (c) 2020 Teledyne Marine
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.
#
# @author nhui
#
# @date    Aug 11, 2020
#
# DATE      WHO   DESCRIPTION
# --------  ---   -------------------------------------------------------
# 10/22/20  NH    Updated docstrings
# 10/08/20  NH    Fixed time, refactored
# 09/29/20  NH    Added prints for beam and error codes
# 09/23/20  NH    Fixed print
# 09/21/20  NH    Fixed transformation
# 09/10/20  NH    Removed Scipy, Removed engineering driver, added clean exit,
#                   added time sync
# 09/09/20  NH    Ported changes from test script, fixed transforms, fixed log
# 08/11/20  NH    Initial commit, fixed transform, updated callback signature
###############################################################################

import time
from pymavlink import mavutil
from dvl.dvl import Dvl
from dvl.system import OutputData
import math
import datetime as dt
import numpy as np
import traceback
import struct
import os
import signal
import sys

wayFinderPort = "/dev/ttyUSB0"

roll = np.radians(180)
pitch = np.radians(0)
yaw = np.radians(135)

prevTime = 0
data = []

def wayfinderDataCallback(dataObj: OutputData, *args):
    '''WayFinder Data Callback Function
    
    WayFinder Data Callback - this processed the data from the WayFinder and 
    passes it to the PixHawk autopilot
    
    Arguments:
        dataObj {OutputData} -- WayFinder output data
    '''
    global master
    global wayfinderOffset_us
    global AP_offset_us
    global prevPose
    global currAtt
    global prevTime
    global sensor
    global logFile
    global rotMatrix

    try:
        dataTimestamp_us = dataObj.get_date_time().timestamp() * 1e6
        logFile.write("%9d: " % dataTimestamp_us)

        dataTimestamp_boot_us = int(dataTimestamp_us + AP_offset_us)
        logFile.write("%9d, " % dataTimestamp_boot_us)

        if math.isnan(dataObj.vel_x) or math.isnan(dataObj.vel_y) or \
                math.isnan(dataObj.vel_z) or math.isnan(dataObj.vel_err):
            logFile.write("NaN velocities\n")
            return

        if dataObj.is_velocity_valid():
            vels = np.array([dataObj.vel_x, dataObj.vel_y, dataObj.vel_z])   # m/s
            logFile.write("%9.3f %9.3f %9.3f, " % (vels[0], vels[1], vels[2]))
        else:
            logFile.write("Invalid velocities\n")
            return
        vels =  np.matmul(rotMatrix, vels)

        deltaTime_us = int(dataTimestamp_boot_us - prevTime)
        if prevTime == 0:
            deltaTime_us = 0
        logFile.write("%9d, " % deltaTime_us)

        if deltaTime_us < 0:
            logFile.write("\n")
            return

        angle_delta = currAtt - prevPose[0:3]
        logFile.write("%9.3f %9.3f %9.3f, " % (angle_delta[0], angle_delta[1], \
            angle_delta[2]))
        position_delta = vels * deltaTime_us * 1e-6
        logFile.write("%9.3f %9.3f %9.3f, " % (vels[0], vels[1], vels[2]))

        vel_err_max = 1
        vel_err_min = 0.001
        pct_err_vel = 0.005
        vel_mean = np.mean(vels)
        confidence = 1 - (vel_mean - vel_err_min) / (vel_err_max - vel_err_min)

        master.mav.vision_position_delta_send(dataTimestamp_boot_us, 
            deltaTime_us, angle_delta, position_delta, confidence)
        print("%9.3f %9.3f %9.3f | %9.3f | %9.3f %9.3f %9.3f %9.3f | %s" % \
            (vels[0], vels[1], vels[2], dataObj.vel_err, dataObj.range_beam1, \
                dataObj.range_beam2, dataObj.range_beam3, dataObj.range_beam4, \
                dataObj.bit_code))
        prevTime = dataTimestamp_boot_us
        prevPose[0:3] = currAtt
        data.append([dataObj, AP_offset_us, currAtt, prevPose])
        logFile.write("\n")
    except Exception as e:
        print(e)
        traceback.print_exc()
        logFile.write("EXCEPTION\n")

    logFile.flush()

def signal_handler(*args, **kwargs):
    '''Signal handler to shut down integration cleanly
      
    Arguments:
    '''
    global sensor
    sensor.disconnect()
    sys.exit(0)

def host_setup():
    """Sets up the host system interface
    
    In this case, we connect to the Ardupilot on TCP port 14777 on localhost.
    This is the MAVProxy instance that is running on the companion computer.
    """
    global master
    master = mavutil.mavlink_connection('tcp:localhost:14777')
    master.wait_heartbeat()


def main_loop():
    """Main execution loop
    
    This handles grabbing the rotational velocity data from the ArduPilot to 
    feed back in for making visual odometry work.
    """
    global master
    global AP_offset_us
    global AP_Timestamp_us
    global currAtt

    print("Started loop")
    while run:
        message = master.recv_match(type=['ATTITUDE', "GLOBAL_POSITION_INT"],
            blocking=True, timeout=30)
        if not message:
            continue
        else:
            messageTimestamp_us = dt.datetime.now().timestamp() * 1e6
            AP_Timestamp_us = message.time_boot_ms * 1e3
            AP_offset_us = AP_Timestamp_us - messageTimestamp_us
            if message.get_type() == 'ATTITUDE':
                messageData = message.to_dict()
                currAtt[0] = messageData['roll']
                currAtt[1] = messageData['pitch']
                currAtt[2] = messageData['yaw']
            else:
                print("Unknown")

def log_setup():
    """Configures the data logging
    
    """
    global logFile

    lastLogPath = os.path.join("/home/pi/logs", "lastlog.txt")
    try:
        with open(lastLogPath, 'r') as lastLog:
            logNum = int(lastLog.readline()) + 1
    except Exception:
        logNum = 0

    with open(lastLogPath, 'w') as lastLog:
        lastLog.write("%d\n" % logNum)

    logFile = open(os.path.join("/home/pi/logs", "%06d.log" % logNum), 'w')


def transform_setup():
    """Configures the coordinate transform
    
    """
    global rotMatrix
    global yaw, pitch, roll
    rotMatrix = np.array(\
    [
        [np.cos(yaw) * np.cos(pitch), 
        np.cos(yaw) * np.sin(pitch) * np.sin(roll) - np.sin(yaw) * np.cos(roll),
        np.cos(yaw) * np.sin(pitch) * np.cos(roll) + np.sin(yaw) * np.sin(roll)
        ],
        [np.sin(yaw) * np.cos(pitch), 
        np.sin(yaw) * np.sin(pitch) * np.sin(roll) + np.cos(yaw) * np.cos(roll),
        np.sin(yaw) * np.sin(pitch) * np.cos(roll) - np.cos(yaw) * np.sin(roll)
        ],
        [-np.sin(pitch),
        np.cos(pitch) * np.sin(roll),
        np.cos(pitch) * np.cos(roll)
        ]
    ])

def sensor_setup():
    """Configures the WayFinder
    
    This also synchronizes the WayFinder and host computer clocks.  We wait for
    the top of the 2nd 1 second interval before sending the set time command.
    """
    global sensor

    sensor = Dvl()



    while not sensor.connect(wayFinderPort, 115200):
        pass

    sensor.reset_to_defaults()
    sensor.enter_command_mode()
    twoseconds = dt.datetime.now() + dt.timedelta(seconds=2)
    timetarget = dt.datetime(twoseconds.year, twoseconds.month, twoseconds.day,
        twoseconds.hour, twoseconds.minute, twoseconds.second)
    now = dt.datetime.now()
    time.sleep((timetarget - now).total_seconds())
    
    sensor.set_time(dt.datetime.now())

    sensor.exit_command_mode()
    sensor.register_ondata_callback(wayfinderDataCallback, None)

if __name__ == '__main__':
    global wayfinderOffset_us
    global AP_offset_us
    global prevPose
    global currAtt
    global run

    run = True
    prevPose = np.zeros(6)
    currAtt = np.zeros(3)
    AP_offset_us = -dt.datetime.now().timestamp() * 1e6
    wayfinderOffset_us = 0

    log_setup()
    transform_setup()

    host_setup()

    sensor_setup()

    signal.signal(signal.SIGINT, signal_handler)

    main_loop()
