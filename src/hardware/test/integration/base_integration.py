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
# @date    Oct 08, 2020
#
# DATE      WHO   DESCRIPTION
# --------  ---   -------------------------------------------------------
# 10/22/20  NH    Updated docstrings
# 10/08/20  NH    Initial commit, port from blueROVIntegration
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

def reportVelocities(vels, confidence):
    """Reports velocities to the host computer
    
    This is a callback for reporting velocities to the host computer.
    Velocities are in m/s, confidence is from 0 to 100%
    
    Arguments:
        vels {np.ndarray} -- 3 element array containing the X, Y, and Z 
        velocities in the vehicle coordinate frame
        confidence {np.float64} -- Value from 0 to 100 denoting the confidence
        in the velocities provided.
    """
    pass

def wayfinderDataCallback(dataObj: OutputData, *args):
    '''WayFinder Data Callback Function
    
    WayFinder Data Callback - this processed the data from the WayFinder and 
    passes it to the PixHawk autopilot
    
    Arguments:
        dataObj {OutputData} -- WayFinder output data
    '''
    global AP_offset_us    # This is the Autopilot time offset from the WayFinder in us
    global prevPose
    global currAtt
    global linearVel
    global prevTime
    global sensor
    global logFile
    global rotMatrix

    try:
        dataTimestamp_us = dataObj.get_date_time().timestamp() * 1e6
        logFile.write("%9d: " % dataTimestamp_us)

        dataTimestamp_boot_us = int(dataTimestamp_us + AP_offset_us)
        logFile.write("%9d, " % dataTimestamp_boot_us)

        # Check velocities are valid
        if math.isnan(dataObj.vel_x) or math.isnan(dataObj.vel_y) or math.isnan(dataObj.vel_z) or math.isnan(dataObj.vel_err):
            logFile.write("NaN velocities\n")
            return
        
        # Use the API to check - don't really trust this right now
        if dataObj.is_velocity_valid():
            vels = np.array([dataObj.vel_x, dataObj.vel_y, dataObj.vel_z])   # m/s
            logFile.write("%9.3f %9.3f %9.3f, " % (vels[0], vels[1], vels[2]))
        else:
            logFile.write("Invalid velocities\n")
            return
            
        # Velocity transform to vehicle frame
        vels =  np.matmul(rotMatrix, vels)

        # Time since last
        deltaTime_us = int(dataTimestamp_boot_us - prevTime)
        if prevTime == 0:
            deltaTime_us = 0
        logFile.write("%9d, " % deltaTime_us)

        # Something has gone really wrong
        if deltaTime_us < 0:
            logFile.write("\n")
            return

        # Confidence calculation
        vel_err_max = 1
        vel_err_min = 0.001
        pct_err_vel = 0.005
        vel_mean = np.mean(vels)
        confidence = 1 - (vel_mean - vel_err_min) / (vel_err_max - vel_err_min)

        reportVelocities(vels, confidence)

        print("%9.3f %9.3f %9.3f | %9.3f | %9.3f %9.3f %9.3f %9.3f | %s" % (vels[0], vels[1], vels[2], dataObj.vel_err, dataObj.range_beam1, dataObj.range_beam2, dataObj.range_beam3, dataObj.range_beam4, dataObj.bit_code))
        prevTime = dataTimestamp_boot_us
        data.append([dataObj, AP_offset_us])
        logFile.write("\n")
    except Exception as e:
        print(e)
        traceback.print_exc()
        logFile.write("EXCEPTION\n")

    logFile.flush()

def signal_handler(*args, **kwargs):
    """Signal handler to shut down integration cleanly
    
    """
    global sensor
    sensor.disconnect()
    sys.exit(0)

def host_setup():
    """Sets up the host system interface
    
    Any configuration of the host system interface should happen here.  This
    should include setting up the host system to accept velocity data
    """
    pass


def main_loop():
    """Main execution loop
    
    This handles grabbing any additional data from the host system to make
    the integration work.
    """
    global AP_offset_us

    print("Started loop")
    while run:
        pass

def log_setup():
    """Configures the data logging
    
    """
    global logFile
    
    logDir = "."

    lastLogPath = os.path.join(logDir, "lastlog.txt")
    try:
        with open(lastLogPath, 'r') as lastLog:
            logNum = int(lastLog.readline()) + 1
    except Exception:
        logNum = 0

    with open(lastLogPath, 'w') as lastLog:
        lastLog.write("%d\n" % logNum)

    logFile = open(os.path.join(logDir, "%06d.log" % logNum), 'w')


def transform_setup():
    """Configures the coordinate transform
    
    """
    global rotMatrix
    global yaw, pitch, roll
    rotMatrix = np.array([[np.cos(yaw) * np.cos(pitch), np.cos(yaw) * np.sin(pitch) * np.sin(roll) - np.sin(yaw) * np.cos(roll), np.cos(yaw) * np.sin(pitch) * np.cos(roll) + np.sin(yaw) * np.sin(roll)],
                          [np.sin(yaw) * np.cos(pitch), np.sin(yaw) * np.sin(pitch) * np.sin(roll) + np.cos(yaw) * np.cos(roll), np.sin(yaw) * np.sin(pitch) * np.cos(roll) - np.cos(yaw) * np.sin(roll)],
                          [ -np.sin(pitch), np.cos(pitch) * np.sin(roll), np.cos(pitch) * np.cos(roll)]])

def sensor_setup():
    """Configures the WayFinder
    
    This also synchronizes the WayFinder and host computer clocks.  We wait for
    the top of the 2nd 1 second interval before sending the set time command.
    """
    global sensor

    # Construct object
    sensor = Dvl()

    # Connect
    while not sensor.connect(wayFinderPort, 115200):
        pass

    # Reset
    sensor.reset_to_defaults()
    
    # Set Time
    sensor.enter_command_mode()
    twoseconds = dt.datetime.now() + dt.timedelta(seconds=2)
    timetarget = dt.datetime(twoseconds.year, twoseconds.month, twoseconds.day, twoseconds.hour, twoseconds.minute, twoseconds.second)
    now = dt.datetime.now()
    time.sleep((timetarget - now).total_seconds())
    sensor.set_time(dt.datetime.now())

    sensor.exit_command_mode()
    
    # Register data callback
    sensor.register_ondata_callback(wayfinderDataCallback, None)

if __name__ == '__main__':
    global AP_offset_us
    global run

    run = True
    AP_offset_us = -dt.datetime.now().timestamp() * 1e6

    log_setup()
    transform_setup()

    host_setup()

    sensor_setup()

    signal.signal(signal.SIGINT, signal_handler)

    main_loop()
