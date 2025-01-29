# Modified for SAUV.
#
# Copyright 2011 Brown University Robotics.
# Copyright 2017 Open Source Robotics Foundation, Inc.
# All rights reserved.
#
# Software License Agreement (BSD License 2.0)
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of the Willow Garage nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

import sys
import threading

from geometry_msgs.msg import WrenchStamped
import rclpy

if sys.platform == "win32":
    import msvcrt
else:
    import termios
    import tty


msg = """
This node takes keypresses from the keyboard and publishes them
as Wrench/WrenchStamped messages. It works best with a US keyboard layout.
---------------------------
Moving around:
   u    i    o
   j    k    l
   m    ,    .

For Holonomic mode (strafing), hold down the shift key:
---------------------------
   U    I    O
   J    K    L
   M    <    >

t : up (+z)
b : down (-z)

anything else : stop

q/z : increase/decrease max forces by 10%
w/x : increase/decrease only linear force by 10%
e/c : increase/decrease only angular force by 10%

CTRL-C to quit
"""

moveBindings = {
    "i": (-1, 0, 0, 0, 0, 0),
    "o": (1, 0, 0, 0, 0, -1),
    "j": (0, 0, 0, 0, 0, 1),
    "l": (0, 0, 0, 0, 0, -1),
    "u": (1, 0, 0, 0, 0, 1),
    ",": (1, 0, 0, 0, 0, 0),
    ".": (-1, 0, 0, 0, 0, 1),
    "m": (-1, 0, 0, 0, 0, -1),
    "O": (1, -1, 0, 0, 0, 0),
    "I": (1, 0, 0, 0, 0, 0),
    "J": (0, 1, 0, 0, 0, 0),
    "L": (0, -1, 0, 0, 0, 0),
    "U": (1, 1, 0, 0, 0, 0),
    "<": (-1, 0, 0, 0, 0, 0),
    ">": (-1, -1, 0, 0, 0, 0),
    "M": (-1, 1, 0, 0, 0, 0),
    "t": (0, 0, -1, 0, 0, 0),
    "b": (0, 0, 1, 0, 0, 0),
}

forceBindings = {
    "q": 1.1,
    "z": 0.9,
    "w": 1.1,
    "x": 0.9,
}

torqueBindings = {
    "e": 1.1,
    "c": 0.9,
}


def getKey(settings):
    if sys.platform == "win32":
        # getwch() returns a string on Windows
        key = msvcrt.getwch()
    else:
        tty.setraw(sys.stdin.fileno())
        # sys.stdin.read() returns a string on Linux
        key = sys.stdin.read(1)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key


def saveTerminalSettings():
    if sys.platform == "win32":
        return None
    return termios.tcgetattr(sys.stdin)


def restoreTerminalSettings(old_settings):
    if sys.platform == "win32":
        return
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)


def forces(force, torque):
    return "currently:\tforce %s\ttorque %s " % (force, torque)


def main():
    settings = saveTerminalSettings()

    rclpy.init()

    node = rclpy.create_node("keyboardcontrol")

    pub = node.create_publisher(WrenchStamped, "wrench", 10)

    spinner = threading.Thread(target=rclpy.spin, args=(node,))
    spinner.start()

    force = 0.5
    torque = 1.0
    x = 0.0
    y = 0.0
    z = 0.0
    th = 0.0
    status = 0.0

    wrench_stamped = WrenchStamped()

    wrench = wrench_stamped.wrench
    wrench_stamped.header.stamp = node.get_clock().now().to_msg()

    try:
        print(msg)
        print(forces(force, torque))
        while True:
            key = getKey(settings)
            if key in moveBindings.keys():
                x = moveBindings[key][0]
                y = moveBindings[key][1]
                z = moveBindings[key][2]
                th = moveBindings[key][5]
            elif key in forceBindings.keys():
                force *= forceBindings[key]
                print(forces(force, torque))
                if status == 14:
                    print(msg)
                status = (status + 1) % 15
            elif key in torqueBindings.keys():
                torque *= torqueBindings[key]
                print(forces(force, torque))
                if status == 14:
                    print(msg)
                status = (status + 1) % 15
            else:
                x = 0.0
                y = 0.0
                z = 0.0
                th = 0.0
                if key == "\x03":
                    break

            wrench_stamped.header.stamp = node.get_clock().now().to_msg()

            wrench.force.x = x * force
            wrench.force.y = y * force
            wrench.force.z = z * force
            wrench.torque.x = 0.0
            wrench.torque.y = 0.0
            wrench.torque.z = th * torque
            pub.publish(wrench_stamped)

    except Exception as e:
        print(e)

    finally:
        print("Stopping...")

        wrench_stamped.header.stamp = node.get_clock().now().to_msg()

        wrench.force.x = 0.0
        wrench.force.y = 0.0
        wrench.force.z = 0.0
        wrench.torque.x = 0.0
        wrench.torque.y = 0.0
        wrench.torque.z = 0.0
        pub.publish(wrench_stamped)
        rclpy.shutdown()
        spinner.join()

        restoreTerminalSettings(settings)


if __name__ == "__main__":
    main()
