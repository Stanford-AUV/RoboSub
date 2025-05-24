#!/usr/bin/env python3

import sys
import time
import serial
import rclpy
from rclpy.node import Node
from msgs.msg import HydroDataStamped
from std_msgs.msg import Header

# Sensor and calibration constants
SeaWater = False
CTD_port = "/dev/cu.usbserial-FT6FJLX0"
COM_TEST = 0x00
COM_PCMODE = 0x0C
COM_ONLINE_MEAS = 0x01
ACK = 0x55
TIMEOUT = 2

T_COEFF = [122.622785746828, -0.138854530877331, 0.000108169890868935,
           -5.58470579894668E-8, 1.53702000127998E-11, -1.81435671827578E-15]

P_COEFF = [-1.61597635237222, 0.00565052106231862, -9.09681400791005E-8,
           4.90908801913798E-11, -8.71492645777175E-15, -4.81753801054678E-19]

PTC_COEFF = [7.02439662414173, -0.21053250673308, 0.00980786039230989,
             -0.000240862172070564, 2.17405487656103E-6]

Cond_COEFF = [98.0544546827358, -0.263561387205901, 0.000376092808984272,
              -3.09143413610036E-7, 1.50598310444937E-10, -4.27928881371478E-14,
              6.5323443704887E-18, -4.12913234939624E-22]

CTC_COEFF = [-0.398142680468083, -0.00259321862905614, -0.000684962594896168,
             2.30924943510067E-5, -1.76713340491716E-7]

CTC1_COEFF = [-0.276279974677843, -0.0925221052164181, 0.00180276949585506,
              -1.54831091575708E-5, 2.09671367968997E-7]

Tcr = 23.88
Tpr = 22.44
L = 549
H = 3146
SeawaterDensity = 1.026
g = 9.80665
gc = 100 / g

def poly_eval(coeffs, x):
    return sum(c * (x ** i) for i, c in enumerate(coeffs))

def temp_C(T_raw):
    return poly_eval(T_COEFF, T_raw)

def pressure_bar(P_raw, T_c):
    Pc = P_raw + sum(
        (PTC_COEFF[i] * (Tpr ** (i + 1)) - PTC_COEFF[i] * (T_c ** (i + 1)))
        for i in range(len(PTC_COEFF))
    )
    return poly_eval(P_COEFF, Pc)

def depth_m(P_bar):
    return P_bar * gc / SeawaterDensity if SeaWater else P_bar * gc

def conductivity_mScm(C_raw, T_c):
    Cc0 = C_raw + sum((CTC_COEFF[i] * (Tcr ** (i + 1)) - CTC_COEFF[i] * (T_c ** (i + 1)))
                      for i in range(len(CTC_COEFF)))
    Cc1 = C_raw + sum((CTC1_COEFF[i] * (Tcr ** (i + 1)) - CTC1_COEFF[i] * (T_c ** (i + 1)))
                      for i in range(len(CTC1_COEFF)))
    A = (Cc1 - Cc0) / (H - L)
    B = Cc0 - A * L
    Cc = B + A * C_raw
    return poly_eval(Cond_COEFF, Cc)

class CTDSensorNode(Node):
    def __init__(self):
        super().__init__('ctd_sensor_node')
        self.publisher_ = self.create_publisher(HydroDataStamped, '/hydrodata', 10)
        self.timer = self.create_timer(0.1, self.timer_callback)  # 10 Hz polling

        self.ser = serial.Serial(CTD_port, 4800, timeout=TIMEOUT)
        if not self.init_sensor():
            self.get_logger().error("Failed to initialize CTD sensor.")
            self.destroy_node()

    def init_sensor(self):
        self.ser.write(bytes([COM_TEST]))
        if self.ser.read(2) != bytes([COM_TEST, ACK]):
            return False

        self.ser.write(bytes([COM_PCMODE]))
        if self.ser.read(2) != bytes([COM_PCMODE, 0x02]):
            return False

        self.get_logger().info("CTD sensor initialized.")
        return True

    def timer_callback(self):
        self.ser.write(bytes([COM_ONLINE_MEAS]))
        if self.ser.read(1) == bytes([COM_ONLINE_MEAS]):
            self.ser.write(bytes([ACK]))
            data = self.ser.read(6)
            if len(data) == 6:
                T_raw = data[0] + 256 * data[1]
                P_raw = data[2] + 256 * data[3]
                C_raw = data[4] + 256 * data[5]

                T_c = temp_C(T_raw)
                P_bar = pressure_bar(P_raw, T_c)
                D_m = depth_m(P_bar)
                C_mScm = conductivity_mScm(C_raw, T_c)

                msg = HydroDataStamped()
                msg.header = Header()
                msg.header.stamp = self.get_clock().now().to_msg()
                msg.header.frame_id = "ctd_link"  # Set this to match your TF frame or leave empty
                msg.temperature = T_c
                msg.pressure = P_bar
                msg.depth = D_m
                msg.conductivity = C_mScm

                self.publisher_.publish(msg)
                self.get_logger().debug(f"Published: {msg}")

def main():
    rclpy.init()
    node = CTDSensorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down CTD sensor node.")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()