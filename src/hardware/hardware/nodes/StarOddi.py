import serial
import time
#############################################################################################
###################################### For Software #########################################
#############################################################################################
# Star-Oddi CTD Sensor
# Reminders:
# - make sure to find the correct COM port
# - max polling rate is 50ms (20Hz), do not exceed this
# - baud rate is 4800
SeaWater = False  # Set to True if in seawater, False if in freshwater
CTD_port = "/dev/cu.usbserial-FT6FJLX0" # change for the Orin

# exmaple ouput:
#Testing connection...
#Setting to PC mode...
#Polling measurements...
#Raw values: T=1768, P=276, C=48
#Converted: Temp=25.448 °C, Pressure=-0.133 bar, Depth=-1.35 m, Conductivity=86.074 mS/cm

#############################################################################################
############################## Commmunication Constants #####################################
#############################################################################################
COM_TEST = 0x00
COM_VERSION = 0x04
COM_PCMODE = 0x0C
COM_ONLINE_MEAS = 0x01
ACK = 0x55
TIMEOUT = 2

#############################################################################################
############################### Measurement Calibration #####################################
#############################################################################################
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
    if SeaWater:
        return P_bar * gc / SeawaterDensity
    else:
        return P_bar * gc

def conductivity_mScm(C_raw, T_c):
    Cc0 = C_raw + sum((CTC_COEFF[i] * (Tcr ** (i + 1)) - CTC_COEFF[i] * (T_c ** (i + 1)))
                    for i in range(len(CTC_COEFF)))
    Cc1 = C_raw + sum((CTC1_COEFF[i] * (Tcr ** (i + 1)) - CTC1_COEFF[i] * (T_c ** (i + 1)))
                    for i in range(len(CTC1_COEFF)))
    A = (Cc1 - Cc0) / (H - L)
    B = Cc0 - A * L
    Cc = B + A * C_raw
    return poly_eval(Cond_COEFF, Cc)

#############################################################################################
################################## Main Functions ###########################################
#############################################################################################

def send_command(ser, command):
    ser.write(bytes([command]))
    echo = ser.read(1)
    if not echo or echo[0] != command:
        return None
    ser.write(bytes([ACK]))
    time.sleep(0.25)
    return ser.read_all()

def init_sensor(ser):
    print("Testing connection...")
    ser.write(bytes([COM_TEST]))
    response = ser.read(2)
    if response != bytes([COM_TEST, ACK]):
        print("Test failed. Response:", response)
        return

    print("Setting to PC mode...")
    ser.write(bytes([COM_PCMODE]))
    response = ser.read(2)
    if response != bytes([COM_PCMODE, 0x02]):
        print("Failed to enter PC-mode.")
        return

def poll_measurements(ser):
    ser.write(bytes([COM_ONLINE_MEAS]))
    if ser.read(1) == bytes([COM_ONLINE_MEAS]):
        ser.write(bytes([ACK]))
        data = ser.read(6)
        if len(data) == 6:
            T_raw = data[0] + 256 * data[1]
            P_raw = data[2] + 256 * data[3]
            C_raw = data[4] + 256 * data[5]
            print(f"Raw values: T={T_raw}, P={P_raw}, C={C_raw}")

            T_c = temp_C(T_raw)
            P_bar = pressure_bar(P_raw, T_c)
            D_m = depth_m(P_bar)
            C_mScm = conductivity_mScm(C_raw, T_c)
            print(f"Converted: Temp={T_c:.3f} °C, Pressure={P_bar:.3f} bar, Depth={D_m:.2f} m, Conductivity={C_mScm:.3f} mS/cm\n")
    time.sleep(1)

def main(port):
    with serial.Serial(port, 4800, timeout=TIMEOUT) as ser:
        init_sensor(ser)

        print("Polling measurements...")
        while True:
            poll_measurements(ser)

if __name__ == "__main__":
    main(CTD_port)
