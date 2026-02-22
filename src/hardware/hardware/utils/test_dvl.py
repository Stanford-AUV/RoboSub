import serial
import time

# Configuration
PORT = "/dev/ttyUSB0"  # Change this to match your setup
BAUD_RATE = 115200


def test_dvl_connection():
    try:
        with serial.Serial(PORT, BAUD_RATE, timeout=2) as ser:
            print(f"Connected to DVL on {PORT} at {BAUD_RATE} baud")

            # Example command to request a measurement (adjust based on DVL protocol)
            test_command = b"$VNWRG,07,100*XX\r\n"
            ser.write(test_command)
            print(f"Sent: {test_command.decode(errors='ignore')}")

            # Read response
            time.sleep(1)  # Give time for response
            response = ser.read(ser.in_waiting or 1).decode(errors="ignore")
            print(f"Received: {response}")

            if response:
                print("DVL connection test successful.")
            else:
                print("No response from DVL. Check connections and settings.")

    except serial.SerialException as e:
        print(f"Error: {e}")


if __name__ == "__main__":
    test_dvl_connection()
