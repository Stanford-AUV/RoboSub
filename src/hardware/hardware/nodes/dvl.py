import dvl.dvl
import time
import logging


def main():
    # Configure logging
    logging.basicConfig(
        filename="dvl_data.log", level=logging.INFO, format="%(asctime)s - %(message)s"
    )

    # Initialize the DVL object with the appropriate COM port and baud rate
    dvl = dvl.dvl.Dvl(com="/dev/ttyUSB0", baudrate=9600)

    # Connect to the Wayfinder DVL
    if dvl.connect(dvl.com, dvl.baudrate):
        logging.info("Successfully connected to the Wayfinder DVL.")
    else:
        logging.error("Failed to connect to the Wayfinder DVL.")
        return

    # Retrieve system information
    if dvl.get_system():
        system_info = dvl.system_info
        logging.info("System Information:")
        logging.info(f"  Serial Number: {system_info.serial_number}")
        logging.info(f"  Firmware Version: {system_info.firmware_version}")
        logging.info(f"  Hardware Version: {system_info.hardware_version}")
    else:
        logging.error("Failed to retrieve system information.")

    # Retrieve system configurable settings
    if dvl.get_setup():
        system_setup = dvl.system_setup
        logging.info("System Configurable Settings:")
        logging.info(f"  Speed of Sound: {system_setup.speed_of_sound} m/s")
        logging.info(f"  Maximum Depth: {system_setup.max_depth} meters")
        logging.info(f"  Ping Rate: {system_setup.ping_rate} Hz")
    else:
        logging.error("Failed to retrieve system configurable settings.")

    # Perform built-in tests
    if dvl.run_tests():
        test_results = dvl.test_results
        logging.info("Built-In Test Results:")
        for test, result in test_results.items():
            logging.info(f"  {test}: {'Passed' if result else 'Failed'}")
    else:
        logging.error("Failed to run built-in tests.")

    # Collect and log velocity data
    try:
        logging.info("Starting data collection...")
        start_time = time.time()
        duration = 60  # Collect data for 60 seconds
        while time.time() - start_time < duration:
            if dvl.get_velocity():
                velocity_data = dvl.velocity_data
                logging.info("Velocity Data:")
                logging.info(f"  Time: {velocity_data.time}")
                logging.info(f"  Velocity X: {velocity_data.vel_x} m/s")
                logging.info(f"  Velocity Y: {velocity_data.vel_y} m/s")
                logging.info(f"  Velocity Z: {velocity_data.vel_z} m/s")
                logging.info(f"  Figure of Merit: {velocity_data.fom}")
                logging.info(f"  Vertical Distance: {velocity_data.vdist} meters")
            else:
                logging.warning("Failed to retrieve velocity data.")
            time.sleep(1)  # Adjust the sleep interval as needed
    except KeyboardInterrupt:
        logging.info("Data collection interrupted by user.")
    finally:
        # Disconnect from the Wayfinder DVL
        dvl.disconnect()
        logging.info("Disconnected from the Wayfinder DVL.")


if __name__ == "__main__":
    main()
