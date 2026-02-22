"""Example usage for DVL driver
"""
from dvl.dvl import Dvl
from dvl.system import OutputData

def update_data(output_data: OutputData, obj):
    """Prints data time to screen
    """
    del obj
    if output_data is not None:
        time = output_data.get_date_time()
        txt = time.strftime('%Y-%m-%d %H:%M:%S.%f')[:-3]
        print("Got data {0}".format(txt))

if __name__ == "__main__":
    PORT = input("Please enter your port number (e.g. '1' for COM1) =  ")
    PORT = "COM" + PORT

    # Connect to serial port
    with Dvl(PORT, 115200) as DVL:

        if DVL.is_connected():

            # Get user system setup
            if DVL.get_setup():
                # Print setup 
                print (DVL.system_setup)

            # Stop pinging
            if not DVL.enter_command_mode():
                print("Failed to stop pinging")

            # Reset to factory defaults (requires Wayfinder to be in 'command mode')
            if not DVL.reset_to_defaults():
                print("Failed to reset to factory defaults")

            # Collect data - make sure working folder exists
            if not DVL.start_logging("c:/temp", "DVL"):
                print("Failed to start logging")
            else:
                print("Data logged to {0}".format(DVL.get_log_file_name()))

            # Register callback function
            DVL.register_ondata_callback(update_data)

            # Start pinging
            if not DVL.exit_command_mode():
                print("Failed to start pinging")

            # Blocking call to wait for key pressed to end program
            KEY = input("Press Enter to stop\n")

        else:
            print("Failed to open {0} - make sure it is not used by any other program".format(PORT))

        # Unregister
        DVL.unregister_all_callbacks()
