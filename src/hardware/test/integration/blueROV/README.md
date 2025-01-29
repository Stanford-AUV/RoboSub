# BlueROV Integration
## Installation and Configuration
These scripts are not installed with the WayFinder Python Driver.

Integration Specific Requirements:
 * pymavlink

1.  Ensure ArduSub 4.1.0 or later is installed on the PixHawk
2.  Update the parameters in `bluerov.params` on the PixHawk
3.  Log into the Companion Pi via SSH
4.  Ensure Python 3.6 is installed
5.  Ensure WayFinder Python Driver is installed
6.  Ensure Integration Requirements are installed
7.  Add `tcp:127.0.0.1:14777` to `~/mavproxy.params`
8.  Restart the Companion Pi
9.  Log into the Companion Pi via SSH
10. Modify the global parameters in `blueROVIntegration.py` to fit the vehicle

Example:
```
wayFinderPort = "/dev/ttyUSB0"

roll = np.radians(180)
pitch = np.radians(0)
yaw = np.radians(135)
```

## Running the Integration
To run the configured integration script, simply execute the 
`blueROVIntegration.py` script on the Companion Pi.