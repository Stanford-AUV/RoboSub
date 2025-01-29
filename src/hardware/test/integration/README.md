# Generic Integration Script
## Installation and Configuration
These scripts are not installed with the WayFinder Python Driver.

1.  Ensure Python 3.6 or later is installed on the host computer
2.  Ensure WayFinder Python Driver is installed
3.  Modify the global parameters in `blueROVIntegration.py` to fit the vehicle
4.  Modify `reportVelocities` to report the velocities and confidence to the
host vehicle.
5.  Modify `host_setup` to configure the interface to the host vehicle.
6.  Optionally, modify `log_setup` to log additional data.
7.  Optionally, modify `sensor_setup` to configure the WayFinder differently.


Example parameters:
```
wayFinderPort = "/dev/ttyUSB0"

roll = np.radians(180)
pitch = np.radians(0)
yaw = np.radians(135)
```

## Supported Out Of Box Integrations
* BlueROV 2 and BlueROV Heavy