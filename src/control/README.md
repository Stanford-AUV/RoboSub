## Testing the controller in simulation
To test the controller in simulation, you need to have three terminal windows open: 

1. In the first window, run `./sim.sh`. This opens Gazebo and restarts the simulation.
2. In the second window, run `ros2 launch src/launch/simulation.py`. This runs the simulation package which gives us our current state information.
3. In the third window, run `ros2 launch src/launch/control.py`. This runs the control package and will control the submarine.

Run the commands in the order described above to properly test in simulation. 

## Running unit tests
To run unit tests, simply run `./tests.sh` in your terminal.

## Writing unit tests
Each feature's tests should exist in their own file in `control/test`. The naming convention for unit test files is `test_[feature name].py`. 
Each test function should test one thing and follow a consistent style. 

## Writing node tests
[WIP]
