# Control

**Authors and Maintainers**: 
- Ali Ahmad - aliahmad@stanford.edu
- Khaled Messai - kmessai@stanford.edu
- Iris Xu - izxu28@stanford.edu
- Deren Ji - derenji@stanford.edu

[//]: # (List maintainers and contact info)

---

## Purpose  
The goal of the control package is to closely follow a given reference trajectory given our current state.  

[//]: # (Briefly explain the purpose of this module and its role in the overall system.)

---

## Nodes  
- **Thrust Generator**: Calculate the thrust magnitude and direction for each thruster using a given wrench vector.
- **Controller**: Calculate a wrench vector given a current state and desired reference trajectory with minimal error.
- **Test Path**: Node used to test the package in simulation.

---

## Communication  
- **Inputs**: `'/odometry'`: Odometry; `'/path'`: GeneratedPath; `'/test_path'`: GeneratedPath, Odometry; `'/wrench'`: WrenchStamped
- **Outputs**: `'/wrench'`: WrenchStamped, `'/thrusts'`: ThrustsStamped
- **Interconnections**: This package receives a current state from our localization process and a reference trajectory from our planning process. We use this information to calculate our thrust and send out the individual thrust for each thruster. 

---

## Development Notes  
- The goal of the whole control package is to be modular and abstract. The controller logic should work for any type of control policy and path following algorithm. For each new algorithm implemented, they should all have the same functions that set and return data. Be sure to adhere to common Python best practices for style.
- [Highlight cross-development considerations, dependencies, or constraints.]

---

## Testing  
- To run unit tests, run `./test.sh`. 
- To run tests in simulation, follow the instructions [here](control/README.md)

---

## References  
- [Controls Engineering in the FIRST Robotics Competition](https://file.tavsys.net/control/controls-engineering-in-frc.pdf)
- [Probabilistic Robotics](https://github.com/yvonshong/Probabilistic-Robotics/blob/master/Probabilistic-Robotics-en.pdf)
