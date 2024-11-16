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
- **Inputs**: [What messages or topics this module subscribes to.]
- **Outputs**: [What messages or topics this module publishes.]
- **Interconnections**: [How this module communicates with other modules/packages.]

---

## Development Notes  
- [List any important implementation notes, standards, or practices.]
- [Highlight cross-development considerations, dependencies, or constraints.]

---

## Testing  
- [Outline testing strategies for this module.]
- [Specify any integration tests with other modules.]

---

## References  
[List any relevant documentation, papers, or resources.]
