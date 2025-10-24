from dataclasses import dataclass
from dataclasses_json import dataclass_json


@dataclass_json
@dataclass
class KeyboardState:

    moveLeft: bool = False
    moveRight: bool = False
    moveForward: bool = False
    moveBackward: bool = False
    moveUp: bool = False
    moveDown: bool = False

    turnCCW: bool = False
    turnCW: bool = False

    increaseVel: bool = False
    decreaseVel: bool = False

    increaseLinVel: bool = False
    decreaseLinVel: bool = False

    increaseAngVel: bool = False
    decreaseAngVel: bool = False
