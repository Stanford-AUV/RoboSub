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

    def setState(self, action: str, value: bool):
        """
        Sets one of the keyboard state flags by name.

        Args:
            action (str): The name of the field (e.g. 'moveLeft', 'turnCW')
            value (bool): The state to set (True for pressed, False for released)
        """
        if not hasattr(self, action):
            raise AttributeError(f"Invalid action: {action}")
        setattr(self, action, value)
