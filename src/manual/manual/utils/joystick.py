from dataclasses import dataclass, field
from typing import List
from dataclasses_json import dataclass_json
import json


@dataclass_json
@dataclass
# condense real life joystick controller read into data
class JoystickState:
    lx: float = 0.0  # Left joystick X
    ly: float = 0.0  # Left joystick Y
    rx: float = 0.0  # Right joystick X
    ry: float = 0.0  # Right joystick Y
    light_power: float = 0.0
    enabled: bool = False
