from dataclasses import dataclass
from dataclasses_json import dataclass_json

@dataclass_json
@dataclass
class JoystickState:
    lx: float = 0.0  # Left joystick X
    ly: float = 0.0  # Left joystick Y
    rx: float = 0.0  # Right joystick X
    ry: float = 0.0  # Right joystick Y
    enabled: bool = False