from dataclasses import dataclass

from dataclasses_json import dataclass_json


@dataclass_json
@dataclass
class JoystickState:
    """One joystick read, as sent over NATS by joystick_local.py."""

    lx: float = 0.0  # left stick X
    ly: float = 0.0  # left stick Y
    rx: float = 0.0  # right stick X
    ry: float = 0.0  # right stick Y
    light_power: float = 0.0
    enabled: bool = False
