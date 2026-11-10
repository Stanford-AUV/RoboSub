from dataclasses import dataclass
from dataclasses_json import dataclass_json


@dataclass_json
@dataclass
class KeyboardState:
    """Which teleop keys are held, as sent over NATS by keyboard_local.py."""

    move_left: bool = False
    move_right: bool = False
    move_forward: bool = False
    move_backward: bool = False
    move_up: bool = False
    move_down: bool = False

    turn_ccw: bool = False
    turn_cw: bool = False

    increase_vel: bool = False
    decrease_vel: bool = False

    increase_lin_vel: bool = False
    decrease_lin_vel: bool = False

    increase_ang_vel: bool = False
    decrease_ang_vel: bool = False

    def set_state(self, action: str, value: bool):
        """Set one flag by field name (e.g. "move_left", "turn_cw")."""
        if not hasattr(self, action):
            raise AttributeError(f"Invalid action: {action}")
        setattr(self, action, value)
