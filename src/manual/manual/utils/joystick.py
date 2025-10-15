from dataclasses import dataclass
from dataclasses_json import dataclass_json

@dataclass_json
@dataclass
class JoystickState:

##################################################################
#              TODO: Define variables                            #
##################################################################

    # example_variable: type = default_value
    joy: float = 0.0
    stick: str = "stick"