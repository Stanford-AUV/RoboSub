from dataclasses import dataclass, field
from typing import List
from dataclasses_json import dataclass_json
import json


@dataclass_json
@dataclass
# condense real life joystick controller read into data
class JoystickState:

    ##################################################################
    #              TODO: Define variables                            #
    ##################################################################

    axes: List[float] = field(default_factory=list)
    buttons: List[int] = field(default_factory=list)

    def to_json(self):
        return json.dumps(self.__dict__)
