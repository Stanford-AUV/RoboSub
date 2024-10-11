"""
Controller Node.

Authors:
    Ali Ahmad
    Khaled Messai
"""
import threading
from msgs.msg import 

class Controller(Node):

    def __init__(self, kP, kI, kD, ceil, start_i):
        super().__init__('controller')
        self.lock = threading.Lock()
        self.state_subscription = self.create_subscription(State, "state", self.state_callback, 10)


    def state_callback(self):
        pass

    def reference_callback(self):
        pass

    def update(self):
        pass

    def reset(self):
        pass
