import numpy as np 
import json 

class PID_config():
    "Class to encapsulate PID parameters"
    "Kp - proportional gain"
    "Kd - derivative gain"
    "Ki - integral gain"
    "Three sets of gains for position, velocity, and orientation"
    "Max signals : upper limits for the control signal"
    def __init__(self, 
                kP_position: np.ndarray,
                kD_position: np.ndarray,
                kI_position: np.ndarray,
                kP_velocity: np.ndarray,
                kD_velocity: np.ndarray,
                kI_velocity: np.ndarray,
                kP_orientation: np.ndarray,
                kD_orientation: np.ndarray,
                kI_orientation: np.ndarray,
                kP_angular_velocity: np.ndarray,
                kD_angular_velocity: np.ndarray,
                kI_angular_velocity: np.ndarray,
                max_signal_position: np.ndarray,
                max_signal_velocity: np.ndarray,
                max_signal_orientation: np.ndarray,
                max_signal_angular_velocity: np.ndarray,
                max_integral_position: np.ndarray,
                max_integral_velocity: np.ndarray,
                max_integral_orientation: np.ndarray,
                max_integral_angular_velocity: np.ndarray):

        self.kP_position = kP_position
        self.kD_position = kD_position
        self.kI_position = kI_position

        self.kP_velocity = kP_velocity
        self.kD_velocity = kD_velocity
        self.kI_velocity = kI_velocity

        self.kP_orientation = kP_orientation
        self.kD_orientation = kD_orientation
        self.kI_orientation = kI_orientation

        self.kP_angular_velocity = kP_angular_velocity
        self.kD_angular_velocity = kD_angular_velocity
        self.kI_angular_velocity = kI_angular_velocity

        self.integral_position = np.zeros(3)
        self.integral_velocity = np.zeros(3)
        self.integral_orientation = np.zeros(3)
        self.integral_angular_velocity = np.zeros(3)

        self.max_signal_position = max_signal_position
        self.max_signal_orientation = max_signal_orientation
        self.max_signal_velocity = max_signal_velocity
        self.max_signal_angular_velocity = max_signal_angular_velocity

        self.max_integral_position = max_integral_position
        self.max_integral_orientation = max_integral_orientation
        self.max_integral_velocity = max_integral_velocity
        self.max_integral_angular_velocity = max_integral_angular_velocity



    def update_PID_params(self, gain_type: str, new_values: np.ndarray, new_limits: float):
        if hasattr(self, gain_type):
            setattr(self, gain_type, new_values)
        else:
            raise ValueError("invalid gain type :(")
        
    @staticmethod
    def from_json(json_file_path):
        #lets do this as a config file. 
        with open(json_file_path, 'r') as file:
            data = json.load(file)
        
        return PID_config(  
            kP_position=np.array(data["position"]["kP"]),
            kD_position=np.array(data["position"]["kD"]),
            kI_position=np.array(data["position"]["kI"]),

            kP_velocity=np.array(data["velocity"]["kP"]),
            kD_velocity=np.array(data["velocity"]["kD"]),
            kI_velocity=np.array(data["velocity"]["kI"]),

            kP_orientation=np.array(data["orientation"]["kP"]),
            kD_orientation=np.array(data["orientation"]["kD"]),
            kI_orientation=np.array(data["orientation"]["kI"]),

            kP_angular_velocity=np.array(data["angular_velocity"]["kP"]),
            kD_angular_velocity=np.array(data["angular_velocity"]["kD"]),
            kI_angular_velocity=np.array(data["angular_velocity"]["kI"]),

            max_signal_position=np.array(data["position"]["max_signal"]),
            max_signal_velocity=np.array(data["velocity"]["max_signal"]),
            max_signal_orientation=np.array(data["orientation"]["max_signal"]),
            max_signal_angular_velocity=np.array(data["angular_velocity"]["max_signal"]),

            max_integral_position=np.array(data["position"]["max_integral"]),
            max_integral_velocity=np.array(data["velocity"]["max_integral"]),
            max_integral_orientation=np.array(data["orientation"]["max_integral"]),
            max_integral_angular_velocity=np.array(data["angular_velocity"]["max_integral"])
        )
    

    #getters and setters with clamping
    @property
    def integral_position(self):
        return self._integral_position

    @integral_position.setter
    def integral_position(self, value):
        self._integral_position = np.clip(value, -self.max_integral_position, self.max_integral_position)

    @property
    def integral_velocity(self):
        return self._integral_velocity

    @integral_velocity.setter
    def integral_velocity(self, value):
        self._integral_velocity = np.clip(value, -self.max_integral_velocity, self.max_integral_velocity)

    @property
    def integral_orientation(self):
        return self._integral_orientation

    @integral_orientation.setter
    def integral_orientation(self, value):
        self._integral_orientation = np.clip(value, -self.max_integral_orientation, self.max_integral_orientation)

    @property
    def integral_angular_velocity(self):
        return self._integral_angular_velocity

    @integral_angular_velocity.setter
    def integral_angular_velocity(self, value):
        self._integral_angular_velocity = np.clip(value, -self.max_integral_angular_velocity, self.max_integral_angular_velocity)