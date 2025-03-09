import numpy as np
from ezauv.hardware.sensor_interface import DepthInterface, ImuInterface

# a class to provide fake sensor data for the simulation

class FakeDepthSensor(DepthInterface):
        # TODO currently not working due to 2D environment
        def __init__(self, deviation):
            self.deviation = deviation

        def get_depth(self):
            return 0.
        
        def initialize(self):
            pass

        def overview(self) -> str:
            return f"Simulated Depth Sensor -- Standard deviation: {self.deviation}. Currently hovercraft-only, no depth."
        
class FakeIMU(ImuInterface):

    def __init__(self, max_dev_accel, acceleration_function, rotation_function):
        self.max_dev_accel = max_dev_accel
        self.acceleration_function = acceleration_function
        self.rotation_function = rotation_function

    def get_accelerations(self):
        acceleration = self.acceleration_function() + (np.random.rand(2) - 0.5) * 2 * self.max_dev_accel
        return np.append(acceleration, 0) # bc it expects a 3d acceleration
    
    def get_rotation(self):
        return self.rotation_function()

    def initialize(self):
        pass

    def overview(self) -> str:
        return f"Simulated IMU -- Standard deviation: {self.max_dev_accel}"