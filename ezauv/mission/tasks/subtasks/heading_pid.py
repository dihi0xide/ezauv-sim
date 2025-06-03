from ezauv.mission.mission import Subtask
from ezauv.hardware.sensor_interface import SensorInterface
from ezauv.utils.pid import PID
from ezauv import AccelerationState

import numpy as np
import quaternion



class HeadingPID(Subtask):

    def __init__(self, wanted_heading, Kp, Ki, Kd):
        super().__init__()
        self.pid = PID(Kp, Ki, Kd, 0)
        self.wanted = wanted_heading

    @property
    def name(self) -> str:
        return "Heading PID subtask"

    def update(self, sensors: SensorInterface) -> np.ndarray:
        q = sensors.imu.get_rotation()
                
        yaw = np.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y**2 + q.z**2))
        diff = self.wanted - yaw
        
        if abs(diff) >= 180:
            sign = yaw / abs(yaw)
            abs_diff_yaw = 180 - abs(yaw)
            abs_diff_target = 180 - abs(self.wanted)
            diff = sign * (abs_diff_yaw + abs_diff_target)

        signal = self.pid.signal(-diff)
        return AccelerationState(Rz=signal)