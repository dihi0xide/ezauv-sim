from ezauv.mission.mission import Subtask
from ezauv.hardware.sensor_interface import SensorInterface
from ezauv.utils.pid import PID

import numpy as np
import time
import quaternion



class HeadingPID(Subtask):

    def __init__(self, wanted_heading, Kp, Ki, Kd):
        super().__init__()
        self.pid = PID(Kp, Ki, Kd, 0)
        self.wanted = wanted_heading

    @property
    def name(self) -> str:
        return "Heading PID subtask"

    def update(self, sensors: SensorInterface, wanted_speed: np.ndarray) -> np.ndarray:
        q = sensors.imu.get_rotation()
        v_quat = q * np.quaternion(0, 0.6, 0.0, 0.1) * q.conjugate()
        # v
        v_prime = np.array([v_quat.x, v_quat.y, v_quat.z])

        # print(v_prime)
        yaw = np.degrees(np.arctan2(v_prime[1], v_prime[0]))

        diff = self.wanted - yaw
        # print(yaw)
        
        if abs(diff) >= 180:
            sign = yaw / abs(yaw)
            abs_diff_yaw = 180 - abs(yaw)
            abs_diff_target = 180 - abs(self.wanted)
            diff = sign * (abs_diff_yaw + abs_diff_target)

        signal = self.pid.signal(-diff)
        # print(quaternion.as_euler_angles(sensors.imu.get_rotation()))
        return np.array([0., 0., 0., 0., 0., signal])