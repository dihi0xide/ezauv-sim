from ezauv.mission.mission import Task
from ezauv.hardware.sensor_interface import SensorInterface

import numpy as np
import time

class AccelerateVector(Task):

    def __init__(self, vec, length):
        super().__init__()
        self.vec = vec
        self.length = length
        self.start = -1

    @property
    def name(self) -> str:
        return "Accelerate at vector task"
    
    @property
    def finished(self) -> bool:
        if(self.start == -1):
            self.start = time.time()
        return (time.time() - self.start) >= self.length

    def update(self, sensors: SensorInterface) -> np.ndarray:
        if(self.start == -1):
            self.start = time.time()
        return self.vec
        