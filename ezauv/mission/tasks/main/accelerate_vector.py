from ezauv.mission.mission import Task

import numpy as np
import time

class AccelerateVector(Task):

    def __init__(self, acceleration_state, length):
        super().__init__()
        self.acceleration_state = acceleration_state
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

    def update(self, sensor_data: dict) -> np.ndarray:
        if(self.start == -1):
            self.start = time.time()
        return self.acceleration_state
        