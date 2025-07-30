from ezauv.mission.mission import Task
from ezauv import AccelerationState

import numpy as np

class RunFunction(Task):

    def __init__(self, func):
        super().__init__()
        self.func = func
        self.run = False

    @property
    def name(self) -> str:
        return "Run function task"
    
    @property
    def finished(self) -> bool:
        return self.run

    def update(self, sensors: dict) -> np.ndarray:
        self.func()
        self.run = True
        return AccelerationState()
