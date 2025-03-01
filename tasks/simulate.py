from ..mission import Subtask
from ..sensor_interface import SensorInterface
from ..simulation.simulation import Simulation
from ..simulation.simulation_animator import set_text

import numpy as np
import time

class Simulate(Subtask):

    def __init__(self, simulation: Simulation):
        super().__init__()
        self.simulation = simulation
        self.prevtime = -1.


    @property
    def name(self) -> str:
        return "Simulate subtask"

    def update(self, sensors: SensorInterface, wanted_speed: np.ndarray) -> np.ndarray:
        new_time = time.time()
        if(self.prevtime != -1):
            # set_text(str(new_time - self.prevtime))
            self.simulation.simulate(new_time - self.prevtime)
        self.prevtime = time.time()
        return np.array([0., 0., 0., 0., 0., 0.])
        