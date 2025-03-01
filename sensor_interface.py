import numpy as np
from abc import ABC, abstractmethod
import quaternion
import time


# The abstract interface for interacting with the hardware; this should be
# extended and registered into the auv

class ImuInterface(ABC):

    def __init__(self):
        super().__init__()
        self.log = lambda str, level=None: print(str)

    @abstractmethod
    def get_accelerations(self) -> np.ndarray:
        pass

    @abstractmethod
    def get_rotation(self) -> np.quaternion:
        pass

    @abstractmethod
    def initialize(self) -> None:
        pass

    @abstractmethod
    def overview(self) -> None:
        pass

class DepthInterface(ABC):

    def __init__(self):
        super().__init__()
        self.log = lambda str: print(str)
    
    @abstractmethod
    def get_depth(self) -> float:
        pass

    @abstractmethod
    def initialize(self) -> None:
        pass

    @abstractmethod
    def overview(self) -> str:
        pass

class SensorInterface:

    def __init__(self, *, imu: ImuInterface, depth: DepthInterface):
        self.imu: ImuInterface = imu
        self.depth: DepthInterface = depth
        self.log = lambda str: print(str)

    def initialize(self) -> None:
        self.imu.initialize()
        self.depth.initialize()

    def overview(self) -> None:
        self.imu.overview()
        self.depth.overview()

    def killed(self, average_time=1, max_dev=0.01, timestep=0.001) -> bool:
        avg = 0
        total = int(np.ceil(average_time / timestep))

        for i in range(total):
            avg += self.imu.get_accelerations()
            time.sleep(timestep)

        avg /= total
        return np.all(abs(avg) <= max_dev)
