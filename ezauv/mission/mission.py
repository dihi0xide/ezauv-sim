from abc import ABC, abstractmethod
from typing import Tuple
import numpy as np

class Task(ABC):

    @property
    @abstractmethod
    def name(self) -> str:
        pass

    @property
    @abstractmethod
    def finished(self) -> bool:
        pass

    @abstractmethod
    def update(self, sensors) -> np.ndarray:
        pass

class Subtask(ABC):
    @property
    @abstractmethod
    def name(self) -> str:
        pass

    @abstractmethod
    def update(self, sensors, wanted_speed: np.ndarray) -> np.ndarray:
        pass


class Path:
    def __init__(self, *args: Task):
        self.path: Tuple[Task, ...] = args
