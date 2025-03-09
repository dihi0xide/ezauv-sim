from abc import ABC, abstractmethod
from typing import Tuple
import numpy as np

class Task(ABC):

    @property
    @abstractmethod
    def name(self) -> str:
        """The name of the task."""
        pass

    @property
    @abstractmethod
    def finished(self) -> bool:
        """Whether the task has completed."""
        pass

    @abstractmethod
    def update(self, sensors) -> np.ndarray:
        """Update based on sensor data, should return a numpy array."""
        pass

class Subtask(ABC):
    @property
    @abstractmethod
    def name(self) -> str:
        """The name of the subtask."""
        pass

    @abstractmethod
    def update(self, sensors) -> np.ndarray:
        """Update direction based on sensors. Does not directly set the direction, only adds to it."""
        pass


class Path:
    def __init__(self, *args: Task):
        self.path: Tuple[Task, ...] = args
