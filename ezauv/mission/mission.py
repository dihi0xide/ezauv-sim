from abc import ABC, abstractmethod
from typing import Tuple
import numpy as np

from ezauv import AccelerationState

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
    def update(self, sensors) -> AccelerationState:
        """Update based on sensor data."""
        pass

class Subtask(ABC):
    @property
    @abstractmethod
    def name(self) -> str:
        """The name of the subtask."""
        pass

    @abstractmethod
    def update(self, sensors) -> AccelerationState:
        """Update direction based on sensors. Does not directly set the direction, only adds to it."""
        pass


class Path:
    def __init__(self, *args: Task):
        self.path: Tuple[Task, ...] = args
