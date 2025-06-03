import numpy as np
import traceback
import copy
from typing import Callable, List

from ezauv.hardware.motor_controller import MotorController
from ezauv.hardware.sensor_interface import SensorInterface
from ezauv.mission.mission import Path, Subtask
from ezauv.utils.logger import Logger, LogLevel

class AUV:
    def __init__(self, *,
                 motor_controller: MotorController,     # the object to control the motors with
                 sensors: SensorInterface,              # the interface for sensor data
                 pin_kill: Callable = lambda: None,     # an emergency kill function; should disable all motors via pins

                 logging: bool = False,                 # whether to save log to file
                 console: bool = True,                  # whether to print log to console
                 ):
        """
        Create a sub wrapper object.\n
        motor_controller: the object to control the motors with\n
        sensors: the interface for all sensor data\n
        pin_kill: an emergency kill function, when the library is having issues. Should manually set motors off\n
        logging: whether to save log to file\n
        console: whether to print log to console
        """
        self.motor_controller = motor_controller
        self.sensors = sensors
        self.pin_kill = pin_kill

        self.logger = Logger(console, logging)

        self.motor_controller.log = self.logger.create_sourced_logger("MOTOR")
        self.sensors.log = self.logger.create_sourced_logger("SENSOR")
        self.sensors.depth.log = self.logger.create_sourced_logger("DEPTH")
        self.sensors.imu.log = self.logger.create_sourced_logger("IMU")

        self.logger.log("Sub enabled")
        self.motor_controller.overview()
        self.sensors.overview()

        self.motor_controller.initialize()
        self.sensors.initialize()

        self.subtasks: List[Subtask] = []

    def register_subtask(self, subtask):
        """Register a subtask to be run every iteration for this AUV."""
        self.subtasks.append(subtask)

    def kill(self):
        """Set all motors to 0 speed, killing the sub."""
        self.motor_controller.set_motors(np.array([0 for _ in self.motor_controller.motors]))

    def travel_path(self, mission: Path) -> None:
        """Execute each Task in the given Path, in order, then kill the sub. Handles errors."""

        self.logger.log("Beginning path")

        try:
            for task in mission.path:
                self.logger.log(f"Beginning task {task.name}")
                while(not task.finished):
                    wanted_direction = copy.deepcopy(task.update(self.sensors))
                    
                    for subtask in self.subtasks:
                        wanted_direction += subtask.update(self.sensors)

                    solved_motors = self.motor_controller.solve(
                        wanted_direction,
                        self.sensors.imu.get_rotation()
                    )

                    if(solved_motors[0]):
                        self.motor_controller.set_motors(solved_motors[1])
                    
        except:
            self.logger.log(traceback.format_exc(), level=LogLevel.ERROR)
    
        finally:
            self.logger.log("Killing sub")


            if(not self.motor_controller.killed()):
                kill_methods = [
                ("kill", self.kill),
                # kill through sub interface, uses full library to send kill. should always work

                ("backup kill", self.pin_kill)
                # last resort, directly control pins and send kill commands. doesn't go through library
                # at all, just sends pin commands
                ]
                # when we get more kills (eg hardware kill once we connect it to raspi) add them here
            
                for method_name, method in kill_methods:
                    self.logger.log(f"Attempting {method_name}...")
                    method()
                    if self.motor_controller.killed():
                        self.logger.log(f"{method_name.capitalize()} succeeded")
                        break
                    else:
                        self.logger.log(f"{method_name.capitalize()} failed", level=LogLevel.ERROR)
                else:
                    self.logger.log("All kills ineffective. Manual intervention required", level=LogLevel.ERROR)
            
            self.logger.end()