import numpy as np
import quaternion

from ezauv.auv import AUV
from ezauv.hardware import MotorController, Motor, SensorInterface
from ezauv.utils.inertia import InertiaBuilder, Cuboid
from ezauv.mission.tasks.main import AccelerateVector, RunFunction
from ezauv.mission.tasks.subtasks import HeadingPID, Simulate
from ezauv.mission import Path


from ezauv.simulation.core import Simulation
motor_locations = [
    np.array([-1., -1., 0.]),
    np.array([-1., 1., 0.]),
    np.array([1., 1., 0.]),
    np.array([1., -1., 0.])
    ]

motor_directions = [
    np.array([1., -1., 0.]),
    np.array([1., 1., 0.]),
    np.array([1., -1., 0.]),
    np.array([1., 1., 0.])
    ]





bounds = [[-1, 1]] * 4
deadzone = [[-0.1, 0.1]] * 4
sim = Simulation(motor_locations, motor_directions, 1/6, bounds, deadzone)

sim_anchovy = AUV(
    motor_controller = MotorController(
        inertia = InertiaBuilder(
            Cuboid(
                mass=1,
                width=1,
                height=1,
                depth=0.1,
                center=np.array([0,0,0])
            )).moment_of_inertia(),
            motors = [
                Motor(
                    direction,
                    loc,
                    sim.set_motor(i),
                    lambda: 0,
                    Motor.Range(bounds[i][0], bounds[i][1]),
                    Motor.Range(-deadzone[i][0], deadzone[i][1])
                    )
                for i, (loc, direction) in enumerate(zip(motor_locations, motor_directions))
                ]
        ),
        sensors = SensorInterface(imu=sim.imu(0.05), depth=sim.depth(0.)),
        pin_kill = lambda: None
    )

sim_anchovy.register_subtask(Simulate(sim))
sim_anchovy.register_subtask(HeadingPID(0, 0.03, 0.0, 0.01))

# print()

mission = Path(
    AccelerateVector(np.array([0., 0., 0., 0., 0., 0.]), 20),
    RunFunction(lambda: sim.apply_force(thrust=np.array([0.,0.]), rotation=10)),
    AccelerateVector(np.array([0., 0., 0., 0., 0., 0.]), 20),
    # AccelerateVector(np.array([-0.5, -0.5, 0., 0., 0., 0.]), 3),
    # AccelerateVector(np.array([0., 0., 0., 0., 0., 1.]), 5)
)

sim_anchovy.travel_path(mission)

sim.render()
# print((sum(test) / len(test)) * 1000)