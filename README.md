# ezauv
**ezauv is a Python library to make AUVs, or autonomous underwater vehicles, easy.**
This library was created for BVR AUV, Beaver Country Day School's RoboSub team.

## Why?
In the yearly RoboSub competition, teams build autonomous submarines. Much of the code of these submarine can be abstracted into a few basic concepts: tasks, like travelling a vector or circling a buoy; subtasks, like holding heading or depth; and sensors, such as IMUs and depth sensors. ezauv provides an easy interface for all of these pieces of code to simplify their creation. More importantly, ezauv solves the motors needed to travel a specific vector and rotation under the hood, regardless of the thrust vectors or locations of the motors and the sub's overall dimensions. This allows the code for one hardware surface to be ported to another with almost no change to the code.

ezauv also provides interfaces for logging data to file, building the inertia tensor of your sub from a set of geometries, and some simple tasks and subtasks such as accelerating in a direction or using a PID on rotation.

## Installation

 - Make sure pip is installed and working
 - Check your Python version; ezauv is built for Python 3.11, but will likely work on most versions >3
 - Run the following command with pip:
```
pip install ezauv
```

For developer information, [read the wiki](https://github.com/beaver-auv/ezauv/wiki).
## Example Program
This example creates a simulation with a square hovercraft, moves it forward, moves it backwards, and then spins it.
```python
import numpy as np
from ezauv.auv import AUV
from ezauv.hardware import MotorController, Motor, SensorInterface
from ezauv.utils.inertia import InertiaBuilder, Cuboid
from ezauv.mission.tasks.main import AccelerateVector
from ezauv.mission.tasks.subtasks import HeadingPID, Simulate
from ezauv.mission import Path
from ezauv.simulation.core import Simulation

motor_locations = [
	np.array([-1., -1., 0.]),
	np.array([-1., 1., 0.]),
	np.array([1., 1., 0.]),
	np.array([1., -1., 0.])
] # first, we'll write down the locations of the motors

motor_directions = [
	np.array([1., -1., 0.]),
	np.array([1., 1., 0.]),
	np.array([1., -1., 0.]),
	np.array([1., 1., 0.])
] # next, we'll write down their thrust vectors

# this debug motor configuration is the same as bvr auv's hovercraft

bounds = [[-1, 1]] * 4 # motors can't go outside of (-100%, 100%)...

deadzone = [[-0.1, 0.1]] * 4 # or inside (-10%, 10%), unless they equal 0 exactly

sim = Simulation(motor_locations, motor_directions, 1/6, bounds, deadzone)

sim_anchovy = AUV( # anchovy is the name of bvr auv's sub
	motor_controller = MotorController(
		inertia = InertiaBuilder(
			Cuboid(
				mass=1,
				width=1,
				height=1,
				depth=0.1,
				center=np.array([0,0,0])
			) # just a square hovercraft
		).moment_of_inertia(),

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
			# this creates a motor object for each of the motor speed and location combos
		]
	),
	sensors = SensorInterface( # create a sensor interface from the simulation's sensors
		imu=sim.imu(0.05),
		depth=sim.depth(0.)
	)
)

sim_anchovy.register_subtask(Simulate(sim)) # gotta make sure it knows to simulate the sub

sim_anchovy.register_subtask(HeadingPID(0, 0.03, 0.0, 0.01)) # this will keep it facing straight

mission = Path(
AccelerateVector(np.array([1., 0., 0., 0., 0., 0.]), 2), # start by going forward
AccelerateVector(np.array([-1, 0., 0., 0., 0., 0.]), 2), # slow down...
AccelerateVector(np.array([0., 0., 0., 0., 0., 100.]), 10) # spin as fast as you can!
) 

sim_anchovy.travel_path(mission)
sim.render() # this generates a video of the simulation using pygame in animations/animation.mp4
```

## Important Note
- Currently, the library uses Gurobi. It installs a free license using gurobipy, but eventually it expires and either an academic or (very expensive) commerical license must be used. Eventually, this will be replaced with an open-source solver.
