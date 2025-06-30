from typing import List, Callable, Optional
import numpy as np
import pulp
import quaternion

from ezauv.utils.logger import LogLevel
from ezauv import AccelerationState

class DeadzoneOptimizer:
    """
    An optimizer to determine the best motor outputs to achieve a desired acceleration,
    while respecting motor bounds and deadzones.

    This implementation uses the PuLP modeling framework with a MILP (Mixed-Integer
    Linear Program) formulation. It performs a two-stage optimization:
    1. Minimize the L1 norm of the error between the desired and achievable acceleration.
    2. With the error fixed, minimize the L1 norm of the motor thrusts to find the
       most efficient control output.
    """

    def __init__(self, M, bounds, deadzones):
        self.M = M
        self.bounds = bounds
        self.deadzones = deadzones
        self.m, self.n = M.shape

        # Create the PuLP problem. It will be reused and modified in each call to optimize().
        self.model = pulp.LpProblem("MILP_deadzone_optimizer", pulp.LpMinimize)

        # --- Variable Definitions ---
        # u: Continuous variables for motor speeds
        self.u = pulp.LpVariable.dicts("u", range(self.n), cat=pulp.LpContinuous)

        # eps: Continuous variables for the error between desired and actual acceleration
        self.eps = pulp.LpVariable.dicts("eps", range(self.m), cat=pulp.LpContinuous)

        # z: Binary variables to model whether a motor is active (outside its deadzone)
        self.z = pulp.LpVariable.dicts("z", range(self.n), cat=pulp.LpBinary)

        # s: Binary variables to select the positive or negative side of the deadzone
        self.s = pulp.LpVariable.dicts("s", range(self.n), cat=pulp.LpBinary)

        # Auxiliary variables to linearize the L1 norm (absolute value) objective
        self.eps_abs = pulp.LpVariable.dicts("eps_abs", range(self.m), lowBound=0)
        self.u_abs = pulp.LpVariable.dicts("u_abs", range(self.n), lowBound=0)

        # A large constant for the "big-M" method in MILP constraints.
        self.M0 = (
            max(abs(b) for bound in bounds for b in bound) * 1.2
        )  # Add a 20% buffer

        # --- Static Constraint Definitions ---
        # These constraints are defined once and are part of the model for every optimization.
        for i in range(self.n):
            # 1. Motor speed bounds
            self.model += self.u[i] >= self.bounds[i][0], f"u_lower_bound_{i}"
            self.model += self.u[i] <= self.bounds[i][1], f"u_upper_bound_{i}"

            # 2. Link motor speed 'u' to its on/off state 'z'. If z=0, u=0.
            self.model += self.u[i] <= self.bounds[i][1] * self.z[i], f"u_z_upper_{i}"
            self.model += self.u[i] >= self.bounds[i][0] * self.z[i], f"u_z_lower_{i}"

            # 3. Deadzone constraints. If a motor is on (z=1), its speed 'u' must
            #    be outside the deadzone range [deadzone.min, deadzone.max].
            #    This is the big-M formulation for the disjunctive constraint:
            #    (u[i] <= deadzone.min) OR (u[i] >= deadzone.max)
            self.model += (
                self.u[i] - self.deadzones[i][1] * self.s[i] + self.M0 * (1 - self.s[i])
                >= self.M0 * (1 - self.z[i]),
                f"deadzone_upper_{i}",
            )
            self.model += (
                self.u[i] - self.M0 * self.s[i] + self.deadzones[i][0] * (1 - self.s[i])
                <= self.M0 * (1 - self.z[i]),
                f"deadzone_lower_{i}",
            )

            # 4. Linearization of the L1 norm for motor speeds. |u| = u_abs
            self.model += self.u_abs[i] >= self.u[i], f"u_abs_pos_{i}"
            self.model += self.u_abs[i] >= -self.u[i], f"u_abs_neg_{i}"

        for j in range(self.m):
            # 5. Linearization of the L1 norm for the error. |eps| = eps_abs
            self.model += self.eps_abs[j] >= self.eps[j], f"eps_abs_pos_{j}"
            self.model += self.eps_abs[j] >= -self.eps[j], f"eps_abs_neg_{j}"

    def optimize(self, V):
        """
        Solves the two-stage optimization problem for a given desired acceleration vector V.
        """
        # --- Stage 1: Minimize Error ---

        # Add the physics constraint for this specific run. Overwrites previous if exists.
        for j in range(self.m):
            self.model.constraints[f"physics_{j}"] = (
                pulp.lpSum(self.M[j, i] * self.u[i] for i in range(self.n))
                + self.eps[j]
                == V[j]
            )

        # Set the objective to minimize the L1 norm of the error
        self.model.setObjective(pulp.lpSum(self.eps_abs))

        # Solve the model. msg=0 suppresses solver output.
#        self.model.solve(pulp.SCIP_CMD(msg=0))

        # If no optimal solution found, cleanup and return failure.
#        if self.model.status != pulp.LpStatusOptimal:
#            for j in range(self.m):
#                del self.model.constraints[f"physics_{j}"]
#            return False, None

        # Store the optimal error values from stage 1.
        eps_opt = [pulp.value(self.eps[j]) for j in range(self.m)]

        # --- Stage 2: Minimize Control Effort ---

        # Add constraints to fix the error to the optimal values found above.
        for j in range(self.m):
            self.model.constraints[f"fix_eps_{j}"] = self.eps[j] == eps_opt[j]

        # Change the objective to minimize the L1 norm of motor commands.
        self.model.setObjective(pulp.lpSum(self.u_abs))
        self.model.solve(pulp.SCIP_CMD(msg=0, threads=16))

        # --- Cleanup for the next run ---
        # It's crucial to remove the constraints that were specific to this run (V and eps_opt).
        for j in range(self.m):
            del self.model.constraints[f"physics_{j}"]
            del self.model.constraints[f"fix_eps_{j}"]

        # --- Return solution ---
        if self.model.status == pulp.LpStatusOptimal:
            solution = np.array([pulp.value(self.u[i]) for i in range(self.n)])
            return True, solution

        return False, None


class Motor:
    class Range:
        def __init__(self, bottom: float, top: float):
            self.max = top
            self.min = bottom

    def __init__(
        self,
        thrust_vector: np.ndarray,
        position: np.ndarray,
        set_motor: Callable,
        initialize: Callable,
        bounds: Range,
        deadzone: Range,
    ):
        self.thrust_vector: np.ndarray = thrust_vector
        self.position: np.ndarray = position
        self.set: Callable = set_motor

        self.initialize: Callable = initialize
        self.torque_vector: np.ndarray = np.cross(self.position, self.thrust_vector)

        self.bounds: Motor.Range = bounds
        self.deadzone: Motor.Range = deadzone


class MotorController:
    def __init__(self, *, inertia: np.ndarray, motors: List[Motor]):
        self.inv_inertia: np.ndarray = np.linalg.inv(
            inertia
        )  # the inverse inertia tensor of the entire body
        self.motors: np.ndarray = np.array(
            motors
        )  # the list of motors this sub owns
        self.log: Callable = lambda str, level=None: print(
            f"Motor logger is not set --- {str}"
        )

        self.optimizer: Optional[DeadzoneOptimizer] = None

        self.motor_matrix = None
        self.mT = None
        self.reset_optimizer()

        self.prev_sent = {}

    def overview(self) -> None:
        self.log("---Motor controller overview---")
        self.log(f"Inverse inertia tensor:\n{self.inv_inertia}")
        self.log(f"{len(self.motors)} motors connected")

    def initialize(self) -> None:
        self.log("Initializing motors...")

        problems = 0
        for motor in self.motors:
            problems += motor.initialize()

        level = LogLevel.INFO if problems == 0 else LogLevel.WARNING

        self.log(
            f"Motors initalized with {problems} problem{'' if problems == 1 else 's'}",
            level=level,
        )

    def reset_optimizer(self):
        """
        Recalculate the motor matrix.
        Should be called if the inertia, motor locations, or motor thrust vectors are changed.
        """
        bounds = []
        deadzones = []

        for i, motor in enumerate(self.motors):
            new_vector = np.array(
                [
                    np.concatenate(
                        [motor.thrust_vector, self.inv_inertia @ motor.torque_vector],
                        axis=None,
                    )
                ]
            ).T
            if i == 0:
                self.motor_matrix = new_vector
            else:
                self.motor_matrix = np.hstack((self.motor_matrix, new_vector))

            bounds.append((motor.bounds.min, motor.bounds.max))
            deadzones.append((motor.deadzone.min, motor.deadzone.max))
        self.optimizer = DeadzoneOptimizer(self.motor_matrix, bounds, deadzones)
        self.mT = self.motor_matrix.T

    def solve(self, wanted_acceleration: AccelerationState, rotation):
        """
        Find the array of motor speeds needed to travel at a specific thrust vector and rotation.
        Finds the next best solution if this vector is not possible.
        """
        wanted_acceleration.rotate(rotation.conjugate())
        rotated_wanted = np.append(
            wanted_acceleration.translation, wanted_acceleration.rotation
        )

        optimized = self.optimizer.optimize(rotated_wanted)
        return optimized

    def set_motors(self, motor_speeds):
        """
        Set each motor to a corresponding speed of motor_speeds.
        """
        for i, motor in enumerate(self.motors):
            speed = motor_speeds[i]
            if motor in self.prev_sent and self.prev_sent[motor] == speed:
                continue
            motor.set(speed)
            self.prev_sent[motor] = speed

    def killed(self):
        """
        Check if the last value sent to each motor was zero.
        """
        return np.all(np.isclose([view[1] for view in self.prev_sent.items()], 0))

from ezauv.utils import InertiaBuilder, Cuboid


test = MotorController(inertia=InertiaBuilder(Cuboid(10, np.array([0, 0, 0]), 5, 5, 1)).moment_of_inertia(),
                       motors=[
                            Motor(np.array([-1, 1, 0]), np.array([-1, -1, 0]), lambda num: print(num), lambda _: 0, Motor.Range(-0.2, 0.2), Motor.Range(0.11, 0.11)),
                            Motor(np.array([1, 1, 0]), np.array([1, -1, 0]), lambda num: print(num), lambda _: 0, Motor.Range(-0.2, 0.2), Motor.Range(0.11, 0.11)),
                            Motor(np.array([1, 1, 0]), np.array([-1, 1, 0]), lambda num: print(num), lambda _: 0, Motor.Range(-0.2, 0.2), Motor.Range(0.11, 0.11)),
                            Motor(np.array([-1, 1, 0]), np.array([1, 1, 0]), lambda num: print(num), lambda _: 0, Motor.Range(-0.2, 0.2), Motor.Range(0.11, 0.11)),
                            Motor(np.array([0, 0, 1]), np.array([0, 0.5, 0]), lambda num: print(num), lambda _: 0, Motor.Range(-0.2, 0.2), Motor.Range(0.11, 0.11)),
                            Motor(np.array([0, 0, 1]), np.array([0, -0.5, 0]), lambda num: print(num), lambda _: 0, Motor.Range(-0.2, 0.2), Motor.Range(0.11, 0.11))
                           ]
                       )

target = np.array([0.9,0,0,0,0,0])
#print(test.solve(target, quaternion.from_euler_angles(np.deg2rad(0), 0, 0)))
#test.set_motors(test.solve(target)[1])
#print(quaternion.rotate_vectors(np.quaternion(1,0,0,0), np.array([np.array([1,1,1]), np.array([1,1,1])])))
import time
avg = 0.
count = 0
for i in np.linspace(-1, 1, 10):
    count += 1
    start = time.time()
    print(test.solve(AccelerationState(), quaternion.from_euler_angles(0, 0, 0)))
    avg += time.time() - start

print(f"Average time taken: {(avg / count) * 1000} milliseconds")
