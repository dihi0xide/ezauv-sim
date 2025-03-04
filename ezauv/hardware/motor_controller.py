from typing import List, Callable, Optional
import numpy as np
from gurobipy import GRB, Model, quicksum
import quaternion
import time

from ezauv.utils.logger import LogLevel


class DeadzoneOptimizer:
    #TODO actually document any of this
    # ok just for future reference b4 documentation bounds are like (-1, 1) and deadzones are like (0.1, 0.1). 
    # deadzones do not go negative! might change in future. probably will use ranges from motors?
    # need 1 per motor, so both are lists
    def __init__(self, M, bounds, deadzones):
        self.M = M
        self.bounds = bounds
        self.deadzones = deadzones
        self.m, self.n = M.shape

        self.model = Model("MIQP_deadzone") # miqp = mixed integer quadratic programming
        # quadratic because it minimized the sum of squares of elements of the matrix
        # integer because it uses boolean variables to determine what side of the deadzone the continous variables are on
        # mixed because it also has continuous
        # programming meaning optimization, because it minimizes the sum of squares

        self.eps = self.model.addVars(self.m, lb=-float('inf'), vtype=GRB.CONTINUOUS, name="eps")

        self.u = {}
        for i in range(self.n):
            self.u[i] = self.model.addVar(lb=bounds[i][0], ub=bounds[i][1], vtype=GRB.CONTINUOUS, name=f"u_{i}")

        self.z = self.model.addVars(self.n, vtype=GRB.BINARY, name="z")
        self.s = self.model.addVars(self.n, vtype=GRB.BINARY, name="s")
        

        self.M0 = max(abs(b) for bound in bounds for b in bound)

        for i in range(self.n):
            self.model.addConstr(self.u[i] >= -self.z[i] * bounds[i][1], name=f"u_lower_bound_{i}")
            self.model.addConstr(self.u[i] <= self.z[i] * bounds[i][1], name=f"u_upper_bound_{i}")
            # either bounded in (-b, b) or (0, 0)

        
        for i in range(self.n):
            self.model.addGenConstrIndicator(self.z[i], 1,
                self.u[i] - deadzones[i][1] * self.s[i] + self.M0 * (1 - self.s[i]),
                GRB.GREATER_EQUAL, 0, name=f"deadzone_lower_{i}")

            self.model.addGenConstrIndicator(self.z[i], 1,
                self.u[i] - self.M0 * self.s[i] + deadzones[i][0] * (1 - self.s[i]),
                GRB.LESS_EQUAL, 0, name=f"deadzone_upper_{i}")


        self.constrs = []
        for j in range(self.m):
            expr = quicksum(self.M[j, i] * (self.u[i]) for i in range(self.n)) + self.eps[j]
            # expr = self.eps[j] - 1
            # expr = quicksum(M[j, i] * self.u[i] for i in range(self.n))
            # expr = 1 - self.eps[j]
            self.constrs.append(self.model.addConstr(expr == 0, name=f"eq_row_{j}"))
            # matrix multiplication must be true

        self.model.Params.OutputFlag = 0
        self.model.update()

        

    def optimize(self, V, M):
        for j in range(self.m):
            self.constrs[j].setAttr(GRB.Attr.RHS, V[j])
            for i in range(self.n):
                self.model.chgCoeff(self.constrs[j], self.u[i], M[j, i])

        self.model.setObjective(quicksum(self.eps[j] * self.eps[j] for j in range(self.m)), GRB.MINIMIZE)
        self.model.optimize()

        if self.model.status != GRB.OPTIMAL:
            return False, None


        eps_opt = [self.eps[j].X for j in range(self.m)]
        # print(eps_opt)
        # if(not np.all(np.isclose(eps_opt, 0.0))):
            # set_text("rotation: " + " ".join(str(t) for t in V))
            # print(V[5])
        
        for j in range(self.m):
            self.eps[j].LB = eps_opt[j]
            self.eps[j].UB = eps_opt[j]

        self.model.setObjective(quicksum(self.u[i] * self.u[i] for i in range(self.n)), GRB.MINIMIZE)
        self.model.optimize()

        for j in range(self.m):
            self.eps[j].LB = -float('inf')
            self.eps[j].UB = float('inf')
        if self.model.status == GRB.OPTIMAL:
            return True, np.array([self.u[i].X for i in range(self.n)])

        return False, None


test = []
class Motor:

    class Range:
        def __init__(self, bottom: float, top: float):
            self.max = top
            self.min = bottom

    def __init__(self, thrust_vector: np.ndarray, position: np.ndarray, set_motor: Callable, initialize: Callable, bounds: Range, deadzone: Range):
        self.thrust_vector: np.ndarray = thrust_vector
        self.position: np.ndarray = position
        self.set: Callable = set_motor

        self.initialize: Callable = initialize
        self.inertia_tensor: Optional[np.ndarray] = None
        self.torque_vector: Optional[np.ndarray] = None

        self.bounds: Motor.Range = bounds
        self.deadzone: Motor.Range = deadzone

    def set_inertia_tensor(self, inertia_tensor):
        self.inertia_tensor = inertia_tensor
        self.torque_vector = np.cross(self.position, self.thrust_vector)    


class MotorController:

    def __init__(self, *, inertia: np.ndarray, motors: List[Motor]):
        self.inertia: np.ndarray = inertia  # the inertia tensor of the entire body
        self.motors: np.ndarray = np.array(motors)   # the list of motors this sub owns
        self.log: Callable = lambda str, level=None: print(f"Motor logger is not set --- {str}")

        self.optimizer: Optional[DeadzoneOptimizer] = None

        for motor in motors:
            motor.set_inertia_tensor(self.inertia)

        self.motor_matrix = None
        self.mT = None
        self.reset_optimizer()

    def overview(self) -> None:
        self.log("---Motor controller overview---")
        self.log(f"Inertia tensor:\n{self.inertia}")
        self.log(f"{len(self.motors)} motors connected")

    def initialize(self) -> None:
        self.log("Initializing motors...")
 
        problems = 0
        for motor in self.motors:
            problems += motor.initialize()
        
        level = LogLevel.INFO if problems == 0 else LogLevel.WARNING

        self.log(f"Motors initalized with {problems} problem{'' if problems==1 else 's'}", level=level)
    
    def reset_optimizer(self):
        bounds = []
        deadzones = []

        for i, motor in enumerate(self.motors):
            # print(motor.thrust_vector)
            new_vector = np.array([np.concatenate([motor.thrust_vector, self.inertia @ motor.torque_vector], axis=None)]).T
            if(i == 0):
                self.motor_matrix = new_vector
            else:
                self.motor_matrix = np.hstack((self.motor_matrix, new_vector))

            bounds.append((motor.bounds.min, motor.bounds.max))
            deadzones.append((motor.deadzone.min, motor.deadzone.max))
        self.optimizer = DeadzoneOptimizer(self.motor_matrix, bounds, deadzones)
        self.mT = self.motor_matrix.T

    def rotate(self, rotation):
        rotated_vectors = []
        for vec in self.mT:
            split_vec = np.split(vec, [3])
            # a = []
            # a.extend([quaternion.rotate_vectors(rotation, split_vec)])
            # print(a)
            rotated_vectors.append([val for sublist in quaternion.rotate_vectors(rotation, split_vec) for val in sublist])

        return np.array(rotated_vectors).T

    def solve(self, wanted_vector, rotation):
        # print(self.motor_matrix)
        # print(self.rotate(rotation))
        # wanted_vector = 
        # print(self.optimizer.optimize(np.array([0., 0., 0., 0., 0., 2295.18311443701]), self.rotate(rotation)))
        # raise Exception()
        start = time.time()
        optimized = self.optimizer.optimize(wanted_vector, self.rotate(rotation))
        test.append(time.time() - start)
        # if(not optimized[0]):
        return optimized
    
    def set_motors(self, motor_speeds):
        # print(len(motor_speeds))
        # print(motor_speed)
        # print(self.motor_matrix)
        # print(motor_speeds)
        for i, motor in enumerate(self.motors):
            motor.set(motor_speeds[i])


# from inertia import *


# test = MotorController(inertia=InertiaBuilder(Cuboid(10, np.array([0, 0, 0]), 5, 5, 1)).moment_of_inertia(), 
#                        motors=[
#                             Motor(np.array([-1, 1, 0]), np.array([-1, -1, 0]), lambda num: print(num), lambda _: 0, Motor.Range(-0.2, 0.2), Motor.Range(0.11, 0.11)),
#                             Motor(np.array([1, 1, 0]), np.array([1, -1, 0]), lambda num: print(num), lambda _: 0, Motor.Range(-0.2, 0.2), Motor.Range(0.11, 0.11)),
#                             Motor(np.array([1, 1, 0]), np.array([-1, 1, 0]), lambda num: print(num), lambda _: 0, Motor.Range(-0.2, 0.2), Motor.Range(0.11, 0.11)),
#                             Motor(np.array([-1, 1, 0]), np.array([1, 1, 0]), lambda num: print(num), lambda _: 0, Motor.Range(-0.2, 0.2), Motor.Range(0.11, 0.11)),
#                             Motor(np.array([0, 0, 1]), np.array([0, 0.5, 0]), lambda num: print(num), lambda _: 0, Motor.Range(-0.2, 0.2), Motor.Range(0.11, 0.11)),
#                             Motor(np.array([0, 0, 1]), np.array([0, -0.5, 0]), lambda num: print(num), lambda _: 0, Motor.Range(-0.2, 0.2), Motor.Range(0.11, 0.11))
#                            ]
#                        )

# # target = np.array([0.9,0,0,0,0,0])
# # print(test.solve(target, quaternion.from_euler_angles(np.deg2rad(0), 0, 0)))
# # # test.set_motors(test.solve(target)[1])
# # # print(quaternion.rotate_vectors(np.quaternion(1,0,0,0), np.array([np.array([1,1,1]), np.array([1,1,1])])))
# # import time
# # avg = 0.
# # count = 0
# # for i in np.linspace(-1, 1, 1000):
# #     count += 1
# #     start = time.time()
# #     test.solve(np.array([i, 0, 0, 0, 0, 0]), quaternion.from_euler_angles(0, 0, 0))
# #     avg += time.time() - start

# # print(f"Average time taken: {(avg / count) * 1000} milliseconds")