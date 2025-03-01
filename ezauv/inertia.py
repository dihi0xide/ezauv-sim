from abc import ABC, abstractmethod
import numpy as np

class InertiaGeometry(ABC):

    def __init__(self, mass: int, center: np.ndarray):
        self.mass = mass
        self.center = center

    @abstractmethod
    def inertia_tensor(self):
        pass

    def translate(self, inertia, displacement_vector):
        I_0 = inertia
        m = self.mass 
        R = displacement_vector
        return I_0 + m * (np.dot(R, R) * np.eye(3) - np.outer(R, R))
    
    def rotate(self, inertia, rotation_matrix):
        I_0 = inertia
        R = rotation_matrix
        R_T = rotation_matrix.T
        return R @ I_0 @ R_T
    
    def rotate_to_vector(self, inertia, current_facing, to_face):
        
        if(np.all(np.isclose(current_facing, to_face))):
            return inertia
            
        rotation_axis = np.cross(to_face, current_facing)
        rotation_axis /= np.linalg.norm(rotation_axis)
        rotation_angle = np.arccos(np.dot(to_face, current_facing))

        K = np.array([
            [0, -rotation_axis[2], rotation_axis[1]],
            [rotation_axis[2], 0, -rotation_axis[0]],
            [-rotation_axis[1], rotation_axis[0], 0]
        ])
        R = np.eye(3) + K + np.dot(np.square(K), (1 - np.cos(rotation_angle))/np.sin(rotation_angle))

        return self.rotate(inertia, R)

    def shift_center(self, inertia, new_center):
        displacement = new_center - self.center
        return self.translate(inertia, displacement)



class Sphere(InertiaGeometry):

    def __init__(self, mass, center, radius):
        super().__init__(mass, center)
        self.radius = radius

    def inertia_tensor(self):
        I = (2 / 5) * self.mass * self.radius**2
        return I * np.eye(3)
    
class HollowCylinder(InertiaGeometry):

    def __init__(self, mass, center, inner_radius, outer_radius, height, facing):
        # facing is unit vector of flat side
        super().__init__(mass, center)
        self.inner_radius = inner_radius
        self.outer_radius = outer_radius
        self.height = height
        self.facing = facing / np.linalg.norm(facing)

    def inertia_tensor(self):

        R = self.outer_radius
        k = self.inner_radius
        h = self.height

        I_zz = (1/2) * self.mass * (R**2 + k**2)  
        I_xx_yy = (1/4) * self.mass * (R**2 + k**2) + (1/12) * self.mass * h**2

        principal_tensor = np.diag([I_xx_yy, I_xx_yy, I_zz])

        return self.rotate_to_vector(principal_tensor, np.array([0, 0, 1]), self.facing)
    
class Cuboid(InertiaGeometry):

    def __init__(self, mass, center, width, height, depth):
        super().__init__(mass, center)
        self.width = width 
        self.height = height 
        self.depth = depth 

    def inertia_tensor(self):
        a = self.width
        b = self.height
        c = self.depth

        I_xx = (1 / 12) * self.mass * (b**2 + c**2)
        I_yy = (1 / 12) * self.mass * (a**2 + c**2)
        I_zz = (1 / 12) * self.mass * (a**2 + b**2)

        I = np.diag([I_xx, I_yy, I_zz])  # noqa: E741
        return I




class InertiaBuilder:
   
    def __init__(self, *args: InertiaGeometry):
        self.geometries = args

    def moment_of_inertia(self, center: np.ndarray = np.array([0, 0, 0])):

        total_inertia = np.zeros((3, 3))
        for geometry in self.geometries:
            shifted = geometry.shift_center(geometry.inertia_tensor(), center)
            total_inertia += shifted

        return total_inertia

# HollowCylinder(1, 1, 1, 1, 1, np.array([0, 0, 1])).inertia_tensor()