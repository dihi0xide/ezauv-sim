import quaternion
import numpy as np

class AccelerationState:
    def __init__(self, *,
                 Tx=0,
                 Ty=0,
                 Tz=0,
                 Rx=0,
                 Ry=0,
                 Rz=0
                 ):
        self.translation = np.array([Tx, Ty, Tz])
        self.rotation = np.array([Rx, Ry, Rz])

    def quaternion(self):
        return quaternion.from_euler_angles(self.rotation)
    
    def rotate(self, R):
        self.translation = quaternion.as_vector_part(
            R * quaternion.from_vector_part(self.translation) * R.conjugate()
        )
        self.rotation = quaternion.as_vector_part(
            R * quaternion.from_vector_part(self.rotation) * R.conjugate()
        )

    def __add__(self, other):

        if isinstance(other, AccelerationState):
            t = self.translation + other.translation
            r = self.rotation + other.rotation
            return AccelerationState(
                Tx=t[0], Ty=t[1], Tz=t[2],
                Rx=r[0], Ry=r[1], Rz=r[2]
            )
        else:
            raise TypeError(f"Unsupported operand type for +: AccelerationState and {type(other)}")
        
    def __str__(self):
        t = self.translation
        r = self.rotation
        return f"AccelerationState object: T=[{t[0]}, {t[1]}, {t[2]}], R=[{r[0]}, {r[1]}, {r[2]}]"