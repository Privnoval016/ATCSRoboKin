import numpy as np

class Vector3:
    def __init__(self, x=0, y=0, z=0):
        self.vector = np.array([x, y, z], dtype=float).squeeze()


    def x(self):
        return self.vector[0]


    def y(self):
        return self.vector[1]


    def z(self):
        return self.vector[2]


    @staticmethod
    def up():
        return Vector3(0, 1, 0)

    @staticmethod
    def right():
        return Vector3(1, 0, 0)

    @staticmethod
    def forward():
        return Vector3(0, 0, 1)

    @staticmethod
    def one():
        return Vector3(1, 1, 1)

    @staticmethod
    def zero():
        return Vector3(0, 0, 0)


    def __repr__(self):
        return f"Vector3({self.vector[0]}, {self.vector[1]}, {self.vector[2]})"


    def __add__(self, other):
        if isinstance(other, Vector3):
            return Vector3(self.vector[0] + other.vector[0],
                           self.vector[1] + other.vector[1],
                           self.vector[2] + other.vector[2])
        else:
            raise TypeError("Unsupported operand type(s) for +: 'Vector3' and '{}'".format(type(other)))


    def __sub__(self, other):
        if isinstance(other, Vector3):
            return Vector3(self.vector[0] - other.vector[0],
                           self.vector[1] - other.vector[1],
                           self.vector[2] - other.vector[2])
        else:
            raise TypeError("Unsupported operand type(s) for -: 'Vector3' and '{}'".format(type(other)))


    def __mul__(self, scalar):
        if isinstance(scalar, (int, float)):
            return Vector3(self.vector[0] * scalar,
                           self.vector[1] * scalar,
                           self.vector[2] * scalar)
        else:
            raise TypeError("Unsupported operand type(s) for *: 'Vector3' and '{}'".format(type(scalar)))


    def __truediv__(self, scalar):
        if isinstance(scalar, (int, float)):
            if scalar == 0:
                raise ZeroDivisionError("division by zero")
            return Vector3(self.vector[0] / scalar,
                           self.vector[1] / scalar,
                           self.vector[2] / scalar)
        else:
            raise TypeError("Unsupported operand type(s) for /: 'Vector3' and '{}'".format(type(scalar)))

    def __neg__(self):
        return Vector3(-self.vector[0], -self.vector[1], -self.vector[2])

    def __eq__(self, other):
        if isinstance(other, Vector3):
            return np.array_equal(self.vector, other.vector)
        else:
            raise TypeError("Unsupported operand type(s) for ==: 'Vector3' and '{}'".format(type(other)))


    def __ne__(self, other):

        if isinstance(other, Vector3):
            return not np.array_equal(self.vector, other.vector)
        else:
            raise TypeError("Unsupported operand type(s) for !=: 'Vector3' and '{}'".format(type(other)))



    def __iter__(self):
        return iter(self.vector)


    def homogeneous(self, w=1):
        return np.append(self.vector * w, w)


    def magnitude(self):
        return np.linalg.norm(self.vector)


    def normalized(self):
        mag = self.magnitude()
        if mag == 0:
            return Vector3.zero()
        return self / mag


    def dot(self, other):
        if isinstance(other, Vector3):
            return np.dot(self.vector, other.vector)
        else:
            raise TypeError("Unsupported operand type(s) for dot: 'Vector3' and '{}'".format(type(other)))


    def cross(self, other):
        if isinstance(other, Vector3):
            return Vector3(*np.cross(self.vector, other.vector))
        else:
            raise TypeError("Unsupported operand type(s) for cross: 'Vector3' and '{}'".format(type(other)))


    def distance(self, other):
        if isinstance(other, Vector3):
            return np.linalg.norm(self.vector - other.vector)
        else:
            raise TypeError("Unsupported operand type(s) for distance: 'Vector3' and '{}'".format(type(other)))


    def angle_between(self, other):
        if isinstance(other, Vector3):
            dot_product = self.dot(other)
            mag_self = self.magnitude()
            mag_other = other.magnitude()
            if mag_self == 0 or mag_other == 0:
                raise ValueError("Cannot calculate angle with zero vector")
            cos_theta = dot_product / (mag_self * mag_other)
            return np.arccos(np.clip(cos_theta, -1.0, 1.0))
        else:
            raise TypeError("Unsupported operand type(s) for angle_between: 'Vector3' and '{}'".format(type(other)))


    def to_rotation(self, angle):
        axis = self.normalized()
        K = axis.skew_symmetric()

        I = np.eye(3)

        R = I + np.sin(angle) * K + (1 - np.cos(angle)) * (K @ K)
        return R


    def skew_symmetric(self):
        x, y, z = self.x(), self.y(), self.z()
        return np.array([
            [0, -z, y],
            [z, 0, -x],
            [-y, x, 0]
        ])
