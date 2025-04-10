import numpy as np

class Angle:
    def __init__(self, angle, degrees=False):
        self.angle = angle % (2 * np.pi) if not degrees else np.radians(angle) % (2 * np.pi)
        self.degrees = np.degrees(self.angle)
        self.sin = np.sin(angle)
        self.cos = np.cos(angle)
        self.tan = np.tan(angle)

    def __repr__(self):
        return f"Angle({self.angle})"

    def __add__(self, other):
        if isinstance(other, Angle):
            return Angle(self.angle + other.angle)
        elif isinstance(other, (int, float)):
            return Angle(self.angle + other)
        else:
            raise TypeError("Unsupported operand type(s) for +: 'Angle' and '{}'".format(type(other)))

    def __sub__(self, other):
        if isinstance(other, Angle):
            return Angle(self.angle - other.angle)
        elif isinstance(other, (int, float)):
            return Angle(self.angle - other)
        else:
            raise TypeError("Unsupported operand type(s) for -: 'Angle' and '{}'".format(type(other)))

    def __mul__(self, scalar):
        if isinstance(scalar, (int, float)):
            return Angle(self.angle * scalar)
        else:
            raise TypeError("Unsupported operand type(s) for *: 'Angle' and '{}'".format(type(scalar)))

    def __truediv__(self, scalar):
        if isinstance(scalar, (int, float)):
            if scalar == 0:
                raise ZeroDivisionError("division by zero")
            return Angle(self.angle / scalar)
        else:
            raise TypeError("Unsupported operand type(s) for /: 'Angle' and '{}'".format(type(scalar)))

    def __eq__(self, other):
        if isinstance(other, Angle):
            return np.isclose(self.angle, other.angle)
        elif isinstance(other, (int, float)):
            return np.isclose(self.angle, Angle(other).angle)
        else:
            raise TypeError("Unsupported operand type(s) for ==: 'Angle' and '{}'".format(type(other)))

    def __ne__(self, other):
        if isinstance(other, Angle):
            return not np.isclose(self.angle, other.angle)
        elif isinstance(other, (int, float)):
            return not np.isclose(self.angle, Angle(other).angle)
        else:
            raise TypeError("Unsupported operand type(s) for !=: 'Angle' and '{}'".format(type(other)))


    @staticmethod
    def normalize(angle, degrees=False):
        return angle % (2 * np.pi) if not degrees else np.radians(angle) % (2 * np.pi)
