import numpy as np

from RobotArmSim.GameEngine3D.Posing.Angle import Angle
from RobotArmSim.GameEngine3D.Posing.Vector3 import Vector3


class RotationMatrix:
    def __init__(self, roll=0, pitch=0, yaw=0, degrees=False):
        if isinstance(roll, np.ndarray) and roll.shape == (3, 3):
            self.matrix = roll
            self.roll, self.pitch, self.yaw = RotationMatrix.rotation_matrix_to_euler_zyx(roll)
        else:
            if degrees:
                roll = np.radians(roll)
                pitch = np.radians(pitch)
                yaw = np.radians(yaw)

            self.roll = Angle(roll)
            self.pitch = Angle(pitch)
            self.yaw = Angle(yaw)

        self.euler_angles = Vector3(np.degrees(self.roll.angle),
                                    np.degrees(self.pitch.angle),
                                    np.degrees(self.yaw.angle))

        self.matrix = RotationMatrix.calculate_rotation_matrix(self.roll.angle, self.pitch.angle, self.yaw.angle)

    @staticmethod
    def rotation_matrix_to_euler_zyx(R):
        if abs(R[2][0]) != 1:
            pitch = -np.arcsin(R[2][0])
            roll = np.arctan2(R[2][1] / np.cos(pitch),
                              R[2][2] / np.cos(pitch))
            yaw = np.arctan2(R[1][0] / np.cos(pitch),
                             R[0][0] / np.cos(pitch))
        else:
            # Gimbal lock
            yaw = 0
            if R[2][0] == -1:
                pitch = np.pi / 2
                roll = yaw + np.arctan2(R[0][1], R[0][2])
            else:
                pitch = -np.pi / 2
                roll = -yaw + np.arctan2(-R[0][1], -R[0][2])

        return Angle(roll), Angle(pitch), Angle(yaw)

    def __repr__(self):
        return f"RotationMatrix(roll={self.roll}, pitch={self.pitch}, yaw={self.yaw})"

    def __add__(self, other):
        if isinstance(other, RotationMatrix):
            return RotationMatrix(self.roll.angle + other.roll.angle,
                                  self.pitch.angle + other.pitch.angle,
                                  self.yaw.angle + other.yaw.angle)
        else:
            raise TypeError("Unsupported operand type(s) for +: 'RotationMatrix' and '{}'".format(type(other)))

    def __sub__(self, other):
        if isinstance(other, RotationMatrix):
            return RotationMatrix(self.roll.angle - other.roll.angle,
                                  self.pitch.angle - other.pitch.angle,
                                  self.yaw.angle - other.yaw.angle)
        else:
            raise TypeError("Unsupported operand type(s) for -: 'RotationMatrix' and '{}'".format(type(other)))

    def __mul__(self, scalar):
        if isinstance(scalar, (int, float)):
            return RotationMatrix(self.roll.angle * scalar,
                                  self.pitch.angle * scalar,
                                  self.yaw.angle * scalar)
        else:
            raise TypeError("Unsupported operand type(s) for *: 'RotationMatrix' and '{}'".format(type(scalar)))

    def __truediv__(self, scalar):
        if isinstance(scalar, (int, float)):
            if scalar == 0:
                raise ZeroDivisionError("division by zero")
            return RotationMatrix(self.roll.angle / scalar,
                                  self.pitch.angle / scalar,
                                  self.yaw.angle / scalar)
        else:
            raise TypeError("Unsupported operand type(s) for /: 'RotationMatrix' and '{}'".format(type(scalar)))


    def __eq__(self, other):
        if isinstance(other, RotationMatrix):
            return (np.isclose(self.roll.angle, other.roll.angle) and
                    np.isclose(self.pitch.angle, other.pitch.angle) and
                    np.isclose(self.yaw.angle, other.yaw.angle))
        else:
            raise TypeError("Unsupported operand type(s) for ==: 'RotationMatrix' and '{}'".format(type(other)))

    def __ne__(self, other):
        if isinstance(other, RotationMatrix):
            return not (np.isclose(self.roll.angle, other.roll.angle) and
                        np.isclose(self.pitch.angle, other.pitch.angle) and
                        np.isclose(self.yaw.angle, other.yaw.angle))
        else:
            raise TypeError("Unsupported operand type(s) for !=: 'RotationMatrix' and '{}'".format(type(other)))

    def __matmul__(self, other):
        if isinstance(other, RotationMatrix):
            # Matrix multiplication
            return RotationMatrix(self.matrix @ other.matrix)
        if isinstance(other, Vector3):
            # Vector rotation
            return Vector3(*(self.matrix @ other.vector))
        else:
            raise TypeError("Unsupported operand type(s) for @: 'RotationMatrix' and '{}'".format(type(other)))

    @staticmethod
    def calculate_rotation_matrix(roll, pitch, yaw):
        # Rotation around the x-axis (roll)
        Rx = np.array([[1, 0, 0],
                       [0, np.cos(roll), -np.sin(roll)],
                       [0, np.sin(roll), np.cos(roll)]])

        # Rotation around the y-axis (pitch)
        Ry = np.array([[np.cos(pitch), 0, np.sin(pitch)],
                       [0, 1, 0],
                       [-np.sin(pitch), 0, np.cos(pitch)]])

        # Rotation around the z-axis (yaw)
        Rz = np.array([[np.cos(yaw), -np.sin(yaw), 0],
                       [np.sin(yaw), np.cos(yaw), 0],
                       [0, 0, 1]])

        # Combined rotation matrix
        R = Rz @ Ry @ Rx
        return R

    def inverse(self):
        # Invert the rotation matrix
        return RotationMatrix(np.linalg.inv(self.matrix))

    @staticmethod
    def copy(matrix):
        return RotationMatrix(matrix.roll, matrix.pitch, matrix.yaw)


    @staticmethod
    def identity():
        return RotationMatrix(0, 0, 0)