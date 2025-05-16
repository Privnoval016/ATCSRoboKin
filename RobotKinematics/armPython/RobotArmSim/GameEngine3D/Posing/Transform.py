import numpy as np
import scipy

from RobotArmSim.GameEngine3D.Posing.RotationMatrix import RotationMatrix
from RobotArmSim.GameEngine3D.Posing.Vector3 import Vector3


class Transform:

    def __init__(self, position=Vector3(0, 0, 0), rotation=RotationMatrix(0, 0, 0),
                 parent=None):
        self.parent = parent
        parent.add_child(self) if parent else None
        self.childs = []

        self.homogeneous_matrix = Transform.calculate_homogeneous_matrix(position, rotation)


    def local_homogeneous_matrix(self):
        if self.parent is not None:
            return self.parent.inverse_transform() @ self.homogeneous_matrix
        else:
            return self.homogeneous_matrix

    def inverse_transform(self):
        return np.linalg.inv(self.homogeneous_matrix)

    def position(self):
        return Vector3(*self.homogeneous_matrix[:3, 3])

    def rotation(self):
        return RotationMatrix(self.homogeneous_matrix[:3, :3])

    def local_position(self):
        return Vector3(*self.local_homogeneous_matrix()[:3, 3])

    def local_rotation(self):
        return RotationMatrix(self.local_homogeneous_matrix()[:3, :3])


    def set_position(self, position):
        self.set_homogeneous_matrix(Transform.calculate_homogeneous_matrix(position, self.rotation()))

    def set_rotation(self, rotation):
        self.set_homogeneous_matrix(Transform.calculate_homogeneous_matrix(self.position(), rotation))

    def set_local_position(self, position):
        if self.parent is not None:
            self.set_homogeneous_matrix(self.parent.homogeneous_matrix @ Transform.calculate_homogeneous_matrix(position, self.rotation()))
        else:
            self.set_homogeneous_matrix(Transform.calculate_homogeneous_matrix(position, self.rotation()))

    def set_local_rotation(self, rotation):
        if self.parent is not None:
            self.set_homogeneous_matrix(self.parent.homogeneous_matrix @ Transform.calculate_homogeneous_matrix(self.position(), rotation))
        else:
            self.set_homogeneous_matrix(Transform.calculate_homogeneous_matrix(self.position(), rotation))

    def set_homogeneous_matrix(self, matrix):
        old_global = self.homogeneous_matrix
        self.homogeneous_matrix = matrix
        self.update_child_transforms(old_global)

    def set_local_homogeneous_matrix(self, matrix):
        if self.parent is not None:
            self.set_homogeneous_matrix(self.parent.homogeneous_matrix @ matrix)
        else:
            self.set_homogeneous_matrix(matrix)


    def set_transform(self, transform):
        self.set_homogeneous_matrix(transform.homogeneous_matrix)
        self.update_child_transforms(self.homogeneous_matrix)


    def __repr__(self):
        return f"Transform(position={self.position}, rotation={self.rotation})"


    def set_parent(self, parent):
        parent.remove_child(self) if self.parent else None
        self.parent = parent
        parent.add_child(self) if parent else None


    def add_child(self, child):
        self.childs.append(child)
        child.parent = self


    def remove_child(self, child):
        if child in self.childs:
            self.childs.remove(child)
            child.parent = None
        else:
            raise ValueError("Child not found in this transform's children.")


    def update_child_transforms(self, old_global):
        for child in self.childs:
            child.set_homogeneous_matrix( self.homogeneous_matrix @ np.linalg.inv(old_global) @
                child.homogeneous_matrix)


    def rotate_angle_axis(self, angle, axis):
        turn = axis.normalized()
        R = RotationMatrix(turn.to_rotation(angle))
        self.set_rotation(R @ self.rotation())


    def translate(self, distance, direction):
        self.set_position(self.position() + distance * direction)


    @staticmethod
    def calculate_homogeneous_matrix(position, rotation):
        R = np.eye(4)
        R[:3, :3] = rotation.matrix
        R[:3, 3] = position.vector
        return R

    def forward(self):
        return self.rotation() @ Vector3.forward()


    def right(self):
        return self.rotation() @ Vector3.right()


    def up(self):
        return self.rotation() @ Vector3.up()


    @staticmethod
    def look_at(pos, target, up):
        forward = (target - pos).normalized()
        right = (up.cross(forward)).normalized()
        new_up = forward.cross(right)

        mat = np.eye(4, dtype=np.float32)
        mat[0, :3] = right.vector
        mat[1, :3] = new_up.vector
        mat[2, :3] = -forward.vector
        mat[:3, 3] = -mat[:3, :3] @ pos.vector
        return mat


    @staticmethod
    def screw(angular_velocity, q):
        ang_vel_mat = Vector3.skew_symmetric(angular_velocity)
        screw_mat = np.zeros((4, 4))

        screw_mat[:3, :3] = ang_vel_mat
        screw_mat[:3, 3] = -angular_velocity.cross(q).vector

        return screw_mat


    @staticmethod
    def twist_to_six_vector(twist):
        v = twist[:3, 3]

        w_mat = twist[:3, :3]
        w = np.zeros(3)
        w[0] = w_mat[2, 1]
        w[1] = w_mat[0, 2]
        w[2] = w_mat[1, 0]

        return np.concatenate((w, v))

    @staticmethod
    def adjoint_map(transform):
        R = transform[:3, :3]
        p = Vector3(*transform[:3, 3])

        p_skew = p.skew_symmetric()

        adjoint = np.zeros((6, 6))
        adjoint[:3, :3] = R
        adjoint[3:6, 3:6] = R
        adjoint[3:6, :3] = p_skew @ R

        return adjoint






