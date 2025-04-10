import numpy as np

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
        # Set the position of the transform
        self.set_homogeneous_matrix(Transform.calculate_homogeneous_matrix(position, self.rotation()))

    def set_rotation(self, rotation):
        # Set the rotation of the transform
        self.set_homogeneous_matrix(Transform.calculate_homogeneous_matrix(self.position(), rotation))

    def set_homogeneous_matrix(self, matrix):
        # Set the homogeneous matrix of the transform
        old_global = self.homogeneous_matrix
        self.homogeneous_matrix = matrix
        self.update_child_transforms(old_global)

    def set_local_homogeneous_matrix(self, matrix):
        # Set the local homogeneous matrix of the transform
        old_global = self.homogeneous_matrix
        if self.parent is not None:
            self.set_homogeneous_matrix(self.parent.homogeneous_matrix @ matrix)
        else:
            self.set_homogeneous_matrix(matrix)


    def set_transform(self, transform):
        # Set the transform of the transform
        self.set_homogeneous_matrix(transform.homogeneous_matrix)
        self.update_child_transforms(self.homogeneous_matrix)


    def __repr__(self):
        return f"Transform(position={self.position}, rotation={self.rotation})"


    def set_parent(self, parent):
        parent.remove_child(self) if self.parent else None
        self.parent = parent
        parent.add_child(self) if parent else None


    def add_child(self, child):
        # Add a child transform
        self.childs.append(child)
        child.parent = self


    def remove_child(self, child):
        # Remove a child transform
        if child in self.childs:
            self.childs.remove(child)
            child.parent = None
        else:
            raise ValueError("Child not found in this transform's children.")


    def update_child_transforms(self, old_global):
        # Update the transforms of all child transforms
        for child in self.childs:
            child.set_homogeneous_matrix( self.homogeneous_matrix @ np.linalg.inv(old_global) @
                child.homogeneous_matrix)


    def rotate_angle_axis(self, angle, axis):
        turn = axis.normalized()
        R = turn.skew_symmetric(angle)
        self.set_rotation(RotationMatrix(R) @ self.rotation())


    def translate(self, distance, direction):
        self.set_position(self.position() + distance * direction)


    @staticmethod
    def calculate_homogeneous_matrix(position, rotation):
        # Create a 4x4 homogeneous transformation matrix
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
    def perspective(fov, aspect, near, far):
        f = 1 / np.tan(fov / 2)
        q = far / (far - near)

        return np.array([
            [f / aspect, 0, 0, 0],
            [0, f, 0, 0],
            [0, 0, q, 1],
            [0, 0, -q * near, 0]
        ], dtype=np.float32)


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
