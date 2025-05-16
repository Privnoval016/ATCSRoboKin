import numpy as np

from RobotArmSim.GameEngine3D.Posing.RotationMatrix import RotationMatrix
from RobotArmSim.GameEngine3D.Posing.Transform import Transform
from RobotArmSim.GameEngine3D.Posing.Vector3 import Vector3


class Camera:
    def __init__(self, transform, target, up=Vector3.up(),
                 fov=np.radians(70), aspect=1.0, near=0.1, far=100.0):
        self.transform = transform
        self.target = Transform(target, RotationMatrix(0, 0, 0))

        self.original_target = Transform(target, RotationMatrix(0, 0, 0))
        self.original_transform = transform

        self.fov = fov
        self.aspect = aspect
        self.near = near
        self.far = far

        self.rotation = RotationMatrix(0, 0, 0)

    def get_view_matrix(self):
        return Transform.look_at(self.transform.position(), self.target.position(), self.transform.up())

    def get_projection_matrix(self):
        return Camera.perspective(self.fov, self.aspect, self.near, self.far)

    def get_vp_matrix(self):
        return self.get_projection_matrix() @ self.get_view_matrix()


    @staticmethod
    def perspective(fov, aspect, near, far):
        f = 1 / np.tan(fov / 2)
        q = -far / (far - near)

        return np.array([
            [f / aspect, 0, 0, 0],
            [0, f, 0, 0],
            [0, 0, q, q * near],
            [0, 0, -1, 0]
        ], dtype=np.float32)


class Frustum:
    def __init__(self, view_proj_matrix):
        self.planes = self.extract_planes(view_proj_matrix)

    def extract_planes(self, m):
        planes = []
        m = m.T

        def normalize(plane):
            normal = plane[:3]
            length = np.linalg.norm(normal)
            return plane / length

        # Left
        planes.append(normalize(m[3] + m[0]))
        # Right
        planes.append(normalize(m[3] - m[0]))
        # Bottom
        planes.append(normalize(m[3] + m[1]))
        # Top
        planes.append(normalize(m[3] - m[1]))
        # Near
        planes.append(normalize(m[3] + m[2]))
        # Far
        planes.append(normalize(m[3] - m[2]))

        return planes

    def is_sphere_inside(self, center, radius):
        for plane in self.planes:
            distance = np.dot(plane[:3], center) + plane[3]
            if distance < -radius:
                return False
        return True