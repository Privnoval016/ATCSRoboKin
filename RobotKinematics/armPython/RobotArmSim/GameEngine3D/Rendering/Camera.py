import numpy as np
import pygame
from pygame.locals import *

from RobotArmSim.GameEngine3D.Posing.RotationMatrix import RotationMatrix
from RobotArmSim.GameEngine3D.Posing.Transform import Transform
from RobotArmSim.GameEngine3D.Updating.Updater import Updater
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
        return Transform.perspective(self.fov, self.aspect, self.near, self.far)

    def get_vp_matrix(self):
        return self.get_projection_matrix() @ self.get_view_matrix()

    def get_frustum_planes(self):
        vp_matrix = self.get_vp_matrix()
        planes = Camera.extract_frustum_planes(vp_matrix)
        return planes

    @staticmethod
    def extract_frustum_planes(vp_matrix):
        planes = []
        planes.append(vp_matrix[3] + vp_matrix[0])  # Left
        planes.append(vp_matrix[3] - vp_matrix[0])  # Right
        planes.append(vp_matrix[3] + vp_matrix[1])  # Bottom
        planes.append(vp_matrix[3] - vp_matrix[1])  # Top
        planes.append(vp_matrix[3] + vp_matrix[2])  # Near
        planes.append(vp_matrix[3] - vp_matrix[2])  # Far

        normalized = []
        for plane in planes:
            norm = np.linalg.norm(plane[:3])
            normalized.append(plane / norm)
        return normalized


class CameraController(Updater):

    def __init__(self, engine=None, camera=None):
        super().__init__(engine)
        self.camera = camera


    def update(self):

        print("--------------")
        print("CameraPosition: ", self.camera.transform.position())
        print(self.camera.transform.rotation())

        keys = pygame.key.get_pressed()
        speed = 5 * self.engine.delta_time

        if keys[K_r]: self.reset()

        if keys[K_w]: self.move("forward", speed)
        if keys[K_s]: self.move("backward", speed)
        if keys[K_a]: self.move("left", speed)
        if keys[K_d]: self.move("right", speed)
        if keys[K_SPACE]: self.move("up", speed)
        if keys[K_LSHIFT]: self.move("down", speed)

        # Rotation with arrow keys
        if keys[K_UP]:
            self.camera_rotate(speed, self.camera.transform.right())
        if keys[K_DOWN]:
            self.camera_rotate(-speed, self.camera.transform.right())
        if keys[K_LEFT]:
            self.camera_rotate(speed, self.camera.transform.forward())
        if keys[K_RIGHT]:
            self.camera_rotate(-speed, self.camera.transform.forward())


    def move(self, direction, amount):
        forward = (self.camera.target.position() - self.camera.transform.position()).normalized()
        right = forward.cross(self.camera.transform.up()).normalized()
        up = self.camera.transform.up()

        if direction == "forward":
            self.camera.transform.set_position(self.camera.transform.position() + forward * amount)
            self.camera.target.set_position(self.camera.target.position() + forward * amount)
        elif direction == "backward":
            self.camera.transform.set_position(self.camera.transform.position() - forward * amount)
            self.camera.target.set_position(self.camera.target.position() - forward * amount)
        elif direction == "right":
            self.camera.transform.set_position(self.camera.transform.position() - right * amount)
            self.camera.target.set_position(self.camera.target.position() - right * amount)
        elif direction == "left":
            self.camera.transform.set_position(self.camera.transform.position() + right * amount)
            self.camera.target.set_position(self.camera.target.position() + right * amount)
        elif direction == "up":
            self.camera.transform.set_position(self.camera.transform.position() + up * amount)
            self.camera.target.set_position(self.camera.target.position() + up * amount)
        elif direction == "down":
            self.camera.transform.set_position(self.camera.transform.position() - up * amount)
            self.camera.target.set_position(self.camera.target.position() - up * amount)


    def camera_rotate(self, angle, axis):
        turn = axis.normalized()
        R = turn.skew_symmetric(angle)
        T = Transform.calculate_homogeneous_matrix(Vector3.zero(), RotationMatrix(R))
        self.camera.transform.set_homogeneous_matrix(T @ self.camera.transform.homogeneous_matrix)



    def reset(self):
        self.camera.transform.set_transform(self.camera.original_transform)
        self.camera.target.set_transform(self.camera.original_target)