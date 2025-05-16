import pygame
from pygame.locals import *

from RobotArmSim.GameEngine3D.Posing.RotationMatrix import RotationMatrix
from RobotArmSim.GameEngine3D.Posing.Transform import Transform
from RobotArmSim.GameEngine3D.Posing.Vector3 import Vector3
from RobotArmSim.GameEngine3D.Updating.Updater import Updater


class CameraController(Updater):

    def __init__(self, engine=None, camera=None):
        super().__init__(engine)
        self.camera = camera


    def start(self):
        pygame.mouse.set_visible(False)
        pygame.event.set_grab(True)


    def update(self):

        keys = pygame.key.get_pressed()
        speed = 5 * self.engine.delta_time

        if keys[K_r]: self.reset()

        if keys[K_w]: self.move("forward", speed)
        if keys[K_s]: self.move("backward", speed)
        if keys[K_a]: self.move("left", speed)
        if keys[K_d]: self.move("right", speed)
        if keys[K_SPACE]: self.move("up", speed)
        if keys[K_LSHIFT]: self.move("down", speed)

        mouse_x, mouse_y = pygame.mouse.get_rel()

        yaw_speed = 0.1 * self.engine.delta_time
        pitch_speed = 0.1 * self.engine.delta_time

        if mouse_x != 0:
            self.camera_rotate(mouse_x * yaw_speed, self.camera.transform.up())
        if mouse_y != 0:
            self.camera_rotate(mouse_y * pitch_speed, self.camera.transform.right())


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
        R = turn.to_rotation(angle)
        T = Transform.calculate_homogeneous_matrix(Vector3.zero(), RotationMatrix(R))
        self.camera.transform.set_homogeneous_matrix(T @ self.camera.transform.homogeneous_matrix)



    def reset(self):
        self.camera.transform.set_transform(self.camera.original_transform)
        self.camera.target.set_transform(self.camera.original_target)