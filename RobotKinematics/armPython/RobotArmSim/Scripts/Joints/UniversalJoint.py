import numpy as np
import pygame

from RobotArmSim.GameEngine3D.Posing.RotationMatrix import RotationMatrix
from RobotArmSim.GameEngine3D.Posing.Vector3 import Vector3
from RobotArmSim.GameEngine3D.Posing.Angle import Angle
from RobotArmSim.Scripts.Joints.Joint import Joint


class UniversalJoint(Joint):
    def __init__(self, engine=None, global_position=Vector3(0, 0, 0), global_rotation=RotationMatrix(0, 0, 0), parent=None,
                 axis=None, angular_velocity=None, current_angle=None, limits=None):
        super().__init__(engine, global_position, global_rotation, parent)

        if current_angle is None:
            current_angle = [0, 0]
        if angular_velocity is None:
            angular_velocity = [np.pi / 2, np.pi / 2]

        if len(limits) == 1:
            limits = [limits[0], limits[0]]
        if len(current_angle) == 1:
            current_angle = [current_angle[0], current_angle[0]]
        if len(angular_velocity) == 1:
            angular_velocity = [angular_velocity[0], angular_velocity[0]]
        if len(limits) == 1:
            limits = [limits[0], limits[0]]

        self.axis = axis
        self.current_angle = [Angle(current_angle[0]), Angle(current_angle[1])]
        self.target_angle = [self.current_angle[0], self.current_angle[1]]
        self.ccw = [True, True]
        self.angular_velocity = angular_velocity

        self.limits = limits # angle

        self.delta_angle = [0, 0]


    def calculate_angle(self):
        for i in range(2):
            if self.current_angle[i] != self.target_angle[i]:
                if self.limits[i] is not None and self.limits[0].angle <= self.target_angle[i].angle <= self.limits[1].angle:
                    direction = 1 if self.target_angle[i] > self.current_angle[i] else -1
                    self.delta_angle[i] = self.angular_velocity[i] * self.engine.delta_time * direction
                elif self.limits[i] is None:
                    direction = 1 if self.ccw[i] else -1
                    self.delta_angle[i] = self.angular_velocity[i] * self.engine.delta_time * direction
                else:
                    self.delta_angle[i] = 0
            else:
                self.delta_angle[i] = 0


    def update_transform(self):
        # Update the transform based on the current angle
        for i in range(2):
            if self.delta_angle[i] == 0:
                continue

            self.current_angle[i] += self.delta_angle[i]
            self.transform.rotate_angle_axis(self.delta_angle[i], self.transform.rotation() @ self.axis[i])


    def set_target_angle(self, index, angle, degrees=False):
        if isinstance(angle, (int, float)):
            self.target_angle[index] = Angle(angle) if not degrees else Angle(np.radians(angle))
        elif isinstance(angle, Angle):
            self.target_angle[index] = angle
        else:
            raise TypeError("Unsupported type for angle: {}".format(type(angle)))


    def manual_move(self):
        keys = pygame.key.get_pressed()
        d_theta = [0, 0]
        d_theta[0] = self.angular_velocity[0] * self.engine.delta_time
        d_theta[1] = self.angular_velocity[1] * self.engine.delta_time

        if keys[pygame.K_j]:
            self.ccw[0] = True
            self.set_target_angle(0, self.target_angle[0] + d_theta[0])
        if keys[pygame.K_l]:
            self.ccw[0] = False
            self.set_target_angle(0, self.target_angle[0] - d_theta[0])
        if keys[pygame.K_k]:
            self.set_target_angle(0, self.current_angle[0])


        if keys[pygame.K_u]:
            self.ccw[1] = True
            self.set_target_angle(1, self.target_angle[1] + d_theta[1])
        if keys[pygame.K_o]:
            self.ccw[1] = False
            self.set_target_angle(1, self.target_angle[1] - d_theta[1])
        if keys[pygame.K_i]:
            self.set_target_angle(1, self.current_angle[1])