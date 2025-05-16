import numpy as np
import pygame

from RobotArmSim.GameEngine3D.Posing.RotationMatrix import RotationMatrix
from RobotArmSim.GameEngine3D.Posing.Vector3 import Vector3
from RobotArmSim.GameEngine3D.Posing.Angle import Angle
from RobotArmSim.Scripts.Joints.Joint import Joint


class RevoluteJoint(Joint):
    def __init__(self, engine=None, global_position=Vector3(0, 0, 0), global_rotation=RotationMatrix(0, 0, 0), parent_joint=None,
                 axis=Vector3.forward(), angular_velocity=np.pi/2, current_angle=0, limits=None):
        super().__init__(engine, global_position, global_rotation, parent_joint)

        self.axis = axis
        self.global_axis = self.transform.rotation() @ self.axis
        self.current_angle = Angle(current_angle)
        self.target_angle = Angle(current_angle)
        self.ccw = True
        self.angular_velocity = angular_velocity

        self.limits = limits # angle

        self.delta_angle = 0


    def calculate_angle(self):
        if np.abs(self.current_angle.angle - self.target_angle.angle) >= self.angular_velocity:
            if self.limits is not None and self.limits[0].angle - 2*np.pi < self.target_angle.angle < self.limits[1].angle:

                direction = 1 if self.ccw else -1
                self.delta_angle = self.angular_velocity * self.engine.delta_time * direction

                if not self.ccw:
                    if self.current_angle.angle + self.delta_angle > self.limits[1].angle:
                        self.delta_angle = self.limits[1].angle - self.current_angle.angle
                else:
                    if self.current_angle.angle + self.delta_angle < self.limits[0].angle - 2*np.pi:
                        self.delta_angle = self.limits[0].angle - 2*np.pi - self.current_angle.angle

            elif self.limits is None:
                direction = 1 if self.ccw else -1
                self.delta_angle = self.angular_velocity * self.engine.delta_time * direction

            else:
                self.delta_angle = 0
        else:
            self.delta_angle = 0


    def update_transform(self):
        # Update the transform based on the current angle
        if self.delta_angle == 0:
            return

        self.current_angle += self.delta_angle
        self.transform.rotate_angle_axis(self.delta_angle, self.transform.rotation() @ self.axis)

    def update(self):
        diff = self.target_angle.angle - self.current_angle.angle
        self.current_angle.angle = self.current_angle.angle + diff

        self.transform.rotate_angle_axis(diff, self.transform.rotation() @ self.axis)

    def set_target_angle(self, angle, degrees=False):
        if isinstance(angle, (int, float)):
            self.target_angle = Angle(angle) if not degrees else Angle(np.radians(angle))
        elif isinstance(angle, Angle):
            self.target_angle = angle
        else:
            raise TypeError("Unsupported type for angle: {}".format(type(angle)))


    def manual_move(self):
        keys = pygame.key.get_pressed()
        d_theta = self.angular_velocity * self.engine.delta_time

        if keys[pygame.K_j]:
            self.ccw = True
            self.set_target_angle(self.target_angle + d_theta)
        if keys[pygame.K_l]:
            self.ccw = False
            self.set_target_angle(self.target_angle - d_theta)
        if keys[pygame.K_k]:
            self.set_target_angle(self.current_angle)



