import pygame

from RobotArmSim.GameEngine3D.Rendering.Color import Color
from RobotArmSim.GameEngine3D.Posing.RotationMatrix import RotationMatrix
from RobotArmSim.GameEngine3D.Updating.Updater import Updater
from RobotArmSim.GameEngine3D.Posing.Vector3 import Vector3
from RobotArmSim.Joints.RevoluteJoint import RevoluteJoint


class RoboticArm(Updater):
    def __init__(self, engine=None, angular_velocity=1, joint_limits=None):
        super().__init__(engine)
        self.joints = []
        self.angular_velocity = angular_velocity
        self.joint_limits = joint_limits

        self.selected_joint = 0


    def awake(self):

        self.joints.append(RevoluteJoint(self.engine, global_position=Vector3(0, 0, 0),
                                         global_rotation=RotationMatrix(0, 0, 0), parent=None,
                                         axis=Vector3.right(), angular_velocity=self.angular_velocity,
                                         current_angle=0, limits=self.joint_limits))

        self.joints.append(RevoluteJoint(self.engine, global_position=Vector3(0, 0, 5),
                                         global_rotation=RotationMatrix(0, 0, 0), parent=self.joints[0].transform,
                                         axis=Vector3.forward(), angular_velocity=self.angular_velocity,
                                         current_angle=0, limits=self.joint_limits))

        self.joints.append(RevoluteJoint(self.engine, global_position=Vector3(0, 0, 10),
                                         global_rotation=RotationMatrix(0, 0, 0), parent=self.joints[1].transform,
                                         axis=Vector3.up(), angular_velocity=self.angular_velocity,
                                         current_angle=0, limits=self.joint_limits))

        self.joints.append(RevoluteJoint(self.engine, global_position=Vector3(0, 0, 17),
                                         global_rotation=RotationMatrix(0, 0, 0),
                                         parent=self.joints[2].transform,
                                         axis=Vector3.right(),
                                         angular_velocity=self.angular_velocity,
                                         current_angle=0, limits=self.joint_limits))


    def start(self):
        colors = [Color.RED(), Color.GREEN(), Color.BLUE()]

        self.joints[0].meshes[0].set_color(colors[0])
        self.joints[1].meshes[0].set_color(colors[1])
        self.joints[2].meshes[0].set_color(colors[2])


    def update(self):
        self.check_joint_input()


    def check_joint_input(self):
        d_theta = self.angular_velocity * self.engine.delta_time

        keys = pygame.key.get_pressed()

        # set the selected joint to the last number key pressed

        for i in range(0, 9):
            if keys[pygame.K_0 + i] and i < len(self.joints):
                self.selected_joint = i
                break


        if keys[pygame.K_j]:
            self.joints[self.selected_joint].ccw = True
            self.joints[self.selected_joint].set_target_angle(
                self.joints[self.selected_joint].target_angle + d_theta)
        if keys[pygame.K_l]:
            self.joints[self.selected_joint].ccw = False
            self.joints[self.selected_joint].set_target_angle(
                self.joints[self.selected_joint].target_angle - d_theta)
        if keys[pygame.K_k]:
            self.joints[self.selected_joint].set_target_angle(
                self.joints[self.selected_joint].current_angle)


