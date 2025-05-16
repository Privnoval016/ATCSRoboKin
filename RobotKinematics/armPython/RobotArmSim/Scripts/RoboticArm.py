import numpy as np
import pygame

from RobotArmSim.GameEngine3D.InverseKinematics import InverseKinematics
from RobotArmSim.GameEngine3D.Posing.Angle import Angle
from RobotArmSim.GameEngine3D.Posing.Transform import Transform
from RobotArmSim.GameEngine3D.Rendering.Color import Color
from RobotArmSim.GameEngine3D.Posing.RotationMatrix import RotationMatrix
from RobotArmSim.GameEngine3D.Updating.Updater import Updater
from RobotArmSim.GameEngine3D.Posing.Vector3 import Vector3
from RobotArmSim.Scripts.Joints.Gripper import Gripper
from RobotArmSim.Scripts.Joints.RevoluteJoint import RevoluteJoint


class RoboticArm(Updater):
    def __init__(self, engine=None, angular_velocity=1, joint_limits=None):
        super().__init__(engine)
        self.gripper = None
        self.colors = None
        self.joints = []
        self.angular_velocity = angular_velocity
        self.joint_limits = (Angle(joint_limits[0], True), Angle(joint_limits[1], True)) if joint_limits is not None else None

        self.selected_joint = 0


    def awake(self):

        self.joints.append(RevoluteJoint(self.engine, global_position=Vector3(0, 0, 0),
                                         global_rotation=RotationMatrix(0, 0, 0), parent_joint=None,
                                         axis=Vector3.up(), angular_velocity=self.angular_velocity,
                                         current_angle=0, limits=self.joint_limits))

        self.joints.append(RevoluteJoint(self.engine, global_position=Vector3(0, 8, 0),
                                         global_rotation=RotationMatrix(0, 0, 0), parent_joint=self.joints[0],
                                         axis=Vector3.right(), angular_velocity=self.angular_velocity,
                                         current_angle=0, limits=self.joint_limits))

        self.joints.append(RevoluteJoint(self.engine, global_position=Vector3(0, 18.5, 0),
                                         global_rotation=RotationMatrix(0, 0, 0), parent_joint=self.joints[1],
                                         axis=Vector3.right(), angular_velocity=self.angular_velocity,
                                         current_angle=0, limits=self.joint_limits))

        self.joints.append(RevoluteJoint(self.engine, global_position=Vector3(0, 18.5, 6.0),
                                         global_rotation=RotationMatrix(0, 0, 0),
                                         parent_joint=self.joints[2],
                                         axis=Vector3.forward(),
                                         angular_velocity=self.angular_velocity,
                                         current_angle=0, limits=self.joint_limits))

        self.gripper = Gripper(self.engine, global_position=Vector3(0, 18.5, 16),
                               global_rotation=RotationMatrix(0, 0, 0),
                               parent_joint=self.joints[3])


    def set_colors(self):
        self.joints[0].meshes[1].set_color(self.colors[0])
        self.joints[1].meshes[1].set_color(self.colors[1])
        self.joints[2].meshes[1].set_color(self.colors[2])
        self.joints[3].meshes[1].set_color(self.colors[3])

        self.joints[self.selected_joint].meshes[1].set_color(Color.WHITE())


    def start(self):
        self.colors = [Color.RED(), Color.GREEN(), Color.BLUE(), Color.YELLOW()]
        self.set_colors()

        fk, jacobian = self.gripper.forward_kinematics()
        theta = InverseKinematics.calculate_ik(4, fk, jacobian,
                                       Transform(Vector3(0, 15, 12.1),
                                                 RotationMatrix(0, 0, 0, True)), 10)

        print(f"Final Joint Angles (in degrees): {self.rads_to_joint_ranges(theta)}")
        output_transform = Transform()
        output_transform.set_homogeneous_matrix(fk(theta))
        print("Final Gripper Position: ", output_transform.position())
        print("Final Gripper Rotation: ", output_transform.rotation().euler_angles)

        for i in range(0, 4):
            self.joints[i].ccw = False
            self.joints[i].set_target_angle(theta[i], degrees=False)


    def update(self):
        self.check_joint_input()

    def check_joint_input(self):
        keys = pygame.key.get_pressed()

        for i in range(0, 9):
            if keys[pygame.K_0 + i] and i < len(self.joints):
                self.selected_joint = i
                break

        if keys[pygame.K_BACKQUOTE]: self.selected_joint = 0

        if keys[pygame.K_p]:
            print("-----------------------")
            print(f"Gripper Position: {self.gripper.transform.position()}")
            print(f"Gripper Rotation: {self.gripper.transform.rotation()}")
            theta = np.zeros(len(self.joints))
            for i in range(len(self.joints)):
                theta[i] = self.joints[i].current_angle.angle

            print(f"Joint Angles (in degrees): {self.rads_to_joint_ranges(theta):5f}")

        self.set_colors()

        self.joints[self.selected_joint].manual_move()


    @staticmethod
    def rads_to_joint_ranges(angles):
        degree_angles = np.degrees(np.copy(angles))
        for i in range(len(degree_angles)):
            if 270 < degree_angles[i] < 360:
                degree_angles[i] = degree_angles[i] - 360

        return degree_angles


