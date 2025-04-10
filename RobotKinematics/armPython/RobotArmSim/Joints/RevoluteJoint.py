import numpy as np

from RobotArmSim.GameEngine3D.Rendering.Mesh import Mesh
from RobotArmSim.GameEngine3D.Posing.RotationMatrix import RotationMatrix
from RobotArmSim.GameEngine3D.Updating.Updater import Updater
from RobotArmSim.GameEngine3D.Posing.Vector3 import Vector3
from RobotArmSim.GameEngine3D.Posing.Angle import Angle


class RevoluteJoint(Updater):
    def __init__(self, engine=None, global_position=Vector3(0, 0, 0), global_rotation=RotationMatrix(0, 0, 0), parent=None,
                 axis=Vector3.forward(), angular_velocity=np.pi/2, current_angle=0, limits=None):
        super().__init__(engine)

        self.transform.set_parent(parent)
        self.transform.set_position(global_position)
        self.transform.set_rotation(global_rotation)

        self.axis = axis
        self.current_angle = Angle(current_angle)
        self.target_angle = Angle(current_angle)
        self.ccw = True
        self.angular_velocity = angular_velocity

        self.limits = limits # angle

        self.delta_angle = 0


    def awake(self):
        for child in self.transform.childs:
            self.add_mesh(
                Mesh([Vector3(0, 0, 0), child.local_position()], [(0, 1)],
                     self.transform))

        self.add_mesh(Mesh.sphere(radius=1, stacks=5, sectors=5, transform=self.transform))
        self.add_mesh(Mesh.basis(1, self.transform))


    def update(self):
        self.calculate_angle()
        self.update_transform()


    def calculate_angle(self):
        if self.current_angle != self.target_angle:
            if self.limits is not None and self.limits[0].angle <= self.target_angle.angle <= self.limits[1].angle:
                direction = 1 if self.target_angle > self.current_angle else -1
                self.delta_angle = self.angular_velocity * self.engine.delta_time * direction
            elif self.limits is None:
                direction = 1 if self.ccw else -1
                self.delta_angle = self.angular_velocity * self.engine.delta_time * direction
            else:
                # If the target angle is outside the limits, do not update
                self.delta_angle = 0
        else:
            self.delta_angle = 0


    def update_transform(self):
        # Update the transform based on the current angle
        if self.delta_angle == 0:
            return

        self.current_angle += self.delta_angle
        self.transform.rotate_angle_axis(self.delta_angle, self.transform.rotation() @ self.axis)


    def set_target_angle(self, angle, degrees=False):
        if isinstance(angle, (int, float)):
            self.target_angle = Angle(angle) if not degrees else Angle(np.radians(angle))
        elif isinstance(angle, Angle):
            self.target_angle = angle
        else:
            raise TypeError("Unsupported type for angle: {}".format(type(angle)))