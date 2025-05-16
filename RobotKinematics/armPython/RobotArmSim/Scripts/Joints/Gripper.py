import numpy as np
import scipy.linalg

from RobotArmSim.GameEngine3D.Posing.Transform import Transform
from RobotArmSim.GameEngine3D.Posing.Vector3 import Vector3
from RobotArmSim.GameEngine3D.Rendering.Mesh import Mesh
from RobotArmSim.GameEngine3D.Updating.Updater import Updater


class Gripper(Updater):
    def __init__(self, engine=None, global_position=None, global_rotation=None,
                 parent_joint=None):
        super().__init__(engine)
        self.parent_joint = parent_joint
        if parent_joint is not None:
            self.transform.set_parent(self.parent_joint.transform)
        self.transform.set_position(global_position)
        self.transform.set_rotation(global_rotation)

    def awake(self):
        # Add meshes for the gripper
        self.add_mesh(Mesh.cube(Vector3.one() * 0.5, self.transform))
        self.add_mesh(Mesh.basis(3, self.transform))

    def forward_kinematics(self):

        cur_joint = self.parent_joint
        space_screws = []

        while cur_joint is not None:
            space_screws.append(
                Transform.screw(cur_joint.global_axis, cur_joint.init_pos))
            cur_joint = cur_joint.parent_joint

        space_screws = space_screws[::-1]


        def f(theta):

            T = np.eye(4)
            for i, screw in enumerate(space_screws):
                T = T @ scipy.linalg.expm(screw * theta[i])

            T = T @ self.transform.homogeneous_matrix

            return T


        def space_jacobian(theta):
            T_final = np.empty((len(theta), 6))

            T_final[0] = Transform.twist_to_six_vector(space_screws[0])

            for i in range(1, len(theta)):
                temp = np.eye(4)
                for j in range(0, i):
                    temp = temp @ scipy.linalg.expm(space_screws[j] * theta[j])

                T_final[i] = Transform.adjoint_map(temp) @ Transform.twist_to_six_vector(space_screws[i])

            return np.transpose(T_final)

        return f, space_jacobian

