from RobotArmSim.GameEngine3D.Posing.Vector3 import Vector3
from RobotArmSim.GameEngine3D.Rendering.Mesh import Mesh
from RobotArmSim.GameEngine3D.Updating.Updater import Updater


class Joint(Updater):
    def __init__(self, engine=None, global_position=None, global_rotation=None, parent_joint=None):
        super().__init__(engine)
        self.parent_joint = parent_joint
        if parent_joint is not None:
            self.transform.set_parent(self.parent_joint.transform)
        self.transform.set_position(global_position)
        self.transform.set_rotation(global_rotation)

        self.init_pos = global_position

    def manual_move(self):
        pass

    def awake(self):
        for child in self.transform.childs:
            self.add_mesh(
                Mesh([Vector3(0, 0, 0), child.local_position()], [(0, 1)],
                     self.transform))

        self.add_mesh(Mesh.sphere(radius=0.5, stacks=16, sectors=32, transform=self.transform, wireframe=True))
        #self.add_mesh(Mesh.cube(Vector3.one() * 0.5, self.transform))
        self.add_mesh(Mesh.basis(3, self.transform))


    def update(self):
        self.calculate_angle()
        self.update_transform()


    def calculate_angle(self):
        pass

    def update_transform(self):
        pass