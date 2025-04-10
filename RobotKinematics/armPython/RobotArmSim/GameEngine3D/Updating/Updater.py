from RobotArmSim.GameEngine3D.Posing.Transform import Transform

class Updater:
    def __init__(self, engine=None):
        self.engine = engine
        engine.add_updater(self) if engine and self not in engine.updaters else None
        self.transform = Transform()
        self.meshes = []
        pass

    def awake(self):
        pass

    def start(self):
        pass

    def update(self):
        pass

    def add_mesh(self, mesh):
        self.meshes.append(mesh)