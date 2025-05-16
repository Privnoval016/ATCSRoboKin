import numpy as np

from RobotArmSim.GameEngine3D.Posing.Transform import Transform


class Light:
    def __init__(self, position, rotation, color, intensity):
        self.transform = Transform(position, rotation)
        self.color = color
        self.intensity = intensity


    def data(self):
        return np.array([self.transform.position().vector,
            self.transform.rotation().matrix,
            self.color.val(),
            self.intensity])