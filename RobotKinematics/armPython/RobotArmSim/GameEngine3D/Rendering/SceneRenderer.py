import pygame
import numpy as np
from pygame.locals import *

from RobotArmSim.GameEngine3D.Rendering.Camera import Camera
from RobotArmSim.GameEngine3D.Posing.RotationMatrix import RotationMatrix
from RobotArmSim.GameEngine3D.Posing.Transform import Transform
from RobotArmSim.GameEngine3D.Posing.Vector3 import Vector3
from RobotArmSim.GameEngine3D.Rendering.Color import Color


class SceneRenderer:
    def __init__(self, engine=None, width=800, height=600):
        self.engine = engine
        self.camera = None
        self.aspect = None
        self.screen = None
        self.clock = None
        self.width = width
        self.height = height

    def initialize(self):
        pygame.init()
        self.screen = pygame.display.set_mode((self.width, self.height))
        pygame.display.set_caption("Software 3D Renderer")
        self.clock = pygame.time.Clock()

        self.aspect = self.width / self.height
        self.camera = Camera(transform=Transform(position=Vector3(-140, 35, -7.5), rotation=RotationMatrix(0, 0, 0)),
        target=Vector3(0, 0, 0), aspect=self.aspect)


    def render(self):
        self.engine.delta_time = self.clock.tick(60) / 1000

        for event in pygame.event.get():
            if event.type == QUIT or (event.type == KEYDOWN and event.key == K_ESCAPE):
                self.engine.running = False


    def post_render(self):
        self.screen.fill(Color.BLACK().to_tuple())

        # Get the view-projection matrix
        vp = self.camera.get_vp_matrix()

        for updater in self.engine.updaters:
            for mesh in updater.meshes:
                for edge in mesh.edges:
                    p1 = SceneRenderer.project_to_screen_space(mesh.vertices()[edge[0]], vp, self.width, self.height)
                    p2 = SceneRenderer.project_to_screen_space(mesh.vertices()[edge[1]], vp, self.width, self.height)
                    if p1 and p2:
                        pygame.draw.line(self.screen, mesh.color, p1, p2, 1)

        pygame.display.flip()

    @staticmethod
    def project_to_screen_space(vertex, vp_matrix, screen_width, screen_height):
        v = np.array([*vertex, 1.0],
                     dtype=np.float32)  # Homogeneous coordinates
        projected = vp_matrix @ v  # Apply projection

        if projected[3] == 0:  # Avoid division by zero
            return None

        # Perspective divide
        projected /= projected[3]

        # Map from [-1, 1] to screen space
        x = (projected[0] + 1) * screen_width / 2
        y = (1 - projected[1]) * screen_height / 2
        return int(x), int(y)
