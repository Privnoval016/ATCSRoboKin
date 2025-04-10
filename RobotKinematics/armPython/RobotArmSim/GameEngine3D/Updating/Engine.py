import pygame

from RobotArmSim.GameEngine3D.Rendering.SceneRenderer import SceneRenderer


class Engine:
    def __init__(self):
        self.delta_time = 0  # Time step for simulation
        self.running = False
        self.updaters = []
        self.last_frame_updaters = []

        self.renderer = SceneRenderer(self, width=800, height=600)
        self.renderer.initialize()


    def add_updater(self, updater):
        # Add an updater to the world
        self.updaters.append(updater)


    def start(self):
        for updater in self.updaters:
            updater.awake()

        for updater in self.updaters:
            updater.start()

        self.last_frame_updaters = self.updaters.copy()


    def pause(self, pause=True):
        self.running = not pause


    def update(self):
        self.renderer.render()

        true_updaters = self.start_new_subscribers()

        for updater in true_updaters:
            updater.update()

        self.renderer.post_render()
        self.last_frame_updaters = self.updaters.copy()


    def start_new_subscribers(self):
        true_updaters = []
        new_updaters = []

        for updater in self.updaters:
            if updater not in self.last_frame_updaters:
                new_updaters.append(updater)
            else:
                true_updaters.append(updater)

        for updater in new_updaters:
            updater.awake()

        for updater in new_updaters:
            updater.start()

        return true_updaters



    def run_game_loop(self):
        self.start()

        while self.running:
            self.update()

        pygame.quit()