import numpy as np

from RobotArmSim.RoboticArm import RoboticArm
from RobotArmSim.GameEngine3D.Updating.Engine import Engine
from RobotArmSim.GameEngine3D.Rendering.Camera import CameraController


def main():
    engine = Engine()
    print ("Engine started")
    sim = RoboticArm(engine, angular_velocity=np.pi / 2, joint_limits=None)

    camera_controller = CameraController(engine, engine.renderer.camera)

    engine.running = True
    engine.run_game_loop()


if __name__ == "__main__":
    main()