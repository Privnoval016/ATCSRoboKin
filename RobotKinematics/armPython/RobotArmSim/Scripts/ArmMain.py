import numpy as np

from RobotArmSim.Scripts.CameraController import CameraController
from RobotArmSim.GameEngine3D.Posing.RotationMatrix import RotationMatrix
from RobotArmSim.GameEngine3D.Posing.Vector3 import Vector3
from RobotArmSim.GameEngine3D.Rendering.Color import Color
from RobotArmSim.GameEngine3D.Rendering.Light import Light
from RobotArmSim.GameEngine3D.Rendering.Mesh import Mesh
from RobotArmSim.GameEngine3D.Updating.Updater import Updater
from RobotArmSim.Scripts.RoboticArm import RoboticArm
from RobotArmSim.GameEngine3D.Updating.Engine import Engine


def main():
    engine = Engine(1920)
    print ("Engine started")
    engine.renderer.camera.transform.set_position(Vector3(-20, 6, -2))
    engine.renderer.camera.transform.set_rotation(RotationMatrix(0, 0, 0))
    engine.renderer.camera.target.set_position(Vector3(0, 0, 0))

    camera_controller = CameraController(engine, engine.renderer.camera)

    light = engine.renderer.add_light(
        Light(Vector3(0, 0, 0), RotationMatrix(0, 0, 0), Color(255, 255, 255),
              0))
    light.transform.set_parent(engine.renderer.camera.transform)
    light.transform.set_local_position(Vector3(0, 0, 0))

    sim = RoboticArm(engine, angular_velocity=np.pi / 24, joint_limits=None)


    axis = Updater(engine)
    axis.add_mesh(Mesh.basis(10, axis.transform))
    axis.transform.set_position(Vector3(0, 0, 0))
    axis.transform.set_rotation(RotationMatrix(0, 0, 0))


    engine.run_game_loop()


if __name__ == "__main__":
    main()