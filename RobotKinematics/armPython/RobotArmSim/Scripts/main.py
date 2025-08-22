from RobotArmSim.Scripts.CameraController import CameraController
from RobotArmSim.GameEngine3D.Posing.RotationMatrix import RotationMatrix
from RobotArmSim.GameEngine3D.Posing.Vector3 import Vector3
from RobotArmSim.GameEngine3D.Rendering.Color import Color
from RobotArmSim.GameEngine3D.Rendering.Light import Light
from RobotArmSim.GameEngine3D.Rendering.Mesh import Mesh
from RobotArmSim.GameEngine3D.Updating.Updater import Updater
from RobotArmSim.GameEngine3D.Updating.Engine import Engine



def main():
    engine = Engine()
    print ("Engine started")
    engine.renderer.camera.transform.set_position(Vector3(5, 5, 5))
    engine.renderer.camera.transform.set_rotation(RotationMatrix(0, 0, 0))
    engine.renderer.camera.target.set_position(Vector3(0, 0, 0))

    light = engine.renderer.add_light(Light(Vector3(0, 0, 0), RotationMatrix(0, 0, 0), Color(255, 255, 255), 1.0))
    light.transform.set_parent(engine.renderer.camera.transform)
    light.transform.set_local_position(Vector3(0, 0, 0))

    camera_controller = CameraController(engine, engine.renderer.camera)

    among = Updater(engine)
    among.add_mesh(Mesh.obj_wireframe("/Users/pranavsukesh/Documents/GitHub/ATCSRoboKin/RobotKinematics/armPython/RobotArmSim/Assets/rick.obj",
                                      among.transform, wireframe=False))

    engine.run_game_loop()


if __name__ == "__main__":
    main()