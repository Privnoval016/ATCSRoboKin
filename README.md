# ATCSRoboKin

## CarPython
Given a 4-wheeled mecanum car controlled by a Raspberry Pi, I implemented a path generation algorithm that takes instructions from a control file about target positions and rotations and converts them into motor power levels, driving the mecanum car along the delineated path. Currently broken, but working on fixes.

## ArmPython
Given a 4-axis robotic arm controlled by a Raspberry Pi, I implemented a simulation in Python, creating a rendering engine to visualize 3D space, utilizing affine transformations and homogeneous coordinates, with additional support for real time update engines. I also implemented an inverse kinematics algorithm to determine the joint angles required to position the end effector to a given pose. All mathematical formulations is documented in the .PDF write-up (work in progress), located in the root directory.

To run:
Clone the repository and run
```
python3 RobotKinematics/armPython/main.py [filepath]
```
to render a 3D model specified by the filepath. If none is specified, it will open a certain model from the assets folder.
