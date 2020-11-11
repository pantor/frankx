import numpy as np

from frankx import Affine, LinearMotion, Robot


# Connect to the robot
robot = Robot('172.16.0.2')
robot.recover_from_errors()
robot.set_dynamic_rel(0.0015)

# Define and move forwards
camera = Affine(-0.0005, 0.079, 0.011, 0.0, 0.0, 0.0)
gripper = Affine(0, 0, -0.18, 0.0, 0.0, 0.0)
home = Affine(0.480, -0.15, 0.40, 0.0)


robot.move(camera, LinearMotion(home, 1.75))

print(robot.current_pose(camera))
