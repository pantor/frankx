import numpy as np

from frankx import Affine, LinearMotion, Robot


# Connect to the robot
robot = Robot("172.16.0.2")
robot.recover_from_errors()
robot.set_dynamic_rel(0.05)

# Define and move forwards
camera = Affine(-0.079, -0.0005, 0.011, 0.0, 0.0, 0.0)
# camera = Affine(-0.079, -0.0005, 0.011, -np.pi / 4, 0.0, -np.pi)
home = Affine(0.480, -0.025, 0.21, 0.0)


robot.move(camera, LinearMotion(home))
