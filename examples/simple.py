from frankx import Affine, LinearRelativeMotion, Robot

# Connect to the robot
robot = Robot("172.16.0.2")
robot.recover_from_errors()

# Reduce the acceleration and velocity
robot.set_dynamic_rel(0.05)

# Finally define and move forwards
way = Affine(0.0, 0.2, 0.0)
motion_forward = LinearRelativeMotion(way)
robot.move(motion_forward)

# And move backwards using the inverse motion
motion_backward = LinearRelativeMotion(way.inverse())
robot.move(motion_backward)
