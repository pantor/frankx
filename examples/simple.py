from frankx import Affine, LinearRelativeMotion, Robot


if __name__ == '__main__':
    # Connect to the robot
    robot = Robot('172.16.0.2')
    robot.set_default_behavior()
    robot.recover_from_errors()

    # Reduce the acceleration and velocity dynamic
    robot.set_dynamic_rel(0.2)

    # Define and move forwards
    way = Affine(0.0, 0.2, 0.0)
    motion_forward = LinearRelativeMotion(way)
    robot.move(motion_forward)

    # And move backwards using the inverse motion
    motion_backward = LinearRelativeMotion(way.inverse())
    robot.move(motion_backward)
