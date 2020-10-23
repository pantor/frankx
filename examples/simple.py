from frankx import Affine, LinearRelativeMotion, JointMotion, Robot


if __name__ == '__main__':
    # Connect to the robot
    robot = Robot("172.16.0.2")
    robot.set_default_behavior()
    robot.recover_from_errors()

    # Reduce the acceleration and velocity dynamic
    robot.set_dynamic_rel(0.3)

    joint_motion = JointMotion([-1.811944, 1.179108, 1.757100, -2.14162, -1.143369, 1.633046, -0.432171])
    robot.move(joint_motion)

    # Define and move forwards
    way = Affine(0.0, 0.2, 0.0)
    motion_forward = LinearRelativeMotion(way)
    robot.move(motion_forward)

    # And move backwards using the inverse motion
    # motion_backward = LinearRelativeMotion(way.inverse())
    # robot.move(motion_backward)
