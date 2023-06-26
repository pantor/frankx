from argparse import ArgumentParser

from franky import Affine, LinearMotion, Robot, ReferenceType


if __name__ == "__main__":
    parser = ArgumentParser()
    parser.add_argument("--host", default="172.16.0.2", help="FCI IP of the robot")
    args = parser.parse_args()

    # Connect to the robot
    robot = Robot(args.host)
    robot.set_dynamic_rel(0.05)
    robot.recover_from_errors()

    # Reduce the acceleration and velocity dynamic
    robot.set_dynamic_rel(0.15)

    # Define and move forwards
    target = Affine([0.0, 0.2, 0.0])
    motion_forward = LinearMotion(target, ReferenceType.Relative)
    robot.move(motion_forward)

    # And move backwards using the inverse motion
    motion_backward = LinearMotion(target.inverse())
    robot.move(motion_backward)
