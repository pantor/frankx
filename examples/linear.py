from argparse import ArgumentParser

from franky import Affine, CartesianMotion, Robot, ReferenceType


if __name__ == "__main__":
    parser = ArgumentParser()
    parser.add_argument("--host", default="172.16.0.2", help="FCI IP of the robot")
    args = parser.parse_args()

    # Connect to the robot
    robot = Robot(args.host)
    robot.relative_dynamics_factor = 0.05
    robot.recover_from_errors()

    # Reduce the acceleration and velocity dynamic
    robot.relative_dynamics_factor = 0.15

    # Define and move forwards
    target = Affine([0.0, 0.2, 0.0])
    motion_forward = CartesianMotion(target, ReferenceType.Relative)
    robot.move(motion_forward)

    # And move backwards using the inverse motion
    motion_backward = CartesianMotion(target.inverse)
    robot.move(motion_backward)
