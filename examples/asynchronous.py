from argparse import ArgumentParser
import time

from franky import Affine, CartesianMotion, Robot, ReferenceType


if __name__ == "__main__":
    parser = ArgumentParser()
    parser.add_argument("--host", default="172.16.0.2", help="FCI IP of the robot")
    args = parser.parse_args()

    # Connect to the robot
    robot = Robot(args.host)
    robot.relative_dynamics_factor = 0.05

    motion1 = CartesianMotion(Affine([0.2, 0.0, 0.0]), ReferenceType.Relative)
    robot.move(motion1, asynchronous=True)

    time.sleep(0.5)
    motion2 = CartesianMotion(Affine([0.2, 0.0, 0.0]), ReferenceType.Relative)
    robot.move(motion2, asynchronous=True)

    # Wait for the robot to finish its motion
    robot.join_motion()
