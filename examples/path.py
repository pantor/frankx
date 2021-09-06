from argparse import ArgumentParser

from frankx import Affine, PathMotion, Robot


if __name__ == '__main__':
    parser = ArgumentParser()
    parser.add_argument('--host', default='172.16.0.2', help='FCI IP of the robot')
    args = parser.parse_args()

    # Connect to the robot
    robot = Robot(args.host)
    robot.set_default_behavior()
    robot.recover_from_errors()

    # Reduce the acceleration and velocity dynamic
    robot.set_dynamic_rel(0.15)

    # Define and move forwards
    motion = PathMotion([
        Affine(0.5, 0.0, 0.35),
        Affine(0.5, 0.0, 0.24, -0.3),
        Affine(0.5, -0.2, 0.35),
    ], blend_max_distance=0.05)
    robot.move(motion)
