from argparse import ArgumentParser
from time import sleep

from frankx import Affine, Robot


if __name__ == '__main__':
    parser = ArgumentParser()
    parser.add_argument('--host', default='172.16.0.2', help='FCI IP of the robot')
    args = parser.parse_args()

    robot = Robot(args.host)
    robot.set_default_behavior()

    while True:
        print('Pose: ', robot.current_pose())
        print('Elbow: ', robot.read_once().elbow)
        sleep(0.05)
