import time

import numpy as np

from frankx import Affine, Robot


if __name__ == '__main__':
    robot = Robot('10.224.60.160')
    robot.set_default_behavior()

    while True:
        print(robot.current_pose())
        print(robot.read_once().elbow)
        time.sleep(0.05)
