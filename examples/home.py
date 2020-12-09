from argparse import ArgumentParser

from frankx import Affine, JointMotion, LinearMotion, Robot


if __name__ == '__main__':
    parser = ArgumentParser()
    parser.add_argument('--host', default='172.16.0.2', help='FCI IP of the robot')
    args = parser.parse_args()

    # Connect to the robot
    robot = Robot(args.host)
    robot.set_default_behavior()
    robot.recover_from_errors()
    robot.set_dynamic_rel(0.15)

    # Joint motion
    robot.move(JointMotion([-1.811944, 1.179108, 1.757100, -2.14162, -1.143369, 1.633046, -0.432171]))

    # Define and move forwards
    camera_frame = Affine(y=0.05)
    home_pose = Affine(0.480, 0.0, 0.40)

    robot.move(camera_frame, LinearMotion(home_pose, 1.75))
