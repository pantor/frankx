from argparse import ArgumentParser

from frankx import Affine, JointMotion, LinearRelativeMotion, Measure, MotionData, Reaction, Robot, StopMotion


if __name__ == '__main__':
    parser = ArgumentParser()
    parser.add_argument('--host', default='172.16.0.2', help='FCI IP of the robot')
    args = parser.parse_args()

    # Connect to the robot
    robot = Robot(args.host, repeat_on_error=False)
    robot.set_default_behavior()
    robot.recover_from_errors()

    # Reduce the acceleration and velocity dynamic
    robot.set_dynamic_rel(0.2)

    joint_motion = JointMotion([-1.811944, 1.179108, 1.757100, -2.14162, -1.143369, 1.633046, -0.432171])
    robot.move(joint_motion)

    # Define and move forwards
    motion_down = LinearRelativeMotion(Affine(0.0, 0.0, -0.12), -0.2)
    motion_down_data = MotionData().with_reaction(Reaction(Measure.ForceZ < -5.0, StopMotion(Affine(0.0, 0.0, 0.002), 0.0)))

    # You can try to block the robot now.
    robot.move(motion_down, motion_down_data)

    if motion_down_data.did_break:
        print('Robot stopped at: ', robot.current_pose())
