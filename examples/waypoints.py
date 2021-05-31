from argparse import ArgumentParser

from frankx import Affine, JointMotion, Robot, Waypoint, WaypointMotion


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
    motion_down = WaypointMotion([
        Waypoint(Affine(0.0, 0.0, -0.12), -0.2, Waypoint.Relative),
        Waypoint(Affine(0.08, 0.0, 0.0), 0.0, Waypoint.Relative),
        Waypoint(Affine(0.0, 0.1, 0.0, 0.0), 0.0, Waypoint.Relative),
    ])

    # You can try to block the robot now.
    robot.move(motion_down)
