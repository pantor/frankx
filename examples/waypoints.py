from argparse import ArgumentParser

from franky import Affine, JointWaypointMotion, JointWaypoint, Robot, CartesianWaypointMotion, CartesianWaypoint, \
    ReferenceType, RobotPose

if __name__ == "__main__":
    parser = ArgumentParser()
    parser.add_argument("--host", default="172.16.0.2", help="FCI IP of the robot")
    args = parser.parse_args()

    # Connect to the robot
    robot = Robot(args.host)
    robot.set_dynamic_rel(0.05)
    robot.recover_from_errors()

    # Reduce the acceleration and velocity dynamic
    robot.set_dynamic_rel(0.2)

    joint_motion = JointWaypointMotion([
        JointWaypoint([-1.8, 1.1, 1.7, -2.1, -1.1, 1.6, -0.4]),
        JointWaypoint([-1.7, 1.2, 1.8, -2.0, -1.0, 1.7, -0.3]),
        JointWaypoint([-1.9, 1.0, 1.6, -2.2, -1.2, 1.5, -0.5])
    ])
    robot.move(joint_motion)

    # Define and move forwards
    wp_motion = CartesianWaypointMotion([
        CartesianWaypoint(RobotPose(Affine([0.0, 0.0, -0.12]), -0.2), ReferenceType.Relative),
        CartesianWaypoint(RobotPose(Affine([0.08, 0.0, 0.0]), 0.0), ReferenceType.Relative),
        CartesianWaypoint(RobotPose(Affine([0.0, 0.1, 0.0, 0.0]), 0.0), ReferenceType.Relative),
    ])

    # You can try to block the robot now.
    robot.move(wp_motion)
