from argparse import ArgumentParser

from franky import Affine, JointMotion, Measure, Reaction, Robot, CartesianPoseStopMotion, LinearMotion, RobotPose, \
    RobotState, ReferenceType


def reaction_callback(robot_state: RobotState, rel_time: float, abs_time: float):
    print(f"Robot stopped at time {rel_time}.")


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

    joint_motion = JointMotion([-1.811944, 1.179108, 1.757100, -2.14162, -1.143369, 1.633046, -0.432171])
    robot.move(joint_motion)

    # Define and move forwards
    reaction = Reaction(Measure.ForceZ < -5.0, CartesianPoseStopMotion())
    reaction.register_callback(reaction_callback)
    motion_down = LinearMotion(RobotPose(Affine([0.0, 0.0, -0.12]), -0.2), ReferenceType.Relative)
    motion_down.add_reaction(reaction)

    # You can try to block the robot now.
    robot.move(motion_down)
