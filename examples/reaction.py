from frankx import Affine, JointMotion, LinearRelativeMotion, Measure, MotionData, Reaction, Robot


if __name__ == '__main__':
    # Connect to the robot
    robot = Robot("172.16.0.2")
    robot.set_default_behavior()
    robot.recover_from_errors()

    # Reduce the acceleration and velocity dynamic
    robot.set_dynamic_rel(0.2)

    joint_motion = JointMotion([-1.811944, 1.179108, 1.757100, -2.14162, -1.143369, 1.633046, -0.432171])
    robot.move(joint_motion)

    # Define and move forwards
    motion_down = LinearRelativeMotion(Affine(0.0, 0.0, -0.11), -0.2)
    motion_down_data = MotionData().with_reaction(Reaction(Measure.ForceZ < -5.0, LinearRelativeMotion(Affine(0.0, 0.0, 0.002), 0.0, 2.0)))

    robot.move(motion_down, motion_down_data)
