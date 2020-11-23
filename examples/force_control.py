from argparse import ArgumentParser

from time import sleep
from frankx import Affine, JointMotion, LinearRelativeMotion, ImpedanceMotion, Robot, Measure, MotionData, Reaction, StopMotion


if __name__ == '__main__':
    parser = ArgumentParser()
    parser.add_argument('--host', default='172.16.0.2', help='FCI IP of the robot')
    args = parser.parse_args()

    robot = Robot(args.host)
    robot.set_default_behavior()
    robot.recover_from_errors()

    # Reduce the acceleration and velocity dynamic
    robot.set_dynamic_rel(0.2)

    joint_motion = JointMotion([-1.811944, 1.179108, 1.757100, -2.14162, -1.143369, 1.633046, -0.432171])
    robot.move(joint_motion)

    linear_motion = LinearRelativeMotion(Affine(z=-0.19), -0.3)
    linear_motion_data = MotionData().with_reaction(Reaction(Measure.ForceZ < -8.0, StopMotion()))  # [N]
    robot.move(linear_motion, linear_motion_data)

    # Define and move forwards
    impedance_motion = ImpedanceMotion(800.0, 80.0)

    # BE CAREFUL HERE!
    # Move forward in y-direction while applying a force in z direction
    impedance_motion.add_force_constraint(z=-8.0)  # [N]
    impedance_motion.set_linear_relative_target_motion(Affine(y=0.1), 2.0)  # [m], [s]
    robot.move(impedance_motion)
