import math

from frankx import *


class Grasping:
    def __init__(self):
        self.robot = Robot("172.16.0.2")
        self.gripper = Gripper("172.16.0.2")

        self.robot.velocity_rel = 0.5
        self.robot.acceleration_rel = 0.15
        self.robot.jerk_rel = 0.004

        self.robot.recover_from_errors()

    @staticmethod
    def get_base(x, y, z, a = 0.0, b = 0.0, c = 0.0):
        return Affine(0.48 + x, -0.204 + y, 0.267 + z, a, b, c)

    def init(self):
        joint_motion = JointMotion([-1.8119446, 1.1791089, 1.7571002, -2.141621, -1.1433693, 1.6330460, -0.4321716])
        joint_data = MotionData(0.5)
        self.robot.move(joint_motion, joint_data)

        self.gripper.move(0.08)

    def grasp(self):
        data_down = MotionData(1.0).with_reaction(Reaction(
            Measure.ForceZ, Comparison.Smaller, -7.0,
            LinearRelativeMotion(Affine(0.0, 0.0, 0.001))
        ))

        data_up = MotionData(1.0)

        grasp_x = 0.05
        grasp_y = 0.03
        grasp_z = -0.185
        grasp_a = 0.6

        motion_down = WaypointMotion([
            Waypoint(self.get_base(grasp_x, grasp_y, -0.04, grasp_a), [0.0, 0.0, -0.1, 0.0, 0.0, 0.0, 0.0]),
            Waypoint(self.get_base(grasp_x, grasp_y, grasp_z, grasp_a)),
        ])
        move_success = self.robot.move(motion_down, data_down)

        if not move_success:
            self.robot.recover_from_errors()

        self.gripper.move(0.06)

        motion_up = WaypointMotion([
            Waypoint(self.get_base(grasp_x, grasp_y, -0.04, grasp_a), [0.0, 0.0, 0.1, 0.0, 0.0, 0.0, 0.0]),
            Waypoint(self.get_base(0.0, 0.0, 0.0), 1.75),
        ])
        self.robot.move(motion_up, data_up)


if __name__ == '__main__':
    grasping = Grasping()
    grasping.init()
    grasping.grasp()
