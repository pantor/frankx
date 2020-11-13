from dataclasses import dataclass

from frankx import *


@dataclass
class Grasp:
    """Planar grasp with stroke d"""
    x: float  # [m]
    y: float  # [m]
    z: float  # [m]
    a: float  # [m]
    d: float  # [m]


class Grasping:
    def __init__(self):
        self.robot = Robot('10.224.60.160')
        self.gripper = self.robot.get_gripper()

        self.robot.set_default_behavior()
        self.robot.set_dynamic_rel(0.03)
        self.robot.recover_from_errors()

        joint_motion = JointMotion([-1.811944, 1.179108, 1.757100, -2.14162, -1.143369, 1.633046, -0.432171])
        self.robot.move(joint_motion, MotionData(0.5))

        self.gripper.move(self.gripper.max_width)

        self.robot.move(LinearMotion(self.get_base(0, 0, 0), 1.75), MotionData(0.5))

    @staticmethod
    def get_base(x, y, z, a=0.0, b=0.0, c=0.0):
        return Affine(0.48 + x, -0.204 + y, 0.267 + z, a, b, c)

    def grasp(self, grasp: Grasp):
        data_down = MotionData(0.5).with_reaction(Reaction(Measure.ForceZ < -7.0, LinearRelativeMotion(Affine(0, 0, 0))))
        data_up = MotionData(0.8)

        self.gripper.move_async(grasp.d)

        motion_down = WaypointMotion([
            Waypoint(self.get_base(grasp.x, grasp.y, -0.04, grasp.a), 1.6),
            Waypoint(self.get_base(grasp.x, grasp.y, grasp.z, grasp.a), 1.3),
        ])

        move_success = self.robot.move(motion_down, data_down)

        if not move_success:
            self.robot.recover_from_errors()

        self.gripper.clamp()

        motion_up = WaypointMotion([
            Waypoint(self.get_base(grasp.x, grasp.y, -0.04, grasp.a), 1.6),
            Waypoint(self.get_base(0.0, 0.0, 0.0), 1.75),
        ])
        self.robot.move(motion_up, data_up)

        self.gripper.release()


if __name__ == '__main__':
    grasping = Grasping()
    grasping.grasp(Grasp(x=0.05, y=0.03, z=-0.185, a=0.6, d=0.06))  # [m]
