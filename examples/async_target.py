from argparse import ArgumentParser

from frankx import Affine, Robot, WaypointMotion, Waypoint


if __name__ == '__main__':
    parser = ArgumentParser()
    parser.add_argument('--host', default='172.16.0.2', help='FCI IP of the robot')
    args = parser.parse_args()

    robot = Robot(args.host)
    gripper = robot.get_gripper()

    robot.set_default_behavior()
    robot.recover_from_errors()
    robot.set_dynamic_rel(0.2)


    home_left = Waypoint(Affine(0.480, -0.15, 0.40), 1.65)
    home_right = Waypoint(Affine(0.450, 0.05, 0.35), 1.65)

    motion = WaypointMotion([home_left], return_when_finished=False)
    thread = robot.move_async(motion)

    gripper_thread = gripper.move_unsafe_async(0.05)

    for new_affine, new_width in [(home_right, 0.02), (home_left, 0.07), (home_right, 0.0)]:
        input('Press enter for new affine (also while in motion!)\n')
        motion.set_next_waypoint(new_affine)
        gripper_thread = gripper.move_unsafe_async(new_width)

    input('Press enter to stop\n')
    motion.finish()
    thread.join()
    gripper_thread.join()
