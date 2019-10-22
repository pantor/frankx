from build._frankx import Gripper as OldGripper

class Gripper(OldGripper):
    def move_async() -> bool:
        return True