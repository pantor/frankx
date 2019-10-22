from build._frankx import Robot as OldRobot

class Robot(OldRobot):
    def move_async() -> bool:
        print('a')
        return True