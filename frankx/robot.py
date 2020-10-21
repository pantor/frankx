from threading import Thread

from build._frankx import Robot as _Robot
from build._frankx import Gripper as _Gripper


class Robot(_Robot):
    def get_gripper(self):
        return _Gripper(self.fci_ip)

    def move_async(self, width) -> Thread:
        p = Thread(target=self.move, args=(width, ))
        p.start()
        return p