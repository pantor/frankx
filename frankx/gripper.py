from threading import Thread

from _frankx import Gripper as _Gripper


class Gripper(_Gripper):
    def move_async(self, width) -> Thread:
        p = Thread(target=self.move, args=(width, ))
        p.start()
        return p
