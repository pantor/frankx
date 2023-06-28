from threading import Thread

from ._franky import GripperInternal


class Gripper(GripperInternal):
    def move_async(self, width) -> Thread:
        p = Thread(target=self.move, args=(width, ), daemon=True)
        p.start()
        return p

    def move_unsafe_async(self, width) -> Thread:
        p = Thread(target=self.move_unsafe, args=(width, ), daemon=True)
        p.start()
        return p
