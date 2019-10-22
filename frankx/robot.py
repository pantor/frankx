from threading import Thread

from build._frankx import Robot as _Robot


class Robot(_Robot):
    def move_async(self, width) -> Thread:
        p = Thread(target=self.move, args=(width, ))
        p.start()
        return p