from _franky import Robot as _Robot

from .robot_web_session import RobotWebSession


class Robot(_Robot):
    def create_web_session(self, username: str, password: str):
        return RobotWebSession(self, username, password)
