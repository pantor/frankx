from ._franky import RobotInternal

from .robot_web_session import RobotWebSession


class Robot(RobotInternal):
    def create_web_session(self, username: str, password: str):
        return RobotWebSession(self, username, password)
