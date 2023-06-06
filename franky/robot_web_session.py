import base64
import hashlib
import json
import ssl
from http.client import HTTPSConnection
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from .robot import Robot


class RobotWebSession:
    def __init__(self, robot: "Robot", username: str, password: str):
        self.__robot = robot
        self.__username = username
        self.__password = password

        self.__client = None
        self.__token = None

    @staticmethod
    def __encode_password(user, password):
        bs = ",".join([str(b) for b in hashlib.sha256((password + "#" + user + "@franka").encode("utf-8")).digest()])
        return base64.encodebytes(bs.encode("utf-8")).decode("utf-8")

    def __enter__(self):
        self.client = HTTPSConnection(self.__robot.fci_hostname, timeout=12, context=ssl._create_unverified_context())
        self.client.connect()
        self.client.request(
            "POST", "/admin/api/login",
            body=json.dumps(
                {"login": self.__username, "password": self.__encode_password(self.__username, self.__password)}),
            headers={"content-type": "application/json"}
        )
        self.token = self.client.getresponse().read().decode("utf8")
        return self

    def __exit__(self, type, value, traceback):
        self.client.close()

    def start_task(self, task):
        self.client.request(
            "POST", "/desk/api/execution",
            body=f"id={task}",
            headers={"content-type": "application/x-www-form-urlencoded", "Cookie": f"authorization={self.token}"}
        )
        return self.client.getresponse().read()

    def unlock_brakes(self):
        self.client.request(
            "POST", "/desk/api/robot/open-brakes",
            headers={"content-type": "application/x-www-form-urlencoded", "Cookie": f"authorization={self.token}"}
        )
        return self.client.getresponse().read()

    def lock_brakes(self):
        self.__robot.stop()
        self.client.request(
            "POST", "/desk/api/robot/close-brakes",
            headers={"content-type": "application/x-www-form-urlencoded", "Cookie": f"authorization={self.token}"}
        )
        return self.client.getresponse().read()
