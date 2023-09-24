import base64
import hashlib
import json
import ssl
import time
from http.client import HTTPSConnection, HTTPResponse
from typing import TYPE_CHECKING, Dict, Optional, Any, Literal
from urllib.error import HTTPError

if TYPE_CHECKING:
    from .robot import Robot


class RobotWebSession:
    def __init__(self, robot: "Robot", username: str, password: str):
        self.__robot = robot
        self.__username = username
        self.__password = password

        self.__client = None
        self.__token = None
        self.__control_token = None
        self.__control_token_id = None

    @staticmethod
    def __encode_password(user: str, password: str) -> str:
        bs = ",".join([str(b) for b in hashlib.sha256((password + "#" + user + "@franka").encode("utf-8")).digest()])
        return base64.encodebytes(bs.encode("utf-8")).decode("utf-8")

    def send_api_request(self, target: str, headers: Optional[Dict[str, str]] = None, body: Optional[Any] = None,
                         method: Literal["GET", "POST", "DELETE"] = "POST"):
        _headers = {
            "Cookie": f"authorization={self.__token}"
        }
        if headers is not None:
            _headers.update(headers)
        self.__client.request(method, target, headers=_headers, body=body)
        res: HTTPResponse = self.__client.getresponse()
        if res.getcode() != 200:
            raise HTTPError(target, res.getcode(), res.reason, res.headers, res.fp)
        return res.read()

    def send_control_api_request(self, target: str, headers: Optional[Dict[str, str]] = None,
                                 body: Optional[Any] = None,
                                 method: Literal["GET", "POST", "DELETE"] = "POST"):
        if self.__control_token is None:
            raise ValueError("Client does not have control. Call take_control() first.")
        _headers = {
            "X-Control-Token": self.__control_token
        }
        _headers.update(headers)
        return self.send_api_request(target, headers=_headers, method=method, body=body)

    def __enter__(self):
        self.__client = HTTPSConnection(self.__robot.fci_hostname, timeout=12, context=ssl._create_unverified_context())
        self.__client.connect()
        payload = json.dumps(
            {"login": self.__username, "password": self.__encode_password(self.__username, self.__password)})
        self.__token = self.send_api_request(
            "/admin/api/login", headers={"content-type": "application/json"},
            body=payload).decode("utf-8")
        return self

    def __exit__(self, type, value, traceback):
        if self.__control_token is not None:
            self.release_control()
        self.__token = None
        self.__client.close()

    def take_control(self, wait_timeout: float = 10.0):
        if self.__control_token is None:
            res = self.send_api_request(
                "/admin/api/control-token/request", headers={"content-type": "application/json"},
                body=json.dumps({"requestedBy": self.__username}))
            response_dict = json.loads(res)
            self.__control_token = response_dict["token"]
            self.__control_token_id = response_dict["id"]
            # One should probably use websockets here but that would introduce another dependency
            start = time.time()
            while time.time() - start < wait_timeout and not self.has_control():
                time.sleep(1.0)

    def release_control(self):
        if self.__control_token is not None:
            self.send_control_api_request(
                "/admin/api/control-token", headers={"content-type": "application/json"}, method="DELETE",
                body=json.dumps({"token": self.__control_token}))
            self.__control_token = None
            self.__control_token_id = None

    def has_control(self):
        if self.__control_token_id is not None:
            status = self.get_system_status()
            return status["controlToken"]["activeToken"]["id"] == self.__control_token_id

    def start_task(self, task: str):
        self.send_api_request(
            "/desk/api/execution", headers={"content-type": "application/x-www-form-urlencoded"},
            body=f"id={task}")

    def unlock_brakes(self):
        self.send_control_api_request(
            "/desk/api/joints/unlock", headers={"content-type": "application/x-www-form-urlencoded"})

    def lock_brakes(self):
        self.__robot.stop()
        self.send_control_api_request(
            "/desk/api/joints/lock", headers={"content-type": "application/x-www-form-urlencoded"})

    def set_mode_programming(self):
        self.send_control_api_request(
            "/desk/api/operating-mode/programming", headers={"content-type": "application/x-www-form-urlencoded"})

    def set_mode_execution(self):
        self.send_control_api_request(
            "/desk/api/operating-mode/execution", headers={"content-type": "application/x-www-form-urlencoded"})

    def get_system_status(self):
        return json.loads(self.send_api_request("/admin/api/system-status", method="GET").decode("utf-8"))

    @property
    def client(self):
        return self.__client

    @property
    def token(self) -> str:
        return self.__token
