import os
import ssl
import json
import hashlib
import base64
import time
from time import gmtime, strftime, sleep
from http.client import HTTPSConnection

import argparse

parser = argparse.ArgumentParser(description='Control Franka Emika Panda Desk interface remotely.')
parser.add_argument('--host', default="10.224.60.160",
                   help='name or ip of host')
parser.add_argument('--user', default="admin",
                   help='user name to login into Franka Desk')
parser.add_argument('--password', help='password')

args = parser.parse_args()

def encode_password(user, password):
    bs = ','.join([str(b) for b in hashlib.sha256((password + '#' + user + '@franka').encode('utf-8')).digest()])
    return base64.encodebytes(bs.encode('utf-8')).decode('utf-8')

class FrankaAPI:
    def __init__(self, hostname, user, password):
        self._hostname = hostname
        self._user = user
        self._password = password

    def __enter__(self):
        self._client = HTTPSConnection(self._hostname, context=ssl._create_unverified_context())
        self._client.connect()
        self._client.request('POST', '/admin/api/login',
                             body=json.dumps(
                                 {'login': self._user, 'password': encode_password(self._user, self._password)}),
                             headers={'content-type': 'application/json'})
        self._token = self._client.getresponse().read().decode('utf8')
        return self

    def __exit__(self, type, value, traceback):
        self._client.close()

    def start_task(self, task):
        self._client.request('POST', '/desk/api/execution',
                             body='id=%s' % task,
                             headers={'content-type': 'application/x-www-form-urlencoded',
                                      'Cookie': 'authorization=%s' % self._token})
        return self._client.getresponse().read()

    def unlock_brakes(self):
        self._client.request('POST', '/desk/api/robot/open-brakes',
                             headers={'content-type': 'application/x-www-form-urlencoded',
                                      'Cookie': 'authorization=%s' % self._token})
        return self._client.getresponse().read()
    
    def lock_brakes(self):
        self._client.request('POST', '/desk/api/robot/close-brakes',
                             headers={'content-type': 'application/x-www-form-urlencoded',
                                      'Cookie': 'authorization=%s' % self._token})
        return self._client.getresponse().read()

def log(message):
    return print(strftime("%H:%M:%S")+"  "+message)

with FrankaAPI(args.host, args.user, args.password) as api:
    log("Unlocking Brakes")
    try:
        api.unlock_brakes()
    except:
        log("ERROR Opening Brakes")
    sleep(11)
    log("Brakes Unlocked")

    sleep(5)
    try:
        api.lock_brakes()
    except:
        log("ERROR locking brakes")
