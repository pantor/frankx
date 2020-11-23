from argparse import ArgumentParser
from time import sleep

from frankx import Robot


if __name__ == '__main__':
    parser = ArgumentParser(description='Control Franka Emika Panda Desk interface remotely.')
    parser.add_argument('--host', default='172.16.0.2', help='FCI IP of the robot')
    parser.add_argument('--user', default='admin', help='user name to login into Franka Desk')
    parser.add_argument('--password', help='password')

    args = parser.parse_args()


    with Robot(args.host, args.user, args.password) as api:
        # api.lock_brakes()
        api.unlock_brakes()
