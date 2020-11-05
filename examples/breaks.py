from pathlib import Path
import yaml

from frankx import Robot


if __name__ == '__main__':
    with open(Path(__file__).parent / 'login.yaml') as file:
        login_data = yaml.full_load(file)

    robot = Robot('172.16.0.2', username=login_data['username'], password=login_data['password'])
    robot.login(headless=False)
    robot.lock_brakes()
