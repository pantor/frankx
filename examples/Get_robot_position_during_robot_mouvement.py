"""

Example of use of get_O_T_EE method

Author: Maxime Edeline
Date: 28/07/2022
Version: 0.1.0

Context :
This work was done during an internship under the supervision of Guenhael Le Quilliec,
associate professor at "Laboratoire de Mécanique Gabriel Lamé" of Polytech Tours, in Tours, France.
The aim of this work was to be able to synchronise the movements of Franka Emika Robot (Panda)
with other systems (e.g. 3D printer head attached to the robot).
The preferred solution was to modify frankx library.
The present Python code is an example of use of the modifications added to frankx source code.
In this example, the position of the robot is simply printed while it is moving.
This position can be used to easily synchronise any other system during the movements of the robot.

"""


from argparse import ArgumentParser
from frankx import Affine, PathMotion, Robot

if __name__ == '__main__':
    parser = ArgumentParser()
    parser.add_argument('--host', default='172.16.0.2', help='FCI IP of the robot')
    args = parser.parse_args()

    # Connect to the robot
    robot = Robot(args.host)
    robot.set_default_behavior()
    robot.recover_from_errors()

    # Reduce the acceleration and velocity dynamic
    robot.set_dynamic_rel(0.15)

    # Define and move forwards
    motion = PathMotion([
        Affine(0.5, 0.0, 0.35),
        Affine(0.5, 0.25, 0.55),
        Affine(0.5, 0.5, 0.35),
    ], blend_max_distance=0.05)


    pthread = robot.move_async(motion)

    while pthread.is_alive() :
        O_T_EE_async = robot.get_O_T_EE_async()
        print(O_T_EE_async)

    print("Execution completed")



