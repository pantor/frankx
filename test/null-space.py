import matplotlib
import matplotlib.pyplot as plt
from matplotlib import patches
import numpy as np

from frankx import Affine, Kinematics, NullSpaceHandling


if __name__ == '__main__':
    q0 = [-1.3773043, 1.40564879, 1.774009, -2.181041, -1.35430, 1.401228, 0.059601]

    x_new = Affine(0.5, 0.0, 0.333 - 0.107, 0, 0, 0)

    y = []
    z = []

    for j2 in np.linspace(0.0, 2.9, 200):
        null_space = NullSpaceHandling(1, j2)

        q_new = Kinematics.inverse(x_new.vector(), q0, null_space)
        elbow = Affine(Kinematics.forwardElbow(q_new)).translation()

        print('New joints: ', q_new[1])
        # print('elbow', elbow)

        y.append(elbow[1])
        z.append(elbow[2])


    t = np.arange(0.0, 2.0, 0.01)
    s = 1 + np.sin(2 * np.pi * t)

    fig, ax = plt.subplots()
    ax.plot(y, z)

    ax.set(xlabel='y [m]', ylabel='z [m]')
    ax.grid()

    e1 = patches.Ellipse((0.0, 0.333), 2*0.30086, 2*0.29102, fill=False, color='r')
    # e1 = patches.Ellipse((0.0, 0.333 + 0.0352), 2*0.2972, 2*0.281, fill=False, color='r')
    ax.add_patch(e1)

    plt.show()
