from pathlib import Path as Pathlib

import matplotlib.pyplot as plt
import numpy as np

from movex import Path


def walk_through_path(path, s_diff=0.001):
    s_list = []
    q_list = []
    for s in np.arange(0, p.length, s_diff):
        s_list.append(s)
        q_list.append(p.q(s))
    return np.array(s_list), np.array(q_list)


if __name__ == '__main__':
    p = Path.Linear([
        [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
        [1.0, 0.0, 0.5, 0.0, 0.0, 0.0, 0.0],
        [1.0, 1.0, 1.0, 0.0, 0.0, 0.0, -3.0],
    ], blend_max_distance=0.08)

    s_list, qaxis = walk_through_path(p)
    plt.figure(figsize=(8.0, 2.0 + 3.0 * p.degrees_of_freedom), dpi=120)

    for dof in range(p.degrees_of_freedom):
        plt.subplot(p.degrees_of_freedom, 1, dof + 1)
        plt.plot(s_list, qaxis[:, dof], label=f'r_{dof+1}')
        plt.legend()
        plt.grid(True)

    plt.xlabel('s')
    print(f'Path length: {p.length:0.4f}')

    # plt.show()
    plt.savefig(Pathlib(__file__).parent.parent / 'build' / 'path.png')
