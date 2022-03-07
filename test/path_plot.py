from pathlib import Path as Pathlib

import matplotlib.pyplot as plt
import numpy as np

from pyaffx import Affine
from _movex import Path


def walk_through_path(path, s_diff=0.001):
    s_list, q_list, pdq_list, pddq_list = [], [], [], []
    for s in np.arange(0, p.length, s_diff):
        s_list.append(s)
        q_list.append(p.q(s))
        pdq_list.append(p.pdq(s))
        pddq_list.append(p.pddq(s))
    return np.array(s_list), np.array(q_list), np.array(pdq_list), np.array(pddq_list)


def plot_path(p: Path):
    s_list, qaxis, pdqaxis, pddqaxis = walk_through_path(p)
    plt.figure(figsize=(8.0, 2.0 + 3.0 * p.degrees_of_freedom), dpi=120)

    for dof in range(p.degrees_of_freedom):
        plt.subplot(p.degrees_of_freedom, 1, dof + 1)
        plt.plot(s_list, qaxis[:, dof], label=f'q_{dof+1}')
        plt.plot(s_list, pdqaxis[:, dof], label=f'dq_{dof+1}')
        plt.plot(s_list, pddqaxis[:, dof], label=f'ddq_{dof+1}')
        plt.legend()
        plt.grid(True)

    plt.xlabel('s')
    print(f'Path length: {p.length:0.4f}')

    # plt.show()
    plt.savefig(Pathlib(__file__).parent.parent / 'build' / 'path.png')


if __name__ == '__main__':
    p = Path([
        Affine(0.0, 0.0, 0.0, 0.0, 0.0, 0.0),
        Affine(1.0, 0.0, 0.5, 0.0, 0.0, 0.0),
        Affine(1.0, 1.0, 1.0, 0.0, 0.0, -3.0),
    ], blend_max_distance=0.06)

    print(p.max_pddq())
    print(p.max_pdddq())
    plot_path(p)
