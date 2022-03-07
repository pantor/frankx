from pathlib import Path as Pathlib

import matplotlib.pyplot as plt
import numpy as np

from pyaffx import Affine
from _movex import Path, TimeParametrization


def walk_through_path(traj):
    p = traj.path
    t_list, s_list, q_list, dq_list, ddq_list = [], [], [], [], []
    for state in traj.states:
        t_list.append(state.t)
        s_list.append(state.s)
        q_list.append(p.q(state.s))
        dq_list.append(p.dq(state.s, state.ds))
        ddq_list.append(p.ddq(state.s, state.ds, state.dds))
    return np.array(t_list), np.array(s_list), np.array(q_list), np.array(dq_list), np.array(ddq_list)


def plot_trajectory(traj):
    t_list, s_list, qaxis, dqaxis, ddqaxis = walk_through_path(traj)
    plt.figure(figsize=(8.0, 2.0 + 3.0 * p.degrees_of_freedom), dpi=120)

    for dof in range(p.degrees_of_freedom):
        plt.subplot(p.degrees_of_freedom, 1, dof + 1)
        plt.plot(t_list, qaxis[:, dof], label=f'q_{dof+1}')
        plt.plot(t_list, dqaxis[:, dof], label=f'dq_{dof+1}')
        plt.plot(t_list, ddqaxis[:, dof], label=f'ddq_{dof+1}')
        plt.legend()
        plt.grid(True)

    plt.xlabel('t')
    print(f'Path length: {p.length:0.4f}')
    print(f'Trajectory duration: {len(t_list) * 0.0001:0.4f} [s]')

    # plt.show()
    plt.savefig(Pathlib(__file__).parent.parent / 'build' / 'trajectory.png')


if __name__ == '__main__':
    p = Path([
        Affine(0.0, 0.0, 0.0, 0.0, 0.0, 0.0),
        Affine(1.0, 0.0, 0.5, 0.0, 0.0, 0.0),
        Affine(1.0, 1.0, 1.0, 0.0, 0.0, -3.0),
    ], blend_max_distance=0.04)

    tp = TimeParametrization(delta_time=0.0001)
    trajectory = tp.parametrize(p, 10.0, 10.0, 10.0)
    plot_trajectory(trajectory)
