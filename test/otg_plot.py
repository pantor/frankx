import copy
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np

from movex import Quintic, InputParameter, OutputParameter, Result, Ruckig, Smoothie, Reflexxes


def walk_through_trajectory(otg, inp):
    t = 0.0
    t_list, out_list = [], []
    out = OutputParameter()

    res = Result.Working
    while res == Result.Working:
        res = otg.update(inp, out)

        inp.current_position = out.new_position
        inp.current_velocity = out.new_velocity
        inp.current_acceleration = out.new_acceleration

        t_list.append(t)
        out_list.append(copy.copy(out))
        t += otg.delta_time

    return t_list, out_list


if __name__ == '__main__':
    inp = InputParameter()
    inp.current_position = [-0.9]
    inp.current_velocity = [-0.04] * inp.degrees_of_freedom
    inp.current_acceleration = [1.0] * inp.degrees_of_freedom
    inp.target_position = [1.0]
    inp.target_velocity = [0.0] * inp.degrees_of_freedom
    inp.target_acceleration = [0.0] * inp.degrees_of_freedom
    inp.max_velocity = [0.2] * inp.degrees_of_freedom
    inp.max_acceleration = [1.5] * inp.degrees_of_freedom
    inp.max_jerk = [0.71] * inp.degrees_of_freedom
    inp.minimum_duration = None

    # otg = Quintic(0.005)
    # otg = Smoothie(0.005)
    otg = Reflexxes(0.005)
    #otg = Ruckig(0.005)

    t_list, out_list = walk_through_trajectory(otg, inp)

    # print(f'Calculation duration: {otg.last_calculation_duration:0.1f} [µs]')

    qaxis = np.array(list(map(lambda x: x.new_position, out_list)))
    dqaxis = np.array(list(map(lambda x: x.new_velocity, out_list)))
    ddqaxis = np.array(list(map(lambda x: x.new_acceleration, out_list)))
    dddqaxis = np.diff(ddqaxis, axis=0, prepend=ddqaxis[0, 0]) / otg.delta_time

    plt.figure(figsize=(8.0, 2.0 + 3.0 * inp.degrees_of_freedom), dpi=120)

    for dof in range(inp.degrees_of_freedom):
        plt.subplot(inp.degrees_of_freedom, 1, dof + 1)
        plt.plot(t_list, qaxis[:, dof], label=f'r_{dof+1}')
        plt.plot(t_list, dqaxis[:, dof], label=f'v_{dof+1}')
        plt.plot(t_list, ddqaxis[:, dof], label=f'a_{dof+1}')
        plt.plot(t_list, dddqaxis[:, dof], label=f'j_{dof+1}')

        # Plot limit lines
        if inp.max_velocity[dof] < 1.5 * np.max(dqaxis[:, dof]):
            plt.axhline(y=inp.max_velocity[dof], color='orange', linestyle='--', linewidth=1.1)

        if -inp.max_velocity[dof] > 1.5 * np.min(dqaxis[:, dof]):
            plt.axhline(y=-inp.max_velocity[dof], color='orange', linestyle='--', linewidth=1.1)

        if inp.max_acceleration[dof] < 1.5 * np.max(ddqaxis[:, dof]):
            plt.axhline(y=inp.max_acceleration[dof], color='g', linestyle='--', linewidth=1.1)

        if -inp.max_acceleration[dof] > 1.5 * np.min(ddqaxis[:, dof]):
            plt.axhline(y=-inp.max_acceleration[dof], color='g', linestyle='--', linewidth=1.1)

        if inp.max_jerk[dof] < 1.5 * np.max(dddqaxis[:, dof]):
            plt.axhline(y=inp.max_jerk[dof], color='red', linestyle='--', linewidth=1.1)

        if -inp.max_jerk[dof] > 1.5 * np.min(dddqaxis[:, dof]):
            plt.axhline(y=-inp.max_jerk[dof], color='red', linestyle='--', linewidth=1.1)

        plt.legend()
        plt.grid(True)


    plt.xlabel('t')
    print(f'Trajectory duration: {t_list[-1]:0.3f} [s]')

    # plt.show()
    plt.savefig(Path(__file__).parent.parent / 'build' / 'otg_trajectory.png')
