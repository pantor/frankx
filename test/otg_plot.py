import copy
import matplotlib.pyplot as plt
import numpy as np

from movex import Quintic, InputParameter, OutputParameter, Result, Ruckig, Smoothie


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
    inp.current_position = [0.0, 1.0, 0.0]
    inp.current_velocity = [0.0] * inp.degrees_of_freedom
    inp.current_acceleration = [0.0] * inp.degrees_of_freedom
    inp.target_position = [0.2, 0.5, 0.8]
    inp.target_velocity = [0.0] * inp.degrees_of_freedom
    inp.target_acceleration = [0.0] * inp.degrees_of_freedom
    inp.max_velocity = [1.5] * inp.degrees_of_freedom
    inp.max_acceleration = [2.0] * inp.degrees_of_freedom
    inp.max_jerk = [3.0] * inp.degrees_of_freedom
    inp.minimum_duration = None

    # otg = Quintic(0.005)
    otg = Smoothie(0.005)
    # otg = Reflexxes(0.005)
    # otg = Ruckig(0.005)

    t_list, out_list = walk_through_trajectory(otg, inp)

    qaxis = np.array(list(map(lambda x: x.new_position, out_list)))
    dqaxis = np.array(list(map(lambda x: x.new_velocity, out_list)))
    ddqaxis = np.array(list(map(lambda x: x.new_acceleration, out_list)))
    dddqaxis = np.diff(ddqaxis, axis=0, prepend=0) / otg.delta_time


    plt.figure(figsize=(8.0, 2.0 + 3.0 * inp.degrees_of_freedom), dpi=120)

    for dof in range(inp.degrees_of_freedom):
        plt.subplot(3, 1, dof + 1)
        plt.plot(t_list, qaxis[:, dof], label=f'r_{dof+1}')
        plt.plot(t_list, dqaxis[:, dof], label=f'v_{dof+1}')
        plt.plot(t_list, ddqaxis[:, dof], label=f'a_{dof+1}')
        plt.plot(t_list, dddqaxis[:, dof], label=f'j_{dof+1}')

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
    print(f'Trajectory duration: {t_list[-1]:0.4f} [s]')

    # plt.show()
    plt.savefig('otg_trajectory.png')
