import matplotlib.pyplot as plt
import numpy as np
import copy

from otgx import Quintic, InputParameter, OutputParameter, Result


if __name__ == '__main__':
    inp = InputParameter()
    inp.current_position = [0.0]
    inp.current_velocity = [0.0]
    inp.current_acceleration = [0.0]
    inp.target_position = [1.0]
    inp.target_velocity = [0.0]
    inp.target_acceleration = [0.0]
    inp.max_velocity = [1.0]
    inp.max_acceleration = [100.0]
    inp.max_jerk = [100.0]

    out = OutputParameter()

    qui = Quintic(0.005)

    t = 0.0
    t_list = []
    out_list = []

    res = Result.Working
    while res == Result.Working:
        res = qui.update(inp, out)

        inp.current_position = out.new_position
        inp.current_velocity = out.new_velocity
        inp.current_acceleration = out.new_acceleration
        
        t_list.append(t)
        out_list.append(copy.copy(out))
        t += qui.delta_time


    dof = 0
    xaxis = t_list
    qaxis = np.array(list(map(lambda x: x.new_position, out_list)))
    dqaxis = np.array(list(map(lambda x: x.new_velocity, out_list)))
    ddqaxis = np.array(list(map(lambda x: x.new_acceleration, out_list)))
    dddqaxis = np.diff(ddqaxis, axis=0, prepend=0) / qui.delta_time

    plt.plot(xaxis, qaxis, label=f'r_{dof+1} (q)')
    plt.plot(xaxis, dqaxis, label=f'v_{dof+1} (dq)')
    plt.plot(xaxis, ddqaxis, label=f'a_{dof+1} (ddq)')
    plt.plot(xaxis, dddqaxis, label=f'j_{dof+1} (ddq)')

    plt.axhline(y=inp.max_velocity[dof], color='orange', linestyle='--', linewidth=1.1)
    plt.axhline(y=-inp.max_velocity[dof], color='orange', linestyle='--', linewidth=1.1)
    # plt.axhline(y=inp.max_acceleration[dof], color='g', linestyle='--', linewidth=1.1)
    # plt.axhline(y=-inp.max_acceleration[dof], color='g', linestyle='--', linewidth=1.1)
    # plt.axhline(y=inp.max_jerk[dof], color='red', linestyle='--', linewidth=1.1)
    # plt.axhline(y=-inp.max_jerk[dof], color='red', linestyle='--', linewidth=1.1)

    plt.xlabel('t')
    plt.legend()
    plt.grid(True)

    # plt.show()
    plt.savefig('quintic_trajectory.png')
