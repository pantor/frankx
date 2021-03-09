import matplotlib
import matplotlib.pyplot as plt
import numpy as np


if __name__ == '__main__':
    # Data for plotting
    dq = np.genfromtxt('../test/data.csv', delimiter=' ')
    t = np.arange(0.0, dq.shape[0] / 1000, 0.001)

    ddq = np.diff(np.diff(dq, axis=0), axis=0)
    print(ddq.shape)

    fig, ax = plt.subplots()
    ax.plot(t, dq)
    # ax.plot(t[:-2], ddq)

    ax.set(xlabel='time (s)', ylabel='dq (rad/s)')
    ax.grid()

    plt.show()
