import matplotlib
import matplotlib.pyplot as plt
from matplotlib import patches
import numpy as np

from franky import Affine, Kinematics, NullSpaceHandling

def Sqrt(x):
    return np.sqrt(x)

def Power(x, a):
    return np.power(x, a)

def Cos(x):
    return np.cos(x)

def Sin(x):
    return np.sin(x)


if __name__ == '__main__':
    q0 = [-1.3773043, 1.40564879, 1.774009, -2.181041, -1.35430, 1.401228, 0.059601]

    x_new = Affine(0.803, 0.0, 0.333 - 0.107, 0, 0, 0)

    y, z = [], []
    for j2 in np.linspace(0.0, 2.9, 200):
        null_space = NullSpaceHandling(1, j2)

        q_new = Kinematics.inverse(x_new.vector(), q0, null_space)
        elbow = Affine(Kinematics.forwardElbow(q_new)).translation()

        # print('New joints: ', j2, elbow)
        # print('elbow', elbow)

        y.append(elbow[1])
        z.append(elbow[2])
        
        # y.append(elbow[0])
        # z.append(elbow[2])

    d3 = 0.316
    d5 = 0.384
    a4 = 0.0825
    a5 = -0.0825
    a7 = 0.088
    ypos = x_new.x

    print(z[50])

    alpha_y, alpha_z = [], []
    for alpha in np.linspace(0.0, 6.2, 100):
        beta = 0.0
        blength = Sqrt(-Power(a4,4) + 2*Power(a4,2)*Power(a5,2) - Power(a5,4) - 2*Power(a4,2)*Power(d3,2) + 2*Power(a5,2)*Power(d3,2) - Power(d3,4) + 2*Power(a4,2)*Power(d5,2) - 2*Power(a5,2)*Power(d5,2) + 2*Power(d3,2)*Power(d5,2) - Power(d5,4) + 2*Power(a4,2)*Power(ypos,2) + 2*Power(a5,2)*Power(ypos,2) + 2*Power(d3,2)*Power(ypos,2) + 2*Power(d5,2)*Power(ypos,2) - Power(ypos,4) - 4*Power(a4,2)*a7*ypos*Cos(beta) - 4*Power(a5,2)*a7*ypos*Cos(beta) - 4*a7*Power(d3,2)*ypos*Cos(beta) - 4*a7*Power(d5,2)*ypos*Cos(beta) + 4*a7*Power(ypos,3)*Cos(beta) + 2*Power(a4,2)*Power(a7,2)*Power(Cos(beta),2) + 2*Power(a5,2)*Power(a7,2)*Power(Cos(beta),2) + 2*Power(a7,2)*Power(d3,2)*Power(Cos(beta),2) + 2*Power(a7,2)*Power(d5,2)*Power(Cos(beta),2) - 6*Power(a7,2)*Power(ypos,2)*Power(Cos(beta),2) + 4*Power(a7,3)*ypos*Power(Cos(beta),3) - Power(a7,4)*Power(Cos(beta),4))/(2.*Sqrt(Power(ypos,2) - 2*a7*ypos*Cos(beta) + Power(a7,2)*Power(Cos(beta),2)))

        alength = Sqrt(-Power(a4,4) + 2*Power(a4,2)*Power(a5,2) - Power(a5,4) + 2*Power(a4,2)*Power(a7,2) - 2*Power(a5,2)*Power(a7,2) - Power(a7,4) - 2*Power(a4,2)*Power(d3,2) + 2*Power(a5,2)*Power(d3,2) + 2*Power(a7,2)*Power(d3,2) - Power(d3,4) + 4*Power(a4,2)*a7*d5 - 4*Power(a5,2)*a7*d5 - 4*Power(a7,3)*d5 + 4*a7*Power(d3,2)*d5 + 2*Power(a4,2)*Power(d5,2) - 2*Power(a5,2)*Power(d5,2) - 6*Power(a7,2)*Power(d5,2) + 2*Power(d3,2)*Power(d5,2) - 4*a7*Power(d5,3) - Power(d5,4) + 2*Power(a4,2)*Power(ypos,2) + 2*Power(a5,2)*Power(ypos,2) + 2*Power(a7,2)*Power(ypos,2) + 2*Power(d3,2)*Power(ypos,2) + 4*a7*d5*Power(ypos,2) + 2*Power(d5,2)*Power(ypos,2) - Power(ypos,4))/(2.*ypos)
        a = -alength * np.cos(alpha)
        b = 0.333 + blength * np.sin(alpha)

        alpha_y.append(a)
        alpha_z.append(b)


    t = np.arange(0.0, 2.0, 0.01)
    s = 1 + np.sin(2 * np.pi * t)

    fig, ax = plt.subplots()
    ax.plot(y, z)
    ax.plot(alpha_y, alpha_z)

    ax.set(xlabel='y [m]', ylabel='z [m]')
    ax.grid()

    # e1 = patches.Ellipse((0.0, 0.333), 2*0.047153, 2*0.0510773, fill=False, color='r')
    # e1 = patches.Ellipse((0.0, 0.333), 2*0.300857, 2*0.291014, fill=False, color='r')

    # e1 = patches.Ellipse((0.0, 0.333), 2*0.0354762, 2*0.0415347, fill=False, color='r')

    # ax.add_patch(e1)

    plt.savefig('elbow.png')
