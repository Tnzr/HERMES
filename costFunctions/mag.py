import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation, PillowWriter


class Charge:
    def __init__(self, pos, pol):
        self.pos = pos
        self.pol = pol


res = 20
x, y = np.meshgrid(np.linspace(0, 600, res), np.linspace(0, 600, res))

positives = np.array([[-15, -10, 1],
            [-10, -5, 1],
             [-5, 0, 1],
             [0, 5, 1],
             [5, 5, 1],
             [9, 0, 1]])
positives[:, 0] += 20
positives[:, 0] *= 20
positives[:, 1] += 20
positives[:, 1] *= 20

print(positives)
negatives = positives + [0, -100, 0]
negatives[:, 2] *= -1
molecules = np.concatenate((positives, negatives))

charges = [Charge(particle[0:2], particle[2]) for particle in molecules]


def curl_map(x, y, charges):
    u = np.zeros(x.shape)
    v = np.zeros(y.shape)
    for charge in charges:
        u += charge.pol * (charge.pos[1] - y) / ((charge.pos[0] - x) ** 2 + (charge.pos[1] - y) ** 2)
        v += -charge.pol * (charge.pos[0] - x) / ((charge.pos[0] - x) ** 2 + (charge.pos[1] - y) ** 2)
    return u, v


def div_map(x, y, charges):
    u = np.zeros(x.shape)
    v = np.zeros(y.shape)
    for charge in charges:
        u += -(charge.pos[0] - x) / ((charge.pos[0] - x) ** 2 + (charge.pos[1] - y) ** 2)
        v += -(charge.pos[1] - y) / ((charge.pos[0] - x) ** 2 + (charge.pos[1] - y) ** 2)
    return u, v


u_curl, v_curl = curl_map(x, y, charges=charges)
u_div, v_div = div_map(x, y, charges=charges)


def plot_vf(x, y, u, v):
    fig, ax = plt.subplots()
    fig.patch.set_visible(False)
    plt.axis("off")
    plt.scatter(negatives[:, 0], negatives[:, 1], c="g")
    plt.scatter(positives[:, 0], positives[:, 1], c="r")
    plt.quiver(x, y, u, v)
    fig, ax = plt.subplots()
    fig.patch.set_visible(False)
    plt.axis("off")
    fig.tight_layout()
    plt.scatter(negatives[:, 0], negatives[:, 1], c="g")
    plt.scatter(positives[:, 0], positives[:, 1], c="r")
    plt.streamplot(x, y, u, v, density=1.5)


plot_vf(x, y, u_curl, v_curl)
plot_vf(x, y, u_div, v_div)


u_comb, v_comb = 0.1*u_curl+1*u_div, 1*v_div+0.1*v_curl


plot_vf(x,y, u_comb, v_comb)
plt.show()