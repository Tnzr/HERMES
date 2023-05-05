import numpy as np
import matplotlib.pyplot as plt


class Charge:
    def __init__(self, pos, pol):
        self.pos = pos
        self.pol = pol


res = 20
x, y = np.meshgrid(np.linspace(-20, 20, res), np.linspace(-20, 20, res))

positives = np.array([[-15, -10, 1],
             [-10, -5, 1],
             [-5, 0, 1],
             [0, 5, 1],
             [5, 5, 1],
             [10, 0, 1]])

negatives = positives + [0, -10, 0]
negatives[:, 2] *= -1
print(negatives)
molecules = np.concatenate((positives, negatives))

charges = [Charge(particle[0:2], particle[2]) for particle in molecules]
cost = np.zeros(x.shape)
for charge in charges:
    cost += (1/((charge.pos[0]-x)**2+(charge.pos[1]-y)**2) +
                   1/((charge.pos[0]-x)**2+(charge.pos[1]-y)**2))

plt.pcolormesh(x, y, cost, shading="gouraud")
plt.show()
