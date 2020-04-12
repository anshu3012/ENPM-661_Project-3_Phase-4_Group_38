import math
import matplotlib.pyplot as plt
import numpy as np


def non_holo_const(w1, w2):
    t = 0
    theta = 30 * math.pi / 180
    r = 0.038
    dist_bw_wheels = 0.354
    dt = 0.1
    x = 4
    y = 5

    while t < 2:

        t = t + dt

        dx = (r / 2) * (w1 + w2) * math.cos(theta) * dt
        dy = (r / 2) * (w1 + w2) * math.sin(theta) * dt
        dtheta = (r / dist_bw_wheels) * (w1 - w2) * dt

        plt.plot([x, x + dx], [y, y + dy], color="blue")

        x = x + dx
        y = y + dy
        theta = theta + dtheta

        print("x", x)
        print("y", y)

    return x, y, theta

actions = [[5, 5], [5, 0], [0, 5], [5, 10], [10, 5]]

for action in actions:
    X1 = non_holo_const(action[0], action[1])
    for action in actions:
        X2 = non_holo_const(action[0], action[1])

plt.title('Curves', fontsize=10)
plt.show()
plt.close()
