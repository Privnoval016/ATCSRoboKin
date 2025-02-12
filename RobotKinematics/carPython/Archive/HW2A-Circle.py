import matplotlib.pyplot as plt
import numpy as np

xCenter = 0
yCenter = 0
rad = 5
step_size = 10

secPerUpdate = 1

x_points = []
y_points = []


def compute_circle(x_center, y_center, radius, deg):
    num_steps = int(360 / deg) + 1

    step = np.linspace(0, 2*np.pi, num_steps)
    x_points.extend(radius * np.cos(step) + x_center)
    y_points.extend(radius * np.sin(step) + y_center)


def start():
    global fig, ax
    fig, ax = plt.subplots()
    ax.set_xlim(xCenter - (1.5 * rad), xCenter + (1.5 * rad))
    ax.set_ylim(yCenter - (1.5 * rad), yCenter + (1.5 * rad))
    plt.axis('scaled')

    compute_circle(xCenter, yCenter, rad, step_size)


def update(frame):
    ax.scatter(x_points[frame], y_points[frame], color='red')
    plt.draw()


start()

for i in range(0, len(x_points)):
    update(i)
    plt.pause(secPerUpdate)

plt.show()
