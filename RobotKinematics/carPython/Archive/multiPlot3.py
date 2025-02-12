"""
by-wheel speed display utility for car movement

Revision History:
Version: date: changes:
         Feb 9  converted to function
"""
__version__ = '0.2'
__date__ = 'Feb 9, 2023'
__author__ = 'Martin Baynes'

import math
import numpy as np
import matplotlib.pyplot as plt
from matplotlib import interactive


# equivalent of Arduino map()
def val_map(value, istart, istop, ostart, ostop):
    return ostart + (ostop - ostart) * ((value - istart) / (istop - istart))


# plot_wheels(graphs, wheelSpeed, maxSpeed, radius):
# use adk[0] for car's path plot
def plot_wheels(axd, wheelName, wheelSpeed, maxSpeed, radius=1):
    for idx, k in enumerate(axd):
        if idx > 0:
            ang = val_map(wheelSpeed[idx-1], -maxSpeed, maxSpeed, np.pi, 0)
            dx = radius * math.cos(ang)
            dy = radius * math.sin(ang)
            axd[k].clear()
            axd[k].set(xlim=(-1.5, 1.5),  xticks=[], yticks=[],
                       ylim=(-1.5, 1.5))
            axd[k].annotate(wheelName[idx - 1], (0.1, 0.5),
                            xycoords='axes fraction', va='center')
            axd[k].quiver(0, 0, dy, dx, width=0.04,
                          pivot='mid', angles='uv',
                          scale_units='height', scale=2)
            axd[k].annotate(wheelSpeed[idx - 1], (.8, 0.5),
                            xycoords='axes fraction', va='center')


fig, axd = plt.subplot_mosaic([['left', 'left', 'upper middle', 'upper right'],
                               ['left', 'left', 'lower middle', 'lower right']
                               ], figsize=(8, 4), layout="constrained")
fig.suptitle('Simulator with wheel speed display')
plt.style.use('_mpl-gallery-nogrid')
wheelName = ['fr', 'fl', 'rr', 'rl']

interactive(True)

maxSpeed = 100
# make data
# wheelSpeed = [50, -50, 100, 0]
for sp in range(-100, 101, 10):
    wheelSpeed = [sp, sp, sp, sp]
    # plot
    plot_wheels(axd, wheelName, wheelSpeed, maxSpeed)
    plt.pause(0.1)


plt.show()
# prevent window exit at end of plotting
bye = input('press enter key to end: ')
print('Bye')
