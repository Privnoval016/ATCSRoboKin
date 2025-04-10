#
__author__ = "Pranav Sukesh"
__date__ = "2025-01-25"


import matplotlib.pyplot as plt
from CarPathGen import *


# Sets up the plot with limits such that all points are visible.
def setup_plot():
    global fig, axd
    fig, axd = plt.subplot_mosaic(
        [[Plt.Main, Plt.Main, Plt.UpperMiddle, Plt.UpperRight],
         [Plt.Main, Plt.Main, Plt.LowerMiddle, Plt.LowerRight]
         ], figsize=(8, 4), layout="constrained")

    x_lim = np.array([points[:, 0, 0].min(), points[:, 0, 0].max()])
    x_dist = x_lim[1] - x_lim[0]
    y_lim = np.array([points[:, 0, 1].min(), points[:, 0, 1].max()])
    y_dist = y_lim[1] - y_lim[0]
    dist = max(x_dist, y_dist)

    axd[Plt.Main].set_xlim(x_lim[0] - 0.3 * dist, x_lim[1] + 0.3 * dist)
    axd[Plt.Main].set_ylim(y_lim[0] - 0.3 * dist, y_lim[1] + 0.3 * dist)

    fig.suptitle('Simulator with wheel speed display')
    plt.style.use('_mpl-gallery-nogrid')

    global wheel_names
    wheel_names = ['fl', 'fr', 'rl', 'rr']


# Plots the wheel speeds on the given axes.
def plot_wheels(axes, wheel_name, wheel_speed, max_speed, radius=1):
    for idx, k in enumerate(axes):
        if idx > 0:
            ang = val_map(wheel_speed[idx-1], -max_speed, max_speed, np.pi, 0)
            dx = radius * math.cos(ang)
            dy = radius * math.sin(ang)
            axes[k].clear()
            axes[k].set(xlim=(-1.5, 1.5),  xticks=[], yticks=[],
                        ylim=(-1.5, 1.5))
            axes[k].annotate(wheel_name[idx - 1], (0.1, 0.5),
                             xycoords='axes fraction', va='center')
            axes[k].quiver(0, 0, dy, dx, width=0.04,
                           pivot='mid', angles='uv',
                           scale_units='height', scale=2)
            axes[k].annotate(round(wheel_speed[idx - 1], 2), (.8, 0.5),
                             xycoords='axes fraction', va='center')


# Called for each frame. Draws the point at the current frame.
def sim_update(frame):
    if plot_vel:
        axd[Plt.Main].quiver(points[frame][0][0], points[frame][0][1],
                             points[frame][1][0], points[frame][1][1],
                             color='red', scale=1, scale_units='xy',
                             angles='xy')
    if plot_dir:
        axd[Plt.Main].quiver(points[frame][0][0], points[frame][0][1],
                             points[frame][3][0], points[frame][3][1],
                             color='blue', scale=1, scale_units='xy',
                             angles='xy')

    plot_wheels(axd, wheel_names, wheel_speeds[frame], 100)

    plt.draw()


# Main function to run the simulation.
def main():
    global time_per_update, plot_vel, plot_dir
    time_per_update, plot_vel, plot_dir, wheel_rad = read_info()

    global points, wheel_speeds
    points, wheel_speeds = start()

    setup_plot()

    for i in range(0, len(points) - 1):
        sim_update(i)
        plt.pause(time_per_update)

    plt.show()


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        plt.close()
        print("Simulation stopped.")
