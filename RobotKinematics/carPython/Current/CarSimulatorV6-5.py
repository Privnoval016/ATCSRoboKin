# A car simulator that reads instructions from a file and simulates the
# movement of a car with mecanum wheels based on the instructions.
# Red arrows indicate the car's velocity, and blue arrows indicate the car's
# forward direction. The individual wheel speeds are displayed in to the right.
__author__ = "Pranav Sukesh"
__date__ = "2025-01-25"


import math
import matplotlib.pyplot as plt
import numpy as np
from enum import Enum
from InstructionParser import parse_instruction_file
import sys
from mecanum import *


# Enum for the plots
class Plt(Enum):
    Main = 0
    UpperMiddle = 1
    UpperRight = 2
    LowerMiddle = 3
    LowerRight = 4


# Changes v from the standard basis to be in the normalized basis defined by b.
def change_basis(b, v):
    v = np.array(v)

    b0 = (np.array(b[0]) / np.linalg.norm(b[0])).squeeze()
    b1 = (np.array(b[1]) / np.linalg.norm(b[1])).squeeze()

    transition_matrix = np.array([b0, b1]).transpose()
    return transition_matrix @ v


# Finds the slope unit vector between the last two points in the given list.
def previous_slope_vector(passed_points):
    return ((passed_points[-1][0] - passed_points[-2][0]) /
            np.linalg.norm(passed_points[-1][0] - passed_points[-2][0]))


# Rotates a vector by the given angle in degrees (positive is CCW).
def rotate_vector(vector, angle):
    angle = np.radians(angle)
    vector = np.array([vector])
    rotation_matrix = np.array([[np.cos(angle), -np.sin(angle)],
                                [np.sin(angle), np.cos(angle)]]).squeeze()

    return np.dot(rotation_matrix, np.transpose(vector))


# Finds the osculating circle given two points and a tangent vector.
def find_center_with_tangent(begin, end, tangent):
    tangent = tangent / np.linalg.norm(tangent)
    normal = np.array([tangent[1], -tangent[0]])
    normal = normal / np.linalg.norm(normal)
    if np.dot(normal, end - begin) < 0:
        normal = -normal

    b_to_e = end - begin
    dist = np.linalg.norm(b_to_e)

    angle = np.arccos(np.dot(normal, b_to_e) / dist)

    radius = 0.5 * dist / np.cos(angle)
    center = begin + radius * normal

    return center, -(radius * normal)


# forward_dirs is an n by 2 array of forward directions
# returns an array of changes in angle
def find_omega(forward_dirs):
    final = np.empty(0)
    final = np.append(final, 0)
    for i in range(len(forward_dirs) - 1):
        angle = np.arccos(np.dot(forward_dirs[i], forward_dirs[i + 1]) /
                          (np.linalg.norm(forward_dirs[i]) *
                           np.linalg.norm(forward_dirs[i + 1])))

        if np.cross(forward_dirs[i], forward_dirs[i + 1]) < 0:
            angle = -angle

        # angf = np.arctan2(forward_dirs[i + 1][0], forward_dirs[i + 1][1])
        # angi = np.arctan2(forward_dirs[i][0], forward_dirs[i][1])
        #
        # angle = angf - angi

        angle /= time_per_update

        final = np.append(final, angle)

    return final


# Models the path given position and velocity functions parameterized by t.
# Point turns between non-smooth operations are automatically added if desired.
def model_path(x, y, x_t, y_t, t, step_size,
               rot_type, begin_dir, point_arr=None):
    path_points = np.transpose([x(t), y(t)])
    path_directions = [x_t(t), y_t(t)]
    magnitudes = np.linalg.norm(path_directions, axis=0)
    path_directions = np.transpose(np.array(path_directions) / magnitudes)
    magnitudes = [step_size / time_per_update for x in range(len(t))]

    if rot_type[0] == "MATCHVELOCITY":
        if rot_type[1] == 0:
            forward_dirs = path_directions
        else:
            forward_dirs = np.array([rotate_vector(x, rot_type[1])
                                     for x in path_directions]).squeeze()
    elif rot_type[0] == "INNORMAL" or rot_type[0] == "OUTNORMAL":  # todo later
        forward_dirs = np.array([rotate_vector(x, 90)
                                 for x in path_directions]).squeeze()
    elif rot_type[0] == "TURN":
        dir_step = np.linspace(0, rot_type[1], len(t))
        forward_dirs = np.array([rotate_vector(begin_dir, x)
                                 for x in dir_step]).squeeze()
    else:
        forward_dirs = [begin_dir for x in range(len(t))]

    final = np.empty((0, 4, 2))
    for i in range(len(t)):
        final = np.append(final, np.array([[[path_points[i][0],
                                             path_points[i][1]],
                                            [path_directions[i][0],
                                             path_directions[i][1]],
                                            [magnitudes[i], magnitudes[i]],
                                            [forward_dirs[i][0],
                                             forward_dirs[i][1]]]]), axis=0)

    if not auto_point_turn:
        return final

    if ((point_arr is not None and len(point_arr) > 0) or
            (point_arr is None and len(points) > 0)):
        current_dir = points[-1][3] if point_arr is None else point_arr[-1][3]
        target_dir = final[0][3]
        turn_points = point_turn_vectors(path_points[0], path_directions[0],
                                         current_dir, target_dir,
                                         ang_step_size)
        final = add_points(turn_points, final)

    return final


# Executes a point turn, rotating by the angle from the current direction.
# The point turn is drawn with an angular step size of omega.
def point_turn(center, velocity, begin_vec, rot_ang, omega):
    if rot_ang == 0:
        return np.empty((0, 4, 2))

    # Convert to numpy arrays
    center = np.array(center)
    begin_vec = np.array(begin_vec)

    num_steps = int(np.abs(rot_ang) / omega) + 1

    step = np.linspace(0, rot_ang, num_steps)

    rot_points = ([center[0] for x in range(num_steps)],
                  [center[1] for x in range(num_steps)])

    rot_vels = ([velocity[0] for x in range(num_steps)],
                [velocity[1] for x in range(num_steps)])

    magnitudes = [0 for x in range(num_steps)]

    forward_dirs = np.transpose(np.array(
        [rotate_vector(begin_vec, x) for x in step]).squeeze())

    final = np.empty((0, 4, 2))
    for i in range(num_steps):
        final = np.append(final, np.array([[[rot_points[0][i],
                                             rot_points[1][i]],
                                            [rot_vels[0][i], rot_vels[1][i]],
                                            [magnitudes[i], magnitudes[i]],
                                            [forward_dirs[0][i],
                                             forward_dirs[1][i]]]]), axis=0)

    return final


# Executes a point turn from a current direction to a target direction.
# The point turn is drawn with an angular step size of omega.
def point_turn_vectors(center, velocity, current_dir, target_dir, omega):
    # Convert to numpy arrays
    center = np.array(center)
    current_dir = np.array(current_dir)
    target_dir = np.array(target_dir)

    angle_to_turn = np.degrees(np.arccos(np.dot(current_dir, target_dir) / (
            np.linalg.norm(current_dir) * np.linalg.norm(target_dir))))

    if np.cross(current_dir, target_dir) < 0:
        angle_to_turn = -angle_to_turn

    return point_turn(center, np.array([0, 0]), current_dir, angle_to_turn, omega)


# Draws an arc between a start and an end point with an initial tangent vector.
# The arc is drawn with arc length steps of step.
def compute_arc(begin, end, tangent, step_size,
                rot_type, begin_dir, point_arr=None):
    # Convert to numpy arrays
    begin = np.array(begin)
    end = np.array(end)
    tangent = np.array(tangent)

    # If a line is defined, return the line
    if (np.cross(end - begin, tangent) == 0
            and np.dot(end - begin, tangent) > 0):
        return compute_line(begin, end, step_size)

    center, radius = find_center_with_tangent(begin, end, tangent)

    omega = np.degrees(step_size / np.linalg.norm(radius))

    begin_angle = np.arctan2(begin[1] - center[1], begin[0] - center[0])
    end_angle = np.arctan2(end[1] - center[1], end[0] - center[0])

    if np.cross(radius, tangent) > 0:
        mult = 1
        if end_angle < begin_angle:
            end_angle += 2 * np.pi
    else:
        mult = -1
        if end_angle > begin_angle:
            end_angle -= 2 * np.pi

    num_steps = int(np.abs(begin_angle - end_angle) / np.radians(omega)) + 1
    step = np.linspace(begin_angle, end_angle, num_steps)

    def x(t):
        return center[0] + np.linalg.norm(radius) * np.cos(t)

    def y(t):
        return center[1] + np.linalg.norm(radius) * np.sin(t)

    def x_t(t):
        return mult * -np.linalg.norm(radius) * np.sin(t)

    def y_t(t):
        return mult * np.linalg.norm(radius) * np.cos(t)

    return model_path(x, y, x_t, y_t, step, step_size,
                      rot_type, begin_dir, point_arr)


# Draws a semicircle between a start and an end point in the given direction.
# The semicircle is drawn with arc length steps of step.
def compute_semicircle(begin, end, ccw, step_size, rot_type, begin_dir):
    # Convert to numpy arrays
    begin = np.array(begin)
    end = np.array(end)

    normal = end - begin
    tangent = np.array([normal[1], -normal[0]])
    if not ccw:
        tangent = -tangent

    return compute_arc(begin, end, tangent, step_size, rot_type, begin_dir)


# Draws a full line between a start and an end point.
# The line is drawn with a step size of step.
def compute_line(begin, end, step_size, rot_type, begin_dir, point_arr=None):
    # Convert to numpy arrays
    begin = np.array(begin)
    end = np.array(end)

    num_steps = int(np.linalg.norm(end - begin) / step_size) + 1
    step = np.linspace(0, 1, num_steps)

    def x(t):
        return begin[0] + t * (end[0] - begin[0])

    def y(t):
        return begin[1] + t * (end[1] - begin[1])

    def x_t(t):
        return end[0] - begin[0] + (t * 0)

    def y_t(t):
        return end[1] - begin[1] + (t * 0)

    return model_path(x, y, x_t, y_t, step, step_size,
                      rot_type, begin_dir, point_arr)


# Draws a polygon between a list of vertices.
# The polygon is drawn with a step size of step.
def compute_polygon(vertices, step_size,
                    rot_type, begin_dir, connect_last=True):
    # Convert to numpy arrays
    vertices = np.array([np.array(vertex) for vertex in vertices])

    poly_points = np.empty((0, 5))
    for vertex in range(0, len(vertices)
                        if connect_last else len(vertices) - 1):

        poly_points = add_points(poly_points,
                                 compute_line(vertices[vertex],
                                              vertices[(vertex + 1)
                                                       % len(vertices)],
                                              step_size, rot_type,
                                              begin_dir, poly_points))

    return poly_points


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


# Adds a set of points to the current set of points.
def add_points(original, to_add):
    return np.append(original, to_add, axis=0)


# Computes the operation given the operation type, the parameters (varies based
# on operation), and the step value.
def compute_operation(operation, params, rot_type, step_size):
    # Continue from the last point if the first point is not defined
    if params[0] == 0:
        params[0] = points[-1][0]

    rot = rot_type

    begin_dir = points[-1][3] if len(points) > 0 else [1, 0]

    if operation == "Start" or operation == 0:
        return np.array([[[params[0][0], params[0][1]],
                          [params[1][0], params[1][1]],
                          [1, 1],
                          [params[1][0], params[1][1]]]])

    if operation == "Polygon" or operation == 1:
        if isinstance(params[-1], bool):
            return compute_polygon(params[:-1],
                                   step_size, rot, begin_dir, params[-1])
        else:
            return compute_polygon(params, step_size, rot, begin_dir)
    elif operation == "Arc" or operation == 2:
        if len(params) == 2:
            return compute_arc(params[0], params[1],
                               previous_slope_vector(points),
                               step_size, rot, begin_dir)
        else:
            return compute_arc(params[0], params[1], params[2],
                               step_size, rot, begin_dir)
    elif operation == "Semicircle" or operation == 3:
        return compute_semicircle(params[0], params[1], params[2],
                                  step_size, rot, begin_dir)
    elif operation == "Line" or operation == 4:
        return compute_line(params[0], params[1], step_size, rot, begin_dir)
    elif operation == "Point_Turn" or operation == 5:
        return point_turn(points[-1][0],
                          points[-1][1],
                          params[0], ang_step_size)
    else:
        return np.empty((0, 4, 2))


# equivalent of Arduino map()
def val_map(value, i_start, i_stop, o_start, o_stop):
    return (o_start + (o_stop - o_start) *
            ((value - i_start) / (i_stop - i_start)))


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


# Calculates individual wheel speeds for mecanum wheels.
def calculate_wheel_speeds(omega, v_x, v_y, a, b):
    return np.array([v_y + v_x - omega * (a + b),
                     v_y - v_x + omega * (a + b),
                     v_y - v_x - omega * (a + b),
                     v_y + v_x + omega * (a + b)])


# Called once at the beginning. Maps the path based on the defined operations.
def start():
    global points, wheel_speeds
    points = np.empty((0, 4, 2))

    for operation in operations:
        points = add_points(points, compute_operation(operation[0],
                                                      operation[1],
                                                      operation[2],
                                                      lin_step_size
                                                      if len(operation) < 4
                                                      else operation[4]))

    points[:, 2, 0] = points[:, 2, 0] / time_per_update

    scaled_vx = np.empty(0)
    scaled_vy = np.empty(0)
    for i in range(len(points)):
        angle = np.arctan2(points[i][3][1], points[i][3][0])
        u = rotate_vector(points[i][1], -np.degrees(angle))
        scaled_vx = np.append(scaled_vx, -u[1])
        scaled_vy = np.append(scaled_vy, u[0])
        # u = change_basis([points[i][3], rotate_vector(points[i][3], 90)], points[i][1])
        # scaled_vx = np.append(scaled_vx, u[1])
        # scaled_vy = np.append(scaled_vy, -u[0])

    omega = find_omega(points[:, 3, :])
    wheel_speeds = np.transpose(calculate_wheel_speeds(omega, scaled_vx,
                                                       scaled_vy,
                                                       x_extent, y_extent))

    print(omega)
    print(scaled_vx)
    print(scaled_vy)
    print(wheel_speeds)

    min = wheel_speeds.min()
    max = wheel_speeds.max()
    wheel_speeds = wheel_speeds[0:]
    for i in range(len(wheel_speeds)):
        for j in range (len(wheel_speeds[0])):
            wheel_speeds[i][j] = val_map(wheel_speeds[i][j], 0, max, 0, 100) \
                if wheel_speeds[i][j] > 0 else val_map(wheel_speeds[i][j], min, 0, -100, 0)


    global max_speed
    max_speed = 100

    setup_plot()


# Called for each frame. Draws the point at the current frame.
def update(frame):
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

    change_speed(wheel_speeds[frame])

    plot_wheels(axd, wheel_names, wheel_speeds[frame], max_speed)

    plt.draw()


# Main function to run the simulation.
def main():
    default_file = "test.txt"
    n = len(sys.argv)
    if n < 2:
        print("Usage: python3 " + sys.argv[0] + " <instruction_file>")
        file_name = default_file
    else:
        file_name = sys.argv[1]

    o, d = parse_instruction_file(file_name)

    global operations
    operations = o

    global lin_step_size, ang_step_size, time_per_update, auto_point_turn, \
        x_extent, y_extent, plot_vel, plot_dir
    time_per_update = d["TIME_SCALE"]
    lin_step_size = d["LIN_VEL"] * time_per_update
    ang_step_size = d["ANG_VEL"] * time_per_update
    auto_point_turn = d["AUTO_TURN"]
    x_extent = d["X_EXTENT"]
    y_extent = d["Y_EXTENT"]
    plot_vel = d["PLOT_VEL"]
    plot_dir = d["PLOT_DIR"]

    pwmOEn = 0

    start()

    for i in range(0, len(points)):
        update(i)
        plt.pause(time_per_update)

    #coast_car()

    plt.show()

    pwmOEn = 1
    GPIO.cleanup()


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        plt.close()
        print("Simulation stopped.")
        coast_car()
        pwmOEn = 1
        GPIO.cleanup()
        exit(0)
