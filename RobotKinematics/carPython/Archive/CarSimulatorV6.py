import math
import matplotlib.pyplot as plt
import numpy as np
from enum import Enum
from Current.InstructionParser import parse_instruction_file
import sys


# Enum for the plots
class Plt(Enum):
    Main = 0
    UpperMiddle = 1
    UpperRight = 2
    LowerMiddle = 3
    LowerRight = 4


# Finds the slope unit vector between the last two points in the given list.
def previous_slope_vector(passed_points):
    return ((passed_points[-1][0:2] - passed_points[-2][0:2]) /
            np.linalg.norm(passed_points[-1][0:2] - passed_points[-2][0:2]))


# Rotates a vector by the given angle (positive is CCW).
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


# computes the instantaneous angular velocity of the robot using the
# derivative of atan2(x(t), y(t)) with respect to t
def find_omega(x, y, x_t, y_t, t):

    def atan2_x(x1, y1):
        if x1 == 0 and y1 == 0:
            return 0
        return -y1 / (x1**2 + y1**2)
    def atan2_y(x1, y1):
        if x1 == 0 and y1 == 0:
            return 0
        return x1 / (x1**2 + y1**2)

    omegas = [atan2_x(x(time), y(time)) * x_t(time) + atan2_y(x(time), y(time)) * y_t(time) for time in t]

    return omegas

# needs to rotate
def actually_find_omega(rot_angle, num_steps):
    return rot_angle / num_steps


# Models the path given position and velocity functions parameterized by t.
# Point turns between non-smooth operations are automatically added if desired.
def model_path(x, y, x_t, y_t, t, step_size, init_dir, rot_angle, auto_point_turn, point_arr=None):
    path_points = [x(t), y(t)]
    path_directions = [x_t(t), y_t(t)]
    magnitudes = np.linalg.norm(path_directions, axis=0)
    path_directions = np.array(path_directions) / magnitudes
    magnitudes = np.array([step_size for x in range(len(t))])
    ang_vel = find_omega(x, y, x_t, y_t, t)

    final = np.transpose([*path_points, *path_directions, magnitudes, ang_vel])

    if not auto_point_turn:
        return final

    if ((point_arr is not None and len(point_arr) > 0) or
            (point_arr is None and len(points) > 0)):
        current_dir = points[-1][2:4] if point_arr is None else point_arr[-1][2:4]
        target_dir = [path_directions[0][0], path_directions[1][0]]
        turn_points = point_turn_vectors((path_points[0][0], path_points[1][0]), current_dir, target_dir, ang_step_size)
        final = add_points(turn_points, final)

    return final


# Executes a point turn, rotating by the angle from the current direction.
# The point turn is drawn with an angular step size of omega.
def point_turn(center, begin_vec, rot_ang, omega):
    if rot_ang == 0:
        return np.transpose([[center[0]], [center[1]],
                             [begin_vec[0]], [begin_vec[1]], [1]])

    # Convert to numpy arrays
    center = np.array(center)
    begin_vec = np.array(begin_vec)

    num_steps = int(np.abs(rot_ang) / omega) + 1

    step = np.linspace(0, rot_ang, num_steps)

    rot_points = ([center[0] for x in range(num_steps)],
                  [center[1] for x in range(num_steps)])

    directions = np.transpose(np.array(
        [rotate_vector(begin_vec, x) for x in step]).squeeze())

    magnitudes = [1 for x in range(num_steps)]
    ang_vel = [omega for x in range(num_steps)]

    return np.transpose([*rot_points, *directions, magnitudes, ang_vel])


# Executes a point turn from a current direction to a target direction.
# The point turn is drawn with an angular step size of omega.
def point_turn_vectors(center, current_dir, target_dir, omega):
    # Convert to numpy arrays
    center = np.array(center)
    current_dir = np.array(current_dir)
    target_dir = np.array(target_dir)

    angle_to_turn = np.degrees(np.arccos(np.dot(current_dir, target_dir) / (
            np.linalg.norm(current_dir) * np.linalg.norm(target_dir))))

    if np.cross(current_dir, target_dir) < 0:
        angle_to_turn = -angle_to_turn

    return point_turn(center, current_dir, angle_to_turn, omega)


# Draws an arc between a start and an end point with an initial tangent vector.
# The arc is drawn with arc length steps of step.
def compute_arc(begin, end, tangent, step_size, init_dir, rot_ang, point_arr=None):
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

    return model_path(x, y, x_t, y_t, step, step_size, auto_point_turn, point_arr)


# Draws a semicircle between a start and an end point in the given direction.
# The semicircle is drawn with arc length steps of step.
def compute_semicircle(begin, end, ccw, step_size):
    # Convert to numpy arrays
    begin = np.array(begin)
    end = np.array(end)

    normal = end - begin
    tangent = np.array([normal[1], -normal[0]])
    if not ccw:
        tangent = -tangent

    return compute_arc(begin, end, tangent, step_size)


# Draws a full line between a start and an end point.
# The line is drawn with a step size of step.
def compute_line(begin, end, step_size, point_arr=None):
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

    return model_path(x, y, x_t, y_t, step, step_size, auto_point_turn, point_arr)


# Draws a polygon between a list of vertices.
# The polygon is drawn with a step size of step.
def compute_polygon(vertices, step_size, connect_last=True):
    # Convert to numpy arrays
    vertices = np.array([np.array(vertex) for vertex in vertices])

    poly_points = np.empty((0, 5))
    for vertex in range(0, len(vertices)
                        if connect_last else len(vertices) - 1):

        poly_points = add_points(poly_points,
                                 compute_line(vertices[vertex],
                                              vertices[(vertex + 1)
                                                       % len(vertices)],
                                              step_size, poly_points))

    return poly_points


# Sets up the plot with limits such that all points are visible.
def setup_plot():
    global fig, axd
    fig, axd = plt.subplot_mosaic(
        [[Plt.Main, Plt.Main, Plt.UpperMiddle, Plt.UpperRight],
         [Plt.Main, Plt.Main, Plt.LowerMiddle, Plt.LowerRight]
         ], figsize=(8, 4), layout="constrained")

    x_lim = np.array([points[:, 0].min(), points[:, 0].max()])
    x_dist = x_lim[1] - x_lim[0]
    y_lim = np.array([points[:, 1].min(), points[:, 1].max()])
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
def compute_operation(operation, params, step_size):
    # Continue from the last point if the first point is not defined
    if params[0] == 0:
        params[0] = points[-1][0:2]

    if operation == "Polygon" or operation == 1:
        if isinstance(params[-1], bool):
            return compute_polygon(params[:-1], step_size, params[-1])
        else:
            return compute_polygon(params, step_size)
    elif operation == "Arc" or operation == 2:
        if len(params) == 2:
            return compute_arc(params[0], params[1],
                               previous_slope_vector(points), step_size)
        else:
            return compute_arc(params[0], params[1], params[2], step_size)
    elif operation == "Semicircle" or operation == 3:
        return compute_semicircle(params[0], params[1], params[2], step_size)
    elif operation == "Line" or operation == 4:
        return compute_line(params[0], params[1], step_size)
    elif operation == "Point_Turn" or operation == 5:
        return point_turn((points[-1][0], points[-1][1]),
                          (points[-1][2], points[-1][3]),
                          params[0], ang_step_size)
    else:
        return np.empty((0, 6))


# equivalent of Arduino map()
def val_map(value, i_start, i_stop, o_start, o_stop):
    return o_start + (o_stop - o_start) * ((value - i_start) / (i_stop - i_start))


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
            axes[k].annotate(round(wheel_speed[idx - 1], 3), (.8, 0.5),
                            xycoords='axes fraction', va='center')

def calculate_wheel_speeds(omega, v_x, v_y, a, b):
    return np.array([v_y + v_x - omega * (a + b),
                     v_y - v_x + omega * (a + b),
                     v_y - v_x - omega * (a + b),
                     v_y + v_x + omega * (a + b)])

# Called once at the beginning. Maps the path based on the defined operations.
def start():
    global points, wheel_speeds
    points = np.empty((0, 6))

    for operation in operations:
        points = add_points(points, compute_operation(operation[0],
                                                      operation[1],
                                                      lin_step_size
                                                      if len(operation) < 3
                                                      else operation[2]))

    scaled_vx = points[:, 2] * points[:, 4]
    scaled_vy = points[:, 3] * points[:, 4]
    wheel_speeds = np.transpose(calculate_wheel_speeds(points[:, 5], scaled_vx, scaled_vy, 1, 1))
    global max_speed
    # max speed is the highest value in the wheel speeds
    max_speed = np.max(np.abs(wheel_speeds))

    setup_plot()


# Called for each frame. Draws the point at the current frame.
def update(frame):
    axd[Plt.Main].quiver(points[frame][0], points[frame][1],
              points[frame][2], points[frame][3],
              color='red', scale=1, scale_units='xy', angles='xy')

    plot_wheels(axd, wheel_names, wheel_speeds[frame], max_speed)

    plt.draw()


# Main function to run the simulation.
def main():
    default_file = "../Current/Tests/test.txt"
    n = len(sys.argv)
    if n < 2:
        print("Usage: python3 " + sys.argv[0] + " <instruction_file>")
        file_name = default_file
    else:
        file_name = sys.argv[1]

    o, d = parse_instruction_file(file_name)

    global operations
    operations = o

    global lin_step_size, ang_step_size, time_per_update, auto_point_turn
    lin_step_size = d["LIN_STEP"]
    ang_step_size = d["ARC_STEP"]
    time_per_update = d["TIME_SCALE"]
    auto_point_turn = d["AUTO_TURN"]

    start()

    for i in range(0, len(points)):
        update(i)
        plt.pause(time_per_update)

    plt.show()


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        plt.close()
        print("Simulation stopped.")
        exit(0)
