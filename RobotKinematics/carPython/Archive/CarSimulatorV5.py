import matplotlib.pyplot as plt
import numpy as np
from enum import Enum
from Current.InstructionParser import parse_instruction_file
import sys


# Enum for the operation (string/integer lookup supported)
class Op(Enum):
    Polygon = 1
    Arc = 2
    Semicircle = 3
    Line = 4
    PointTurn = 5


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


# Finds the center of an osculating circle given a start, end, and radius
# Note: not called in this code, but could be used in another architecture
def find_center(begin, end, radius):
    b_to_e = end - begin
    dist = np.linalg.norm(b_to_e)
    midpoint = begin + (b_to_e / 2)
    orth_dist = np.sqrt(radius**2 - (dist / 2)**2)
    center = midpoint + orth_dist * np.array([b_to_e[1], -b_to_e[0]]) / dist

    return center


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

    return np.transpose([*rot_points, *directions, magnitudes])


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
def compute_arc(begin, end, tangent, step_size, point_arr=None):
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

    arc_points = [center[0] + np.linalg.norm(radius) * np.cos(step),
                  center[1] + np.linalg.norm(radius) * np.sin(step)]
    directions = [mult * np.linalg.norm(radius) * -np.sin(step),
                  mult * np.linalg.norm(radius) * np.cos(step)]

    magnitudes = np.linalg.norm(directions, axis=0)
    directions = np.array(directions) / magnitudes

    magnitudes = np.array([step_size for x in range(num_steps)])

    final = np.transpose([*arc_points, *directions, magnitudes])

    if not auto_point_turn:
        return final

    if ((point_arr is not None and len(point_arr) > 0) or
            (point_arr is None and len(points) > 0)):
        current_dir = points[-1][2:4] \
            if point_arr is None else point_arr[-1][2:4]
        target_dir = np.array([directions[0][0], directions[1][0]])
        turn_points = point_turn_vectors((arc_points[0][0],
                                          arc_points[1][0]), current_dir,
                                         target_dir, ang_step_size)
        final = add_points(turn_points, final)

    return final


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

    line_points = np.array([begin[0] + step * (end[0] - begin[0]),
                            begin[1] + step * (end[1] - begin[1])])

    directions = ([end[0] - begin[0] for x in range(num_steps)],
                  [end[1] - begin[1] for x in range(num_steps)])

    magnitudes = np.linalg.norm(directions, axis=0)
    directions = np.array(directions) / magnitudes

    magnitudes = np.array([step_size for x in range(num_steps)])

    final = np.transpose([*line_points, *directions, magnitudes])

    if not auto_point_turn:
        return final

    if ((point_arr is not None and len(point_arr) > 0) or
            (point_arr is None and len(points) > 0)):
        current_dir = points[-1][2:4] \
            if point_arr is None else point_arr[-1][2:4]
        target_dir = np.array([end[0] - begin[0], end[1] - begin[1]])

        # if the directions are the same, no need to turn
        if (np.dot(current_dir, target_dir) ==
                np.linalg.norm(current_dir) * np.linalg.norm(target_dir)):
            return final

        turn_points = point_turn_vectors(
            (line_points[0][0], line_points[1][0]),
            current_dir, target_dir, ang_step_size)
        final = add_points(turn_points, final)

    return final


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
    global fig, ax, colormap
    fig, ax = plt.subplots()

    x_lim = np.array([points[:, 0].min(), points[:, 0].max()])
    x_dist = x_lim[1] - x_lim[0]
    y_lim = np.array([points[:, 1].min(), points[:, 1].max()])
    y_dist = y_lim[1] - y_lim[0]

    ax.set_xlim(x_lim[0] - 0.3 * x_dist, x_lim[1] + 0.3 * x_dist)
    ax.set_ylim(y_lim[0] - 0.3 * y_dist, y_lim[1] + 0.3 * y_dist)

    ax.set_title("Kinematics simulation")
    plt.axis('scaled')


# Adds a set of points to the current set of points.
def add_points(original, to_add):
    return np.append(original, to_add, axis=0)


# Computes the operation given the operation type, the parameters (varies based
# on operation), and the step value.
def compute_operation(operation, params, step_size):
    # Continue from the last point if the first point is not defined
    if params[0] == 0:
        params[0] = points[-1][0:2]

    if operation == Op.Polygon or operation == "Polygon" or operation == 1:
        if isinstance(params[-1], bool):
            return compute_polygon(params[:-1], step_size, params[-1])
        else:
            return compute_polygon(params, step_size)
    elif operation == Op.Arc or operation == "Arc" or operation == 2:
        if len(params) == 2:
            return compute_arc(params[0], params[1],
                               previous_slope_vector(points), step_size)
        else:
            return compute_arc(params[0], params[1], params[2], step_size)
    elif (operation == Op.Semicircle or operation == "Semicircle" or
          operation == 3):
        return compute_semicircle(params[0], params[1], params[2], step_size)
    elif operation == Op.Line or operation == "Line" or operation == 4:
        return compute_line(params[0], params[1], step_size)
    elif (operation == Op.PointTurn or operation == "PointTurn" or
          operation == 5):
        return point_turn((points[-1][0], points[-1][1]),
                          (points[-1][2], points[-1][3]),
                          params[0], ang_step_size)
    else:
        return np.empty((0, 5))


# Called once at the beginning. Maps the path based on the defined operations.
def start():
    global points
    points = np.empty((0, 5))

    for operation in operations:
        points = add_points(points, compute_operation(operation[0],
                                                      operation[1],
                                                      lin_step_size
                                                      if len(operation) < 3
                                                      else operation[2]))

    setup_plot()


# Called for each frame. Draws the point at the current frame.
def update(frame):
    ax.quiver(points[frame][0], points[frame][1],
              points[frame][2], points[frame][3],
              color='red', scale=1, scale_units='xy', angles='xy')
    plt.draw()


# Main function to run the simulation.
def main():
    n = len(sys.argv)
    if n < 2:
        print("Usage: python3 CarSimulatorV5.py <instruction_file>")
        exit(1)
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
