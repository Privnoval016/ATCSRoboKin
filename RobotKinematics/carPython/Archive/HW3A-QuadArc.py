import matplotlib.pyplot as plt
import numpy as np
from enum import Enum


# Enum for the operation (string/integer lookup supported)
class Op(Enum):
    Polygon = 1
    Arc = 2
    Semicircle = 3
    Line = 4


# Default parameters
step_size = 0.5
ang_step_size = 10      # not used currently
time_per_update = 0.05


# List of operations to perform (edit this to change the path)
# Defined as (Operation Type, [Parameters], Optional Step Size)
# Polygon: [(point 1), (point 2), ..., (optional) connect_ends]
# Arc: [(begin), (end), (tangent)] (tangent is optional)
# Semicircle: [(begin), (end), CCW] (CCW is a boolean)
# Line: [(begin), (end)]
operations = [
    (Op.Polygon, [(0, 0), (4, 0), (5, 3), (-2, 3.5)]),
    (Op.Semicircle, [(0, 0), (4, 0), False]),
    (Op.Arc, [(4, 0), (5, -1), (1, 0)]),
    (Op.Line, [(5, -1), (5, -3)]),
    (Op.Arc, [(5, -3), (4, -4)]),
    (Op.Line, [(4, -4), (4, -5)]),
    (Op.Semicircle, [(4, -5), (2, -5), False]),
    (Op.Semicircle, [(2, -5), (0, -5), False]),
    (Op.Line, [(0, -5), (0, -2)]),
    (Op.Line, [(0, -2), (1, -2)]),
    (Op.Semicircle, [(1, -2), (1, 0), True]),
    (Op.Line, [(1, 0), (0, 0)]),
    (Op.Arc, [(0, 0), (0, -2), (-0.71, -0.71)]),
]


# Finds the slope unit vector between the last two points in the given list.
def previous_slope_vector(passed_points):
    return ((passed_points[-1] - passed_points[-2]) /
            np.linalg.norm(passed_points[-1] - passed_points[-2]))


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


# Draws an arc between a start and an end point with an initial tangent vector.
# The arc is drawn with arc length steps of step.
def compute_arc(begin, end, tangent, step):
    # Convert to numpy arrays
    begin = np.array(begin)
    end = np.array(end)
    tangent = np.array(tangent)

    # If a line is defined, return the line
    if (np.cross(end - begin, tangent) == 0
            and np.dot(end - begin, tangent) > 0):
        return compute_line(begin, end, step)

    center, radius = find_center_with_tangent(begin, end, tangent)

    omega = np.degrees(step / np.linalg.norm(radius))

    begin_angle = np.arctan2(begin[1] - center[1], begin[0] - center[0])
    end_angle = np.arctan2(end[1] - center[1], end[0] - center[0])

    if np.cross(radius, tangent) > 0:
        if end_angle < begin_angle:
            end_angle += 2 * np.pi
    else:
        if end_angle > begin_angle:
            end_angle -= 2 * np.pi

    num_steps = int(np.abs(begin_angle - end_angle) / np.radians(omega)) + 1
    step = np.linspace(begin_angle, end_angle, num_steps)

    arc_points = [center[0] + np.linalg.norm(radius) * np.cos(step),
                  center[1] + np.linalg.norm(radius) * np.sin(step)]

    return np.transpose(arc_points)


# Draws a semicircle between a start and an end point in the given direction.
# The semicircle is drawn with arc length steps of step.
def compute_semicircle(begin, end, ccw, step):
    # Convert to numpy arrays
    begin = np.array(begin)
    end = np.array(end)

    normal = end - begin
    tangent = np.array([normal[1], -normal[0]])
    if not ccw:
        tangent = -tangent

    return compute_arc(begin, end, tangent, step)


# Draws a full line between a start and an end point.
# The line is drawn with a step size of step.
def compute_line(begin, end, step):
    # Convert to numpy arrays
    begin = np.array(begin)
    end = np.array(end)

    num_steps = int(np.linalg.norm(end - begin) / step) + 1
    step = np.linspace(0, 1, num_steps)

    line_points = np.array([begin[0] + step * (end[0] - begin[0]),
                            begin[1] + step * (end[1] - begin[1])])

    return np.transpose(line_points)


# Draws a polygon between a list of vertices.
# The polygon is drawn with a step size of step.
def compute_polygon(vertices, step, connect_last=True):
    # Convert to numpy arrays
    vertices = np.array([np.array(vertex) for vertex in vertices])

    return np.concatenate(
        [compute_line(vertices[vertex],
                      vertices[(vertex + 1) % len(vertices)], step)
            for vertex in range(0, len(vertices)
                                if connect_last else len(vertices) - 1)])


# Sets up the plot with limits such that all points are visible.
def setup_plot():
    global fig, ax
    fig, ax = plt.subplots()

    x_lim = np.array([points[:, 0].min(), points[:, 0].max()])
    x_dist = x_lim[1] - x_lim[0]
    y_lim = np.array([points[:, 1].min(), points[:, 1].max()])
    y_dist = y_lim[1] - y_lim[0]

    ax.set_xlim(x_lim[0] - 0.3 * x_dist, x_lim[1] + 0.3 * x_dist)
    ax.set_ylim(y_lim[0] - 0.3 * y_dist, y_lim[1] + 0.3 * y_dist)

    ax.set_title("Kinematics simulation - lines and arcs")
    plt.axis('scaled')


# Adds a set of points to the current set of points.
def add_points(original, to_add):
    return np.append(original, to_add, axis=0)


# Computes the operation given the operation type, the parameters (varies based
# on operation), and the step value.
def compute_operation(operation, params, step):
    if operation == Op.Polygon or operation == "Polygon" or operation == 1:
        if isinstance(params[-1], bool):
            return compute_polygon(params[:-1], step, params[-1])
        else:
            return compute_polygon(params, step)
    elif operation == Op.Arc or operation == "Arc" or operation == 2:
        if len(params) == 2:
            return compute_arc(params[0], params[1],
                               previous_slope_vector(points), step)
        else:
            return compute_arc(params[0], params[1], params[2], step)
    elif (operation == Op.Semicircle or operation == "Semicircle" or
          operation == 3):
        return compute_semicircle(params[0], params[1], params[2], step)
    elif operation == Op.Line or operation == "Line" or operation == 4:
        return compute_line(params[0], params[1], step)
    else:
        return np.empty((0, 2))


# Called once at the beginning. Maps the path based on the defined operations.
def start():
    global points
    points = np.empty((0, 2))

    for operation in operations:
        points = add_points(points, compute_operation(operation[0],
                                                      operation[1],
                                                      step_size
                                                      if len(operation) < 3
                                                      else operation[2]))

    setup_plot()


# Called for each frame. Draws the point at the current frame.
def update(frame):
    ax.scatter(points[frame][0], points[frame][1], color='red')
    plt.draw()


# Main function to run the simulation.
def main():
    start()

    for i in range(0, len(points)):
        update(i)
        plt.pause(time_per_update)

    plt.show()


if __name__ == "__main__":
    main()
