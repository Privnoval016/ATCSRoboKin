from collections import defaultdict, deque

import numpy as np

from RobotArmSim.GameEngine3D.Rendering.Color import Color
from RobotArmSim.GameEngine3D.Posing.Vector3 import Vector3


# Utility functions


def is_convex(polygon, vertices):
    num_vertices = len(polygon)
    for i in range(num_vertices):
        p0, p1, p2 = vertices[polygon[i - 1]], vertices[polygon[i]], \
        vertices[polygon[(i + 1) % num_vertices]]

        cross_product = (p1[0] - p0[0]) * (p2[1] - p1[1]) - (
                    p1[1] - p0[1]) * (p2[0] - p0[0])

        # cross product is positive, counter-clockwise
        # cross product is negative, clockwise
        # cross product is zero, collinear
        if cross_product < 0:
            return False  # Not convex if any clockwise
    return True  # Convex if all turns are counter-clockwise


def triangulate_convex_n_gon(polygon, vertices):
    triangles = []
    first_vertex = polygon[0]

    for i in range(1, len(polygon) - 1):
        triangle = [first_vertex, polygon[i], polygon[i + 1]]
        triangles.append(triangle)

    return triangles


def ear_clip_triangulation(polygon, vertices):

    def is_ear(polygon, i, vertices):
        p0, p1, p2 = vertices[polygon[i - 1]], vertices[polygon[i]], \
        vertices[polygon[(i + 1) % len(polygon)]]

        # TODO: Check if middle vertex and no other vertex is inside
        #       the triangle at some point

        return True

    triangles = []
    polygon_copy = polygon[:]

    while len(polygon_copy) > 3:
        for i in range(len(polygon_copy)):
            if is_ear(polygon_copy, i, vertices):
                ear_triangle = [polygon_copy[i - 1], polygon_copy[i],
                                polygon_copy[(i + 1) % len(polygon_copy)]]
                triangles.append(ear_triangle)
                del polygon_copy[i]
                break

    triangles.append(polygon_copy)

    return triangles


def break_n_gon_into_triangles(polygon, vertices):
    if is_convex(polygon, vertices):
        return triangulate_convex_n_gon(polygon, vertices)
    else:
        return ear_clip_triangulation(polygon, vertices)


def build_edge_graph(edges):
    graph = defaultdict(list)
    for a, b in edges:
        graph[a].append(b)
        graph[b].append(a)
    return graph


def is_planar(cycle, vertices, tol=1e-5):
    if len(cycle) < 3:
        return False
    v0 = np.array(vertices[cycle[0]], dtype=float)
    v1 = np.array(vertices[cycle[1]], dtype=float)
    v2 = np.array(vertices[cycle[2]], dtype=float)

    normal = np.cross(v1 - v0, v2 - v0)
    if np.linalg.norm(normal) < tol:
        return False  # Degenerate normal (no plane)
    normal /= np.linalg.norm(normal)

    for i in range(3, len(cycle)):
        vi = np.array(vertices[cycle[i]], dtype=float)
        if abs(np.dot(normal, vi - v0)) > tol:
            return False  # Non-planar vertex
    return True


def has_chord(cycle, edge_set):
    cycle_len = len(cycle)
    for i in range(cycle_len):
        for j in range(i + 2, cycle_len):
            if (j + 1) % cycle_len == i:
                continue  # skip adjacent vertices
            a, b = cycle[i], cycle[j]
            if (a, b) in edge_set or (b, a) in edge_set:
                return True
    return False


def find_faces(vertices, edges, max_cycle_length=10):
    vertices = [v.vector for v in vertices]

    edge_graph = build_edge_graph(edges)
    edge_set = set(map(tuple, map(sorted, edges)))
    faces = set()

    for start in range(len(vertices)):
        for neighbor in edge_graph[start]:
            stack = deque()
            stack.append(([start, neighbor], neighbor))
            while stack:
                path, current = stack.pop()
                if len(path) > max_cycle_length:
                    continue
                for next_vert in edge_graph[current]:
                    if next_vert == path[0] and len(path) >= 3:
                        cycle = path[:]
                        if (not has_chord(cycle, edge_set) and
                                is_planar(cycle, vertices)):
                            face_key = tuple(sorted(cycle))
                            if face_key not in faces:
                                faces.add(face_key)
                    elif next_vert not in path:
                        stack.append((path + [next_vert], next_vert))
    cycles = [list(face) for face in faces]
    c = [cycle if len(cycles) == 3 else break_n_gon_into_triangles(cycle, vertices) for cycle in cycles]
    return [item for sublist in c for item in sublist]


def ensure_outward_normals(vertices, faces):
    def compute_face_normal(v0, v1, v2):
        edge1 = v1 - v0
        edge2 = v2 - v0
        normal = edge1.cross(edge2)
        return normal.normalized()

    face_normals = []

    for face in faces:
        v0, v1, v2 = vertices[face[0]], vertices[face[1]], vertices[face[2]]

        normal = compute_face_normal(v0, v1, v2)

        # Invert the normal if the face is defined in clockwise (CW) order
        if normal.dot((v1 - v0).cross(v2 - v0)) < 0:
            normal = -normal  # Flip the normal

        face_normals.append(normal)

    return face_normals


# Mesh class

class Mesh:
    def __init__(self, vertices, edges, transform=None, wireframe=False):
        if not all(isinstance(v, Vector3) for v in vertices):
            raise TypeError("Vertices must be a list of Vector3 objects")
        self.local_vertices = vertices
        self.edges = edges
        self.faces = find_faces(vertices, edges) if not wireframe else []
        self.local_normals = ensure_outward_normals(vertices, self.faces) if not wireframe else []
        self.transform = transform
        self.face_colors = [Color.WHITE().val() for _ in range(len(self.faces))]
        self.edge_colors = [Color.WHITE().val() for _ in range(len(self.edges))]


    def normals(self):
        if len(self.local_normals) == 0:
            return []
        if self.transform is not None:
            matrix = self.transform.homogeneous_matrix
            h_normals = np.array([n.homogeneous() for n in self.local_normals])
            global_normals_h = (matrix @ h_normals.T).T
            x = global_normals_h[:, :3] / global_normals_h[:, 3:]

            return [Vector3(*v) for v in x]

        else:
            return self.local_normals

    def vertices(self):
        if self.transform is not None:
            matrix = self.transform.homogeneous_matrix
            h_vertices = np.array([v.homogeneous() for v in self.local_vertices])
            global_vertices_h = (matrix @ h_vertices.T).T
            x = global_vertices_h[:, :3] / global_vertices_h[:, 3:]

            return [Vector3(*v) for v in x]

        else:
            return self.local_vertices


    def set_color(self, color):
        self.edge_colors = [color.val() for _ in range(len(self.edges))]
        self.face_colors = [color.val() for _ in range(len(self.faces))]
        pass

    def set_face_color(self, color, index):
        if index < len(self.face_colors):
            self.face_colors[index] = color.val()
            return True
        else:
            return False

    def set_edge_color(self, color, index):
        if index < len(self.edge_colors):
            self.edge_colors[index] = color.val()
            return True
        else:
            return False

    def scale_by(self, scale):
        self.local_vertices = [v * scale for v in self.local_vertices]
        return self


    @staticmethod
    def cube(half_extents=Vector3.one(), transform=None, wireframe=False):
        vertices = [
            Vector3(-half_extents.x(), -half_extents.y(), -half_extents.z()),
            Vector3(half_extents.x(), -half_extents.y(), -half_extents.z()),
            Vector3(half_extents.x(), half_extents.y(), -half_extents.z()),
            Vector3(-half_extents.x(), half_extents.y(), -half_extents.z()),
            Vector3(-half_extents.x(), -half_extents.y(), half_extents.z()),
            Vector3(half_extents.x(), -half_extents.y(), half_extents.z()),
            Vector3(half_extents.x(), half_extents.y(), half_extents.z()),
            Vector3(-half_extents.x(), half_extents.y(), half_extents.z())
        ]
        edges = [
            (0, 1), (1, 2), (2, 3), (3, 0),
            (4, 5), (5, 6), (6, 7), (7, 4),
            (0, 4), (1, 5), (2, 6), (3, 7)
        ]
        return Mesh(vertices, edges, transform, wireframe)


    @staticmethod
    def basis(length=1, transform=None):
        vertices = [
            Vector3(0, 0, 0),
            Vector3(1, 0, 0),
            Vector3(0, 1, 0),
            Vector3(0, 0, 1)
        ]
        vertices = [v * length for v in vertices]

        edges = [
            (0, 1), (0, 2), (0, 3)
        ]
        m = Mesh(vertices, edges, transform)

        m.set_edge_color(Color.RED(), 0)
        m.set_edge_color(Color.GREEN(), 1)
        m.set_edge_color(Color.BLUE(), 2)

        return m

    @staticmethod
    def sphere(radius=1.0, stacks=5, sectors=5, transform=None, wireframe=False):
        vertices = []
        edges = []

        for i in range(stacks + 1):
            stack_angle = np.pi / 2 - i * np.pi / stacks
            xy = radius * np.cos(stack_angle)
            z = radius * np.sin(stack_angle)

            for j in range(sectors + 1):
                sector_angle = j * 2 * np.pi / sectors
                x = xy * np.cos(sector_angle)
                y = xy * np.sin(sector_angle)
                vertices.append([x, y, z])

        for i in range(stacks):
            for j in range(sectors):
                curr = i * (sectors + 1) + j
                following = curr + sectors + 1

                # horizontal edge
                edges.append((curr, curr + 1))
                # vertical edge
                edges.append((curr, following))

        return Mesh([Vector3(*v) for v in vertices], edges, transform, wireframe)


    @staticmethod
    def obj_wireframe(filepath, transform=None, wireframe=False):
        vertices = []
        edges = set()

        with open(filepath, 'r') as f:
            for line in f:
                parts = line.split()

                if len(parts) == 0:
                    continue

                if parts[0] == 'v':
                    x, y, z = map(float, parts[1:4])
                    vertices.append((x, y, z))

                elif parts[0] == 'f':
                    face_indices = [int(p.split('/')[0]) - 1 for p in parts[1:]]

                    for i in range(len(face_indices)):
                        a = face_indices[i]
                        b = face_indices[
                            (i + 1) % len(face_indices)]
                        edge = tuple(sorted((a, b)))
                        edges.add(edge)

        return Mesh([Vector3(*v) for v in vertices], list(edges), transform, wireframe)

    @staticmethod
    def obj_to_faces(filepath, transform=None):
        vertices = []
        faces = []

        # retrieve all vertex coordinates and face indices

        with open(filepath, 'r') as f:
            for line in f:
                parts = line.split()

                if len(parts) == 0:
                    continue

                if parts[0] == 'v':
                    x, y, z = map(float, parts[1:4])
                    vertices.append((x, y, z))

                elif parts[0] == 'f':
                    face_indices = [int(p.split('/')[0]) - 1 for p in parts[1:]]
                    faces.append(face_indices)

        return vertices, faces