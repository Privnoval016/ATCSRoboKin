import numpy as np

from RobotArmSim.GameEngine3D.Rendering.Color import Color
from RobotArmSim.GameEngine3D.Posing.Vector3 import Vector3


class Mesh:
    def __init__(self, vertices, edges, transform=None):
        if not all(isinstance(v, Vector3) for v in vertices):
            raise TypeError("Vertices must be a list of Vector3 objects")
        self.local_vertices = vertices
        self.edges = edges
        self.transform = transform
        self.color = Color.WHITE().to_tuple()

    def vertices(self):
        if self.transform is not None:
            matrix = self.transform.homogeneous_matrix
            h_vertices = np.array([v.to_homogeneous() for v in self.local_vertices])  # shape (n, 4)
            global_vertices_h = (matrix @ h_vertices.T).T  # shape (n, 4)
            x = global_vertices_h[:, :3] / global_vertices_h[:, 3:]

            return [Vector3(*v) for v in x]  # Convert back to Vector3

        else:
            return self.local_vertices


    def set_color(self, color):
        self.color = color.to_tuple()
        pass

    @staticmethod
    def cube(half_extents=Vector3.one(), transform=None):
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
        return Mesh(vertices, edges, transform)


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
        m.set_color(Color.MAGENTA())
        return m

    @staticmethod
    def sphere(radius=1.0, stacks=5, sectors=5, transform=None):
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

        return Mesh([Vector3(*v) for v in vertices], edges, transform)


    @staticmethod
    def obj_wireframe(filepath, transform=None):
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

        return Mesh([Vector3(*v) for v in vertices], list(edges), transform)