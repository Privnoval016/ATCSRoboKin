import numpy as np

def stl_decompress(stl):
    # return list of (vertices, edges, faces)

    vertices = []
    edges = set()
    faces = []

    for facet in stl:
        v1 = tuple(facet[0])
        v2 = tuple(facet[1])
        v3 = tuple(facet[2])

        for v in [v1, v2, v3]:
            if v not in vertices:
                vertices.append(v)

        i1 = vertices.index(v1)
        i2 = vertices.index(v2)
        i3 = vertices.index(v3)

        faces.append((i1, i2, i3))

        edges.add(tuple(sorted((i1, i2))))
        edges.add(tuple(sorted((i2, i3))))
        edges.add(tuple(sorted((i3, i1))))


def intersection(vertex1, vertex2, plane_point, plane_normal):

    line_point = np.array(vertex1)
    line_dir = np.array(vertex2) - np.array(vertex1)


    plane_point = np.array(plane_point)
    plane_normal = np.array(plane_normal)

    plane_normal = plane_normal / np.linalg.norm(plane_normal)

    denom = np.dot(plane_normal, line_dir)
    if np.abs(denom) < 1e-6:
        return None  # Line is parallel to the plane

    d = np.dot(plane_normal, plane_point - line_point) / denom
    if d < 0 or d > 1:
        return None  # Intersection is outside the segment

    intersection_point = line_point + d * line_dir
    return intersection_point
