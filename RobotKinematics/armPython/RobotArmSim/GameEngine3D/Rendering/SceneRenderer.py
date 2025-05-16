import pygame
import numpy as np
from pygame.locals import *
import numba
import scipy

from RobotArmSim.GameEngine3D.Rendering.Camera import Camera
from RobotArmSim.GameEngine3D.Posing.RotationMatrix import RotationMatrix
from RobotArmSim.GameEngine3D.Posing.Transform import Transform
from RobotArmSim.GameEngine3D.Posing.Vector3 import Vector3
from RobotArmSim.GameEngine3D.Rendering.Color import Color

# C compiled methods for efficiency


def project_to_screen_space(vertex, vp_matrix, screen_width, screen_height):
    v = vertex.homogeneous()
    projected = vp_matrix @ v

    if projected[3] == 0:
        return None

    projected /= projected[3]

    # Map from [-1, 1] to screen space
    x = (projected[0] + 1) * screen_width / 2
    y = (1 - projected[1]) * screen_height / 2
    return int(x), int(y), -projected[2]


@numba.njit
def normalize(v):
    norm = np.linalg.norm(v)
    if norm == 0: return v
    return v / norm

@numba.njit
def calculate_lighting(position, normal, base_color, light_matrix, camera_position):
    final_color = np.zeros(3)  # Initialize color (RGB)


    for i in range(light_matrix.shape[0]):  # Iterate over columns (lights)
        light_position = light_matrix[i, 0]  # x, y, z (first row)
        light_color = light_matrix[i, 0]  # r, g, b (second row)
        light_intensity = light_matrix[i, 0]  # intensity (third row)

        # Normalize the vectors
        light_dir = normalize(light_position - position)
        view_dir = normalize(camera_position - position)

        # Ambient lighting (constant illumination independent of position and direction)
        ambient_strength = 0.1
        ambient = ambient_strength * light_color * light_intensity

        # Diffuse lighting (depends on the angle between the light and the surface)
        normal = np.array([normal[0], normal[1], normal[2]], dtype=np.float64)
        light_dir = np.array([light_dir[0], light_dir[1], light_dir[2]], dtype=np.float64)
        diff = np.maximum(np.dot(normal, light_dir), 0.0)
        diffuse = diff * light_color * light_intensity

        # Specular lighting (simulates shiny highlights based on the viewer's position)
        reflect_dir = normalize(
            2 * np.dot(normal, light_dir) * normal - light_dir)
        spec_strength = 0.5
        shininess = 32
        spec = np.maximum(np.dot(view_dir, reflect_dir), 0.0) ** shininess
        specular = spec_strength * spec * light_color * light_intensity

        # Accumulate the lighting from each light source
        val = ambient + diffuse + specular
        final_color += np.array([val[0] * base_color[0],
                                  val[1] * base_color[1],
                                  val[2] * base_color[2]], dtype=np.float64)

    # Clamp the color to be within valid range (0-255)
    final_color = np.clip(final_color, 0, 255).astype(np.uint8)
    return final_color


@numba.njit
def draw_line(p1, p2, color, radius, z_buffer, framebuffer):

    x1, y1, z1 = p1
    x2, y2, z2 = p2
    dx = x2 - x1
    dy = y2 - y1
    dist = int(max(1, (dx ** 2 + dy ** 2) ** 0.5))

    for i in range(dist + 1):
        t = i / dist
        x = int(x1 + dx * t)
        y = int(y1 + dy * t)
        z = z1 * (1 - t) + z2 * t  # linear interpolation of depth

        if 0 <= x < z_buffer.shape[0] and 0 <= y < z_buffer.shape[1]:
            # Check surrounding pixels for thickness
            for oy in range(-radius, radius + 1):
                for ox in range(-radius, radius + 1):
                    xx = x + ox
                    yy = y + oy
                    if (ox ** 2 + oy ** 2 <= radius ** 2 and
                            0 <= xx < z_buffer.shape[0] and 0 <= yy <
                            z_buffer.shape[1]):

                        if z > z_buffer[xx, yy]:
                            z_buffer[xx, yy] = z
                            framebuffer[xx, yy] = color


@numba.njit
def draw_triangle(p0, p1, p2, color, normal, z_buffer, framebuffer, camera_pos, vp_matrix, light_matrix):

    x0, y0, z0 = p0
    x1, y1, z1 = p1
    x2, y2, z2 = p2

    min_x = max(min(x0, x1, x2), 0)
    max_x = min(max(x0, x1, x2), len(framebuffer) - 1)
    min_y = max(min(y0, y1, y2), 0)
    max_y = min(max(y0, y1, y2), len(framebuffer[0]) - 1)

    def edge(a, b, c):
        return (c[0] - a[0]) * (b[1] - a[1]) - (c[1] - a[1]) * (b[0] - a[0])

    for y in range(min_y, max_y):
        for x in range(min_x, max_x):
            p = (x, y)
            w0 = edge((x1, y1), (x2, y2), p)
            w1 = edge((x2, y2), (x0, y0), p)
            w2 = edge((x0, y0), (x1, y1), p)

            # Barycentric coordinates :(
            if w0 >= 0 and w1 >= 0 and w2 >= 0:
                area = edge((x0, y0), (x1, y1), (x2, y2))
                if area == 0: continue
                w0 /= area
                w1 /= area
                w2 /= area

                z = w0 * z0 + w1 * z1 + w2 * z2
                if z > z_buffer[x, y]:
                    z_buffer[x, y] = z
                    framebuffer[x, y] = color

                    # unscaled_x = x / framebuffer.shape[0] * 2 - 1
                    # unscaled_y = y / framebuffer.shape[1] * 2 - 1
                    # screen_pos = np.array([unscaled_x, unscaled_y, z, 1.0], dtype=np.float32)
                    #
                    # world_pos = np.linalg.inv(vp_matrix) @ screen_pos
                    #
                    # framebuffer[x, y] = calculate_lighting(
                    #     world_pos[:3], normal, color, light_matrix, camera_pos)


class SceneRenderer:
    def __init__(self, engine=None, width=800, height=600):
        self.engine = engine
        self.camera = None
        self.aspect = None
        self.screen = None
        self.clock = None
        self.lights = []
        self.width = width
        self.height = height

        self.z_buffer = np.full((self.width, self.height), -np.inf)
        self.framebuffer = np.zeros((self.width, self.height, 3))

    def initialize(self):
        pygame.init()
        self.screen = pygame.display.set_mode((self.width, self.height))
        pygame.display.set_caption("Robot Kinematics Arm Simulator")
        self.clock = pygame.time.Clock()

        self.aspect = self.width / self.height
        self.camera = Camera(transform=Transform(position=Vector3.zero(), rotation=RotationMatrix.identity()),
        target=Vector3.zero(), aspect=self.aspect)


    def add_light(self, light):
        self.lights.append(light)
        return light


    def render(self):
        self.engine.delta_time = self.clock.tick(60) / 1000

        for event in pygame.event.get():
            if event.type == QUIT or (event.type == KEYDOWN and event.key == K_ESCAPE):
                self.engine.running = False


    def post_render(self):
        self.screen.fill(Color.BLACK().val())

        #self.sorted_rendering()
        self.depth_buffer_rendering()

        pygame.display.flip()


    def blit_framebuffer(self):
        pygame.surfarray.blit_array(self.screen, self.framebuffer)


    def depth_buffer_rendering(self):
        self.framebuffer.fill(0)
        self.z_buffer.fill(-np.inf)

        vp = self.camera.get_vp_matrix()

        for updater in self.engine.updaters:
            for mesh in updater.meshes:
                points = [project_to_screen_space(v, vp, self.width, self.height) for v in mesh.vertices()]

                for face, normal, color in zip(mesh.faces, mesh.normals(), mesh.face_colors):
                    pts = [points[point] for point in face]
                    if len(pts) == 3:
                        draw_triangle(pts[0], pts[1], pts[2], color, normal.vector,
                                      self.z_buffer, self.framebuffer,
                                      self.camera.transform.position().vector, vp, self.light_matrix())

                for edge, color in zip(mesh.edges, mesh.edge_colors):
                    p1 = points[edge[0]]
                    p2 = points[edge[1]]
                    draw_line(p1, p2, color, 1, self.z_buffer, self.framebuffer)

        self.blit_framebuffer()

    def light_matrix(self):
        light_matrix = np.empty((len(self.lights), 3, 3), dtype=np.float64)
        for i, light in enumerate(self.lights):
            light_matrix[i, 0] = np.array([*light.transform.position().vector], dtype=np.float64)
            light_matrix[i, 1] = np.array([*light.color.val()], dtype=np.float64)
            light_matrix[i, 2] = np.array([light.intensity, 0, 0], dtype=np.float64)

        return light_matrix
