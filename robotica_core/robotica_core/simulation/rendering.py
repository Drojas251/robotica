import numpy as np
import matplotlib.pyplot as plt
from robotica_core.utils.robotica_networking import RoboticaSubscriber
from robotica_core.utils.yml_parser import NetworkingParams


class Shape:
    def __init__(self, fig, ax, origin): 
        self.origin = origin
        self.ax = ax
        self.fig = fig
        self.collision_color = 'red'
        self.default_color = 'blue'

class Rectangle(Shape):
    def __init__(self, fig, ax, origin, size):
        Shape.__init__(self, fig, ax, origin)
        self.l = size[0]
        self.h = size[1]

    def build_shape(self):
        self.square = plt.Rectangle(self.origin, self.l, self.h, linewidth=2, facecolor=self.default_color)
        self.ax.add_patch(self.square)

    def collision(self):
        self.square.set_facecolor(self.collision_color)
        self.fig.canvas.draw()

class Visualization():
    def __init__(self, DH_params, tftree):

        self.plt = plt
        self.figure, ax = self.plt.subplots()
        ax.set_xlim(-1, 1)
        ax.set_ylim(-1, 1)

        self.ax = ax

        # Arm Visual
        self.arm_line, = ax.plot([], [], lw=5)

        # Task Space Trajectory Visual
        self.trajectory, = ax.plot([], [], lw=2)

        # Task Space Planned Path Visual
        self.planned_path, = ax.plot([], [], lw=1, color='green', marker='o', markersize=8)

        # Robot Init Params
        self.DH_params = DH_params
        self.link_length_1 = self.DH_params.a[0]
        self.link_length_2 = self.DH_params.a[1]
        self.tftree = tftree

        self.collision_objs = {}

        networking_params = NetworkingParams() 
        topic, port = networking_params.get_pub_sub_info("path_publisher")
        self.path_listener = RoboticaSubscriber(port=port, topic=topic)
        self.path_listener.subscribe(callback=self._path_listener_callback)

    def run(self):
        self.plt.show()

    def init_space(self):
        theta1 = self.DH_params.theta[0]
        theta2 = self.DH_params.theta[1]
        self.tftree.set_joints([theta1, theta2])
        self.visualize_arm()

    def visualize_arm(self):
        tree = self.tftree.get_tree()
        x_values = []
        y_values = []
        for pt in tree:
            x_values.append(pt[0])
            y_values.append(pt[1])

        self.arm_line.set_data(x_values, y_values)

    def visualize_cartesian_trajectory(self, cartesian_traj):
        x, y = self._format_path_data(cartesian_traj)
        self.trajectory.set_data(x, y)

    def add_rectangle_obj(self, name, origin, size):
        rectangle = Rectangle(self.figure, self.ax, origin, size)
        rectangle.build_shape()
        self.collision_objs[name] = rectangle

    def vis_collision(self, obj_name):
        obj = self.collision_objs[obj_name]
        obj.collision()

    def _format_path_data(self, cartesian_traj):
        x = [p.point[0] for p in cartesian_traj]
        y = [p.point[1] for p in cartesian_traj]

        return x, y

    def _path_listener_callback(self, data):
        x = []
        y = []
        for wpt in data:
            point = wpt.point
            x.append(point[0])
            y.append(point[1])

        self.planned_path.set_data(x,y)