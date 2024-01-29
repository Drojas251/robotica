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
        self._color = 'blue'

    def build_shape(self):
        pass

    def collision(self):
        pass

    def clear_shape(self):
        pass

    def set_color(self, color):
        self._color = color

class Rectangle(Shape):
    def __init__(self, fig, ax, origin, size):
        Shape.__init__(self, fig, ax, origin)
        self.l = size[0]
        self.h = size[1]
        self.square = None

    def build_shape(self):
        self.square = plt.Rectangle(self.origin, self.l, self.h, linewidth=2, facecolor=self._color)
        self.ax.add_patch(self.square)

    def collision(self):
        self.square.set_facecolor(self.collision_color)
        self.fig.canvas.draw()

    def clear_shape(self):
        self.square.remove()

class VisTF:
    def __init__(self, ax):
        self.ax = ax
        self.x_axis, = ax.plot([], [], lw=1, color='red')
        self.y_axis, = ax.plot([], [], lw=1, color='green')
        self.axis_len = 0.075

    def show_x_axis(self, tf):
        pt = np.array([[self.axis_len], [0], [0], [1.0]])
        transformed_point = np.dot(tf, pt)
        transformed_x, transformed_y, _, _ = transformed_point.flatten()
        self.x_axis.set_data([tf[0][3], transformed_x], [tf[1][3], transformed_y])

    def show_y_axis(self, tf):
        pt = np.array([[0], [self.axis_len], [0], [1.0]])
        transformed_point = np.dot(tf, pt)
        transformed_x, transformed_y, _, _ = transformed_point.flatten()
        self.y_axis.set_data([tf[0][3], transformed_x], [tf[1][3], transformed_y])
    
    def show_tf(self, tf):
        self.show_x_axis(tf)
        self.show_y_axis(tf)

    def clear_tf(self):
        self.x_axis.set_data([], [])
        self.y_axis.set_data([], [])

class VisTFTree:
    def __init__(self, tftree, ax):
        self.tftree = tftree
        self.ax = ax
        self.num_frames = self.tftree.num_joints

        self.vis_tfs = {}
        for i in range(self.num_frames):
            self.vis_tfs[i] = VisTF(self.ax)

    def vis_tf_tree(self):
        for i in range(self.num_frames):
            tf = self.tftree.get_base_transform(i+1)
            self.vis_tfs[i].show_tf(tf)

    def clear_tf_tree(self):
        for i in range(self.num_frames):
            self.vis_tfs[i].clear_tf()


class Visualization():
    def __init__(self, robot_model, tftree):

        self.plt = plt
        self.figure, ax = self.plt.subplots()
        ax.set_xlim(-1, 1)
        ax.set_ylim(-1, 1)

        self.ax = ax

        # Arm Visual
        self.arm_links, = ax.plot([], [], lw=5, color='grey')
        self.arm_joints, = ax.plot([], [], color='grey', marker='o', markersize=8, markerfacecolor='red', markeredgecolor='black')
        self.gripper, = ax.plot([], [], color='black', lw=4)
        self.gripper_pts = [
            np.array([[0.02], [0.04], [0.0], [1.0]]),
            np.array([[0.0], [0.04], [0.0], [1.0]]),
            np.array([[0.0], [-0.04], [0.0], [1.0]]),
            np.array([[0.02], [-0.04], [0.0], [1.0]]),
        ]
        
        self.arm_stand = None
        self.arm_base = None

        # Task Space Trajectory Visual
        self.trajectory, = ax.plot([], [], lw=2)

        # Task Space Planned Path Visual
        self.planned_path, = ax.plot([], [], lw=1, color='green', marker='o', markersize=8)

        # WS Visual
        self.ws, = self.plt.plot([], [],'o', c=[0,1,0,0.1], alpha=0.05)

        # Robot Init Params
        self.robot_model = robot_model
        self.DH_params = self.robot_model.DH_params
        self.link_length_1 = self.DH_params.a[0]
        self.link_length_2 = self.DH_params.a[1]
        self.tftree = tftree

        self.collision_objs = {}

        networking_params = NetworkingParams() 
        topic, port = networking_params.get_pub_sub_info("path_publisher")
        self.path_listener = RoboticaSubscriber(port=port, topic=topic)
        self.path_listener.subscribe(callback=self._path_listener_callback)

        self.show_tftree = False
        self.vis_tf_tree = VisTFTree(self.tftree, self.ax)

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

        # Arm Links
        self.arm_links.set_data(x_values, y_values)

        # Arm Joints
        self.arm_joints.set_data(x_values[:-1], y_values[:-1])

        # Gripper
        gripper_x = []
        gripper_y = []
        tf = self.tftree.get_base_transform(self.tftree.num_joints)
        for pt in self.gripper_pts:
            transformed_point = np.dot(tf, pt)
            transformed_x, transformed_y, _, _ = transformed_point.flatten()
            gripper_x.append(transformed_x)
            gripper_y.append(transformed_y)

        self.gripper.set_data(gripper_x, gripper_y)

        if self.arm_base is None:
            self.arm_base = Rectangle(self.figure, self.ax, (-0.05,-0.1), (0.1, 0.01))
            self.arm_base.set_color("black")
            self.arm_base.build_shape()

        if self.arm_stand is None:
            self.arm_stand = Rectangle(self.figure, self.ax, (-0.025,-0.1), (0.05, 0.1))
            self.arm_stand.set_color("grey")
            self.arm_stand.build_shape()

        if self.show_tftree:
            self.vis_tf_tree.vis_tf_tree()
        else:
            self.vis_tf_tree.clear_tf_tree()
            
    def visualize_cartesian_trajectory(self, cartesian_traj):
        x, y = self._format_path_data(cartesian_traj)
        self.trajectory.set_data(x, y)

    def visualize_workspace(self):
        self.ws.set_data(self.robot_model.x_p, self.robot_model.y_p)

    def clear_workspace(self):
        self.ws.set_data([], [])

    def visualize_tftree(self):
        self.show_tftree = True
        self.visualize_arm()

    def clear_tftree(self):
        self.show_tftree = False
        self.visualize_arm()

    def add_rectangle_obj(self, name, origin, size):
        rectangle = Rectangle(self.figure, self.ax, origin, size)
        rectangle.build_shape()
        self.collision_objs[name] = rectangle

    def remove_rectangle_obj(self, name):
        self.collision_objs[name].clear_shape()
        del self.collision_objs[name]

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