import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import sys
import os
import yaml

from robotica_core.utils.yml_parser import RobotParamsLoader
from robotica_core.utils.robotica_networking import RoboticaPublisher
from robotica_core.control.controller_manager import ControllerManager
from robotica_datatypes.kinematic_datatypes.DH_params import DH_parameters    
from robotica_core.kinematics.tftree import TFTree

from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
import tkinter as tk
from tkinter import ttk
import matplotlib.animation as animation
from tkinter import Frame,Label,Entry,Button

class Visualization():
    def __init__(self, DH_params):

        self.plt = plt
        self.figure, ax = self.plt.subplots()
        ax.set_xlim(-1, 1)
        ax.set_ylim(-1, 1)

        # Arm Visual
        self.arm_line, = ax.plot([], [], lw=5)

        # Task Space Path Visual
        self.path, = ax.plot([], [], lw=2)

        # Robot Init Params
        self.DH_params = DH_params
        self.link_length_1 = self.DH_params.a[0]
        self.link_length_2 = self.DH_params.a[1]
        self.tftree = TFTree(DH_params)

    def run(self):
        self.plt.show()

    def init_space(self):
        theta1 = self.DH_params.theta[0]
        theta2 = self.DH_params.theta[1]
        self.visualize_arm(theta1, theta2)

    def visualize_arm(self, theta1, theta2):
        self.tftree.set_joints([theta1, theta2])
        tree = self.tftree.get_tree()
        x_values = []
        y_values = []
        for pt in tree:
            x_values.append(pt[0])
            y_values.append(pt[1])

        self.arm_line.set_data(x_values, y_values)

    def visualize_cartesian_path(self, cartesian_traj):
        self._format_path_data(cartesian_traj)
        self.path.set_data(self.x, self.y)

    def _format_path_data(self, cartesian_traj):
        self.x = [p.point[0] for p in cartesian_traj]
        self.y = [p.point[1] for p in cartesian_traj]


class PluginOption:
    def __init__(self, master, group_name, plugins):
        self.master = master
        self.group_name = group_name
        self.plugins = plugins

        self.label = None
        self.dropdown = None
        self.plugin_selected = "None"

    def make_label(self, row, col):
        self.label = Label(self.master, text=self.group_name, width=25)
        self.label.grid(row=row, column=col)

    def make_drop_down(self, row, col):
        if len(self.plugins) > 0:
            self.plugin_selected = tk.StringVar(value=self.plugins[0])
        else:
            self.plugin_selected = tk.StringVar(value="None")

        self.dropdown = ttk.Combobox(self.master, textvariable=self.plugin_selected, values=self.plugins, state="readonly")
        self.dropdown.grid(row=row, column=col)

class SimApp(Frame):
    PLUGIN_FILE_PATH = "~/.robotica/plugins"
    SELECTED_PLUGIN_FILE = "run_plugins.yml"

    def __init__(self, DH_params, master = None):
        Frame.__init__(self, master)
        self.master = master

        self.vis_scene = Visualization(DH_params)
        self.controller_manager = ControllerManager()
        self.joint_publisher = RoboticaPublisher(port="5153", topic="joint_publisher")
        self.curr_joints = DH_params.theta
        self.sim_interval = 0.01

        # Plugins
        self.plugin_dict = {}
        self.plugin_menu = {}
        self._read_plugin_data()
        
        self._init_gui()

    def _update(self, frame):
        if not self.controller_manager.is_executing_path():
            traj = self.controller_manager.wait_for_joint_trajectory(1000)
            if traj:
                self.vis_scene.visualize_cartesian_path(traj)
            else:
                # Publish joint position
                self._publish_joints()
                return

        # Get next wpts to execute 
        theta1, theta2 = self.controller_manager.consume_wpt()

        # Move Arm
        self.vis_scene.visualize_arm(theta1, theta2)

        self.curr_joints = (theta1, theta2)
        self._publish_joints()

    def _set_plugins(self):
        run_plugins = {}
        for group_name, plugin_options in self.plugin_menu.items():
            run_plugins[group_name] = plugin_options.plugin_selected.get()
            print(f"{group_name}: {plugin_options.plugin_selected.get()}")

        shared_plugin_path = os.path.expanduser(self.PLUGIN_FILE_PATH)
        yaml_file_path = os.path.join(shared_plugin_path, self.SELECTED_PLUGIN_FILE)

        with open(yaml_file_path, "w") as yaml_file:
            yaml.dump(run_plugins, yaml_file, default_flow_style=False)

    def _init_gui(self):
        self.master.title("Use Of FuncAnimation in tkinter based GUI")
        self.pack(fill='both', expand=1)     

        self.set_plugins_button = Button(self,text="Set Plugins",command=self._set_plugins,width=12)
        self.set_plugins_button.grid(row=2,column=1)
        self.set_plugins_button.bind(lambda e:self._set_plugins)

        self._init_plugins()

        tk.Label(self,text="SHM Simulation").grid(column=0, row=3)

        self.canvas = FigureCanvasTkAgg(self.vis_scene.figure, master=self)
        self.canvas.get_tk_widget().grid(column=0,row=4)

        self.ani = FuncAnimation(self.vis_scene.figure, self._update, init_func=self.vis_scene.init_space, frames=np.arange(0, 10000, 1), interval=self.sim_interval, blit=False)

    def _publish_joints(self):
        self.joint_publisher.publish([self.curr_joints[0], self.curr_joints[1]])
        print(f"Curr Joint Position: {self.curr_joints}")

    def _read_plugin_data(self):
        shared_plugin_path = os.path.expanduser(self.PLUGIN_FILE_PATH)
        yaml_file_path = os.path.join(shared_plugin_path, "plugins.yml")

        with open(yaml_file_path, "r") as yaml_file:
            self.plugin_dict = yaml.safe_load(yaml_file)

    def _init_plugins(self):
        row_inc = 0
        for group_name, plugins in self.plugin_dict.items():
            plugin = PluginOption(self, group_name, plugins)
            plugin.make_label(row=3+row_inc, col=1)
            plugin.make_drop_down(row=3+row_inc, col=2)
            self.plugin_menu[group_name] = plugin
            row_inc += 1

        self._set_plugins()


if __name__ == "__main__":

    if len(sys.argv) != 2:
        print("Usage: python sim_env.py <file_path>")
        sys.exit(1)  # Exit with an error code

    # Get the yml_path from the command-line argument
    yml_path = sys.argv[1]
    param_loader = RobotParamsLoader(yml_path)
    theta, a, d, alpha = param_loader.load_dh_params()
    DH_params = DH_parameters(theta, a, d, alpha)

    root = tk.Tk()
    root.geometry("1200x700")
    app = SimApp(DH_params, root)
    tk.mainloop()

