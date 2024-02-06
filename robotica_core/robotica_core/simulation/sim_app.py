import numpy as np
import sys
import os
import yaml
import copy

# Sim
from robotica_core.simulation.sim_core import SimCore
from robotica_core.utils.yml_parser import RobotParamsLoader
from robotica_core.utils.yml_parser import NetworkingParams
from robotica_core.utils.robotica_networking import RoboticaPublisher
from robotica_core.utils.robotica_networking import RoboticaSubscriber
from robotica_core.control.controller_manager import ControllerManager
from robotica_datatypes.kinematic_datatypes.DH_params import DH_parameters    
from robotica_core.kinematics.robot_model import RobotModelBase
from matplotlib.animation import FuncAnimation

# App
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
import tkinter as tk
from tkinter import ttk
from tkinter import Frame,Label,Entry,Button

class SimEnv:
    def __init__(self, robot_model, env_yml_file):
        self.sim_core = SimCore(robot_model, env_yml_file)
        self.rendering = self.sim_core.vis_scene.figure
        self.controller_manager = ControllerManager(robot_model.DH_params.theta)
        self.sim = None

    def start_sim(self):
        self.sim = FuncAnimation(
            self.sim_core.vis_scene.figure, 
            self._update, 
            init_func=self.sim_core.vis_scene.init_space, 
            frames=np.arange(0, 10000, 1), 
            interval=0.01, 
            blit=False,
        )

    def _update(self, frame):
        if not self.controller_manager.is_executing_path():
            self.sim_core.vis_scene.get_planning_vis_cmd(10)
            
            traj = self.controller_manager.wait_for_joint_trajectory(10)
            if traj:
                self.sim_core.executing(True)
                self.sim_core.vis_scene.visualize_cartesian_trajectory(traj)
            else:
                self.sim_core.executing(False)
                return

        # Get next wpts to execute 
        theta1, theta2 = self.controller_manager.consume_wpt()

        # Simulate
        self.sim_core.update_joints([theta1, theta2])
        collision = self.sim_core.collisions_detected()
        
        if collision:
            self.controller_manager.collision_detected()

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

class SimGUI:
    PLUGIN_FILE_PATH = "~/.robotica/plugins"
    SELECTED_PLUGIN_FILE = "run_plugins.yml"

    def __init__(self, robot_model, env_yml_file, sim_control_frame, vis_control_frame):
        self.sim_control_frame = sim_control_frame
        self.vis_control_frame = vis_control_frame
        self.sim_env = SimEnv(robot_model, env_yml_file)

        # Plugins
        self.plugin_dict = {}
        self.plugin_menu = {}
        self._read_plugin_data()

        # Joint Listener
        networking_params = NetworkingParams() 
        topic, port = networking_params.get_pub_sub_info("joint_publisher")
        self.joint_listener = RoboticaSubscriber(port=port, topic=topic)
        self.joint_listener.subscribe(callback=self._joint_listener_callback)

        self._joint_sliders = []
                
    def setup_ui_module(self):
        # Define sim frame        
        self.sim_frame = tk.Frame(self.sim_control_frame, borderwidth=2, relief="ridge")
        self.sim_frame.grid(row=0, column=0, padx=10, pady=10, sticky="nsew")
        tk.Label(self.sim_frame,text="Robotica Simulation").grid(column=0, row=0)

        self.canvas = FigureCanvasTkAgg(self.sim_env.rendering, master=self.sim_frame)
        self.canvas.get_tk_widget().grid(column=0,row=1)

        # Define Plugin Frame
        self.plugin_frame = tk.Frame(self.sim_control_frame, borderwidth=2, relief="ridge")
        self.plugin_frame.grid(row=1, column=0, padx=10, pady=10, sticky="nsew")

        self.set_plugins_button = Button(self.plugin_frame,text="Set Plugins",command=self._set_plugins,width=12)
        self.set_plugins_button.grid(row=0,column=3)
        self.set_plugins_button.bind(lambda e:self._set_plugins)

        self.reload_sim_button = Button(self.plugin_frame,text="Reload Env",command=self._reload_sim,width=12)
        self.reload_sim_button.grid(row=1,column=3)
        self.reload_sim_button.bind(lambda e:self._reload_sim)

        self.reset_robot_button = Button(self.plugin_frame,text="Reset Robot",command=self._reset_robot,width=12)
        self.reset_robot_button.grid(row=2,column=3)
        self.reset_robot_button.bind(lambda e:self._reset_robot)

        # Vis control frame
        self.vis_frame = tk.Frame(self.vis_control_frame, borderwidth=2, relief="ridge")
        self.vis_frame.grid(row=0, column=0, padx=10, pady=10, sticky="nsew")
        self.vis_frame.grid_columnconfigure(0, weight=1)
        self.vis_frame.grid_columnconfigure(1, weight=1)

        # Create a variable to hold the checkbox state
        self._show_ws_check = tk.BooleanVar()
        self.workspace_button = tk.Checkbutton(self.vis_frame, text="Show Workspace", variable=self._show_ws_check, command=self._show_ws)
        self.workspace_button.grid(row=0,column=1)

        self._show_tftree_check = tk.BooleanVar()
        self.tftree_button = tk.Checkbutton(self.vis_frame, text="Show TF Tree", variable=self._show_tftree_check, command=self._show_tftree)
        self.tftree_button.grid(row=0,column=0)

        self._show_planning_check = tk.BooleanVar()
        self.planning_button = tk.Checkbutton(self.vis_frame, text="Show Planning", variable=self._show_planning_check, command=self._show_planning)
        self.planning_button.grid(row=1,column=0)

        self._show_arm_check = tk.BooleanVar(value=True)
        self.arm_button = tk.Checkbutton(self.vis_frame, text="Show Arm", variable=self._show_arm_check, command=self._show_arm)
        self.arm_button.grid(row=1,column=1)

        # Control frame
        self.control_frame = tk.Frame(self.vis_control_frame, borderwidth=2, relief="ridge")
        self.control_frame.grid(row=1, column=0, padx=10, pady=10, sticky="nsew")
        self.control_frame.grid_columnconfigure(0, weight=1)
        self.control_frame.grid_columnconfigure(1, weight=1)

        # Create the first slider
        self.slider1 = tk.Scale(self.control_frame, from_=-6.0, to=6.0, orient=tk.HORIZONTAL, length=300, digits = 3, resolution = 0.01, command=lambda value: self._on_slider_change(1, value))
        self.slider1.grid(row=0, column=1, padx=10, pady=10)
        self._joint_sliders.append(self.slider1)
        self.joint_label1 = Label(self.control_frame, text="Joint 1", width=15)
        self.joint_label1.grid(row=0, column=0)

        # Create the second slider
        self.slider2 = tk.Scale(self.control_frame, from_=-6.0, to=6.0, orient=tk.HORIZONTAL, length=300, digits = 3, resolution = 0.01,command=lambda value: self._on_slider_change(2, value))
        self.slider2.grid(row=1, column=1, padx=10, pady=10)
        self._joint_sliders.append(self.slider2)
        self.joint_label2 = Label(self.control_frame, text="Joint 2", width=15)
        self.joint_label2.grid(row=1, column=0)

        for i, joint_value in enumerate(self.sim_env.controller_manager.curr_joints):
            self._joint_sliders[i].set(joint_value)

        self._init_plugins()
        self.sim_env.start_sim()

    def _show_ws(self):
        if self._show_ws_check.get():
            self.sim_env.sim_core.vis_scene.visualize_workspace()
        else:
            self.sim_env.sim_core.vis_scene.clear_workspace()

    def _show_tftree(self):
        if self._show_tftree_check.get():
            self.sim_env.sim_core.vis_scene.visualize_tftree()
        else:
            self.sim_env.sim_core.vis_scene.clear_tftree()

    def _show_planning(self):
        pass

    def _show_arm(self):
        if self._show_arm_check.get():
            self.sim_env.sim_core.vis_scene.visualize_arm()
        else:
            self.sim_env.sim_core.vis_scene.clear_arm()
        
    def _reload_sim(self):
        self._read_plugin_data()
        self._init_plugins()

        self.sim_env.sim_core.reset_env()

    def _reset_robot(self):
        init_joints = copy.copy(self.sim_env.sim_core.init_joints)
        self.sim_env.controller_manager.set_joints(init_joints)
        self.sim_env.sim_core.update_joints(self.sim_env.controller_manager.curr_joints)
        for i, j in enumerate(init_joints):
            self._joint_sliders[i].set(j)

    def _read_plugin_data(self):
        shared_plugin_path = os.path.expanduser(self.PLUGIN_FILE_PATH)
        yaml_file_path = os.path.join(shared_plugin_path, "plugins.yml")

        with open(yaml_file_path, "r") as yaml_file:
            self.plugin_dict = yaml.safe_load(yaml_file)

    def _on_slider_change(self, slider_id, value):
        if not self.sim_env.sim_core.is_executing():      
            self.sim_env.controller_manager.set_joint(slider_id, float(value))
            self.sim_env.sim_core.update_joints(self.sim_env.controller_manager.curr_joints)

    def _joint_listener_callback(self, data):
        if self.sim_env.sim_core.is_executing():
            for i, j in enumerate(data):
                self._joint_sliders[i].set(j)

    def _init_plugins(self):
        row_inc = 0
        for group_name, plugins in self.plugin_dict.items():
            plugin = PluginOption(self.plugin_frame, group_name, plugins)
            plugin.make_label(row=0+row_inc, col=0)
            plugin.make_drop_down(row=0+row_inc, col=1)
            self.plugin_menu[group_name] = plugin
            row_inc += 1

        self._set_plugins()

    def _set_plugins(self):
        run_plugins = {}
        for group_name, plugin_options in self.plugin_menu.items():
            run_plugins[group_name] = plugin_options.plugin_selected.get()
            print(f"{group_name}: {plugin_options.plugin_selected.get()}")

        shared_plugin_path = os.path.expanduser(self.PLUGIN_FILE_PATH)
        yaml_file_path = os.path.join(shared_plugin_path, self.SELECTED_PLUGIN_FILE)

        with open(yaml_file_path, "w") as yaml_file:
            yaml.dump(run_plugins, yaml_file, default_flow_style=False)

class ContinousLogger:
    """
    This class is ment to append text
    """
    def __init__(self, text_widget):
        self.text_widget = text_widget

    def write(self, message):
        self.text_widget.insert(tk.END, message)
        self.text_widget.see(tk.END)  # Scroll to the end of the text

class SingleLogger:
    """
    This class is ment to overwrite text
    """
    def __init__(self, text_widget):
        self.text_widget = text_widget
        self.prev_msg = None

    def write(self, message):
        if message != self.prev_msg:
            self.text_widget.delete("1.0", tk.END)  # Delete existing content
            self.text_widget.insert(tk.END, message)
            self.prev_msg = message

class GuiLogging:
    def __init__(self, master):
        self.master = master
        self.joint_logger = None

        # Joint Listener
        networking_params = NetworkingParams() 
        topic, port = networking_params.get_pub_sub_info("joint_publisher")
        self.joint_listener = RoboticaSubscriber(port=port, topic=topic)
        self.joint_listener.subscribe(callback=self._joint_listener_callback)

        # Bottom of the right frame is the logging frame
        self.logging_frame = tk.Frame(self.master, borderwidth=2, relief="ridge",width=100, height=20)
        self.logging_frame.grid(row=2, column=0, columnspan=2, sticky="nsew")
        self.logging_frame.grid_rowconfigure(0, weight=1)
        self.logging_frame.grid_rowconfigure(1, weight=1)

    def setup_ui_module(self):
        #self._setup_console_logger()
        self._setup_joint_logger()
        self._setup_console_logger()
        
    def _setup_console_logger(self):
        self.console_frame = tk.Frame(self.logging_frame, borderwidth=2, relief="ridge",width=100, height=5)
        self.console_frame.grid(row=1, column=0, padx=10, pady=10, sticky="nsew")
        tk.Label(self.console_frame,text="Status").grid(column=0, row=0)

        self.console_text = tk.Text(self.console_frame, wrap=tk.WORD, height=5, width=60)
        self.console_text.grid(row=1, column=0)

        # Redirect console outputs to the Text widget
        sys.stdout = ContinousLogger(self.console_text)

    def _setup_joint_logger(self):
        self.joint_frame = tk.Frame(self.logging_frame, borderwidth=2, relief="ridge")
        self.joint_frame.grid(row=0, column=0, padx=10, pady=10, sticky="nsew")
        tk.Label(self.joint_frame,text="Joint Angles").grid(column=0, row=0)

        self.joint_text = tk.Text(self.joint_frame, wrap=tk.WORD, height=5, width=60)
        self.joint_text.grid(row=1, column=0)

        self.joint_logger = SingleLogger(self.joint_text)

    def _joint_listener_callback(self, data):
        msg = ""
        for i, j in enumerate(data):
            msg = msg + f"Joint {i+1}: {j}\n"
        self.joint_logger.write(msg)

class RoboticaApp:           
    def __init__(self, robot_model, env_yml_file, master):
        self.master = master

        # Logging Module
        self.right_frame = tk.Frame(self.master, borderwidth=2, relief="ridge")
        self.right_frame.pack(side="right", fill="both", expand=True)
        self.right_frame.grid_rowconfigure(0, weight=1)
        self.right_frame.grid_rowconfigure(1, weight=1)
        self.right_frame.grid_rowconfigure(2, weight=1)
        self.gui_logging = GuiLogging(self.right_frame)

        # Sim Display and Interface Module
        self.sim_gui_frame = tk.Frame(self.master, borderwidth=2, relief="ridge")
        self.sim_gui_frame.pack(side="left", fill="both", expand=True)
        self.sim_gui_frame.grid_rowconfigure(0, weight=1)
        self.sim_gui_frame.grid_rowconfigure(1, weight=1)
        self.sim_gui = SimGUI(robot_model, env_yml_file, self.sim_gui_frame, self.right_frame)

    def init_gui(self):
        self.gui_logging.setup_ui_module()
        self.sim_gui.setup_ui_module()


if __name__ == "__main__":

    if len(sys.argv) != 3:
        print("Usage: python sim_env.py <robot_model_file_path> <env_file_path>")
        sys.exit(1)  # Exit with an error code

    # Get the yml_path from the command-line argument
    robot_yml_path = sys.argv[1]
    robot_model = RobotModelBase(robot_yml_path)

    # File that describes the environment
    env_yml_path = sys.argv[2]

    root = tk.Tk()
    root.geometry("1200x700")
    app = RoboticaApp(robot_model, env_yml_path, root)
    app.init_gui()
    tk.mainloop()

