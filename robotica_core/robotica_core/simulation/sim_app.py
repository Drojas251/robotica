import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import sys

from robotica_core.utils.yml_parser import RobotParamsLoader
from robotica_core.utils.robotica_networking import RoboticaPublisher
from robotica_core.control.controller_manager import ControllerManager
from robotica_datatypes.kinematic_datatypes.DH_params import DH_parameters    

import random

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

    def run(self):
        self.plt.show()

    def init_space(self):
        theta1 = self.DH_params.theta[0]
        theta2 = self.DH_params.theta[1]
        self.visualize_arm(theta1, theta2)

    def visualize_arm(self, theta1, theta2):
        y_end = self.link_length_1 * np.sin(theta1) + self.link_length_2 * np.sin(theta1 + theta2)
        x_end = self.link_length_1 * np.cos(theta1) + self.link_length_2 * np.cos(theta1 + theta2)

        # [origin_x, x_pos_first_joint, x_pos_ee], [origin.y, y_pos_first_joint, y_pos_ee]
        self.arm_line.set_data([0, self.link_length_1 * np.cos(theta1), x_end], [0, self.link_length_1 * np.sin(theta1), y_end])

    def visualize_cartesian_path(self, cartesian_traj):
        self._format_path_data(cartesian_traj)
        self.path.set_data(self.x, self.y)

    def _format_path_data(self, cartesian_traj):
        self.x = [p.point[0] for p in cartesian_traj]
        self.y = [p.point[1] for p in cartesian_traj]



class SimApp(Frame):
    def __init__(self, DH_params, master = None):
        Frame.__init__(self, master)
        self.master = master

        self.vis_scene = Visualization(DH_params)
        self.controller_manager = ControllerManager()
        self.joint_publisher = RoboticaPublisher(port="5153", topic="joint_publisher")
        self.curr_joints = DH_params.theta
        self.sim_interval = 0.01

        self.init_window()

    def Clear(self):      
        print(f"{self.selected_option.get()}")

    def Plot(self):
        pass

    def update(self, frame):
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

    def init_window(self):
        self.master.title("Use Of FuncAnimation in tkinter based GUI")
        self.pack(fill='both', expand=1)     

        #Create the controls, note use of grid

        # self.labelSpeed = Label(self,text="Speed (km/Hr)",width=12)
        # self.labelSpeed.grid(row=0,column=1)
        # self.labelAmplitude = Label(self,text="Amplitude",width=12)
        # self.labelAmplitude.grid(row=0,column=2)

        # self.textSpeed = Entry(self,width=12)
        # self.textSpeed.grid(row=1,column=1)
        # self.textAmplitude = Entry(self,width=12)
        # self.textAmplitude.grid(row=1,column=2)

        # self.textAmplitude.insert(0, "1.0")
        # self.textSpeed.insert(0, "1.0")
        # self.v = 1.0
        # self.A = 1.0

        # self.buttonPlot = Button(self,text="Print",command=self.Plot,width=12)        
        # self.buttonPlot.grid(row=2,column=1)
        self.buttonClear = Button(self,text="Print",command=self.Clear,width=12)
        self.buttonClear.grid(row=2,column=1)
        self.buttonClear.bind(lambda e:self.Clear)

        # Dropdown menu
        self.labelDropdown = Label(self, text="Kinematics", width=12)
        self.labelDropdown.grid(row=3, column=1)

        self.options = ['Option 1', 'Option 2', 'Option 3']
        self.selected_option = tk.StringVar(value=self.options[0])
        self.dropdown = ttk.Combobox(self, textvariable=self.selected_option, values=self.options, state="readonly")
        self.dropdown.grid(row=3, column=2)

        tk.Label(self,text="SHM Simulation").grid(column=0, row=3)

        self.canvas = FigureCanvasTkAgg(self.vis_scene.figure, master=self)
        self.canvas.get_tk_widget().grid(column=0,row=4)

        self.ani = FuncAnimation(self.vis_scene.figure, self.update, init_func=self.vis_scene.init_space, frames=np.arange(0, 10000, 1), interval=self.sim_interval, blit=False)

    def _publish_joints(self):
        self.joint_publisher.publish([self.curr_joints[0], self.curr_joints[1]])
        print(f"Curr Joint Position: {self.curr_joints}")


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
    root.geometry("1000x700")
    app = SimApp(DH_params, root)
    tk.mainloop()

