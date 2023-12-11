import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import sys

from robotica_core.utils.yml_parser import RobotParamsLoader
from robotica_core.utils.robotica_networking import RoboticaPublisher
from robotica_core.control.controller_manager import ControllerManager
from robotica_datatypes.kinematic_datatypes.DH_params import DH_parameters

import random

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


class SimEnv():
    def __init__(self, DH_params):

        self.vis_scene = Visualization(DH_params)
        self.controller_manager = ControllerManager()
        self.joint_publisher = RoboticaPublisher(port="5153", topic="joint_publisher")
        self.curr_joints = DH_params.theta
        self.sim_interval = 0.01

    def run(self):
        self.animation = FuncAnimation(self.vis_scene.figure, self.update, init_func=self.vis_scene.init_space, frames=np.arange(0, 10000, 1), interval=self.sim_interval, blit=False)
        self.vis_scene.run()
        
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
    
    sim = SimEnv(DH_params)
    sim.run()

