import zmq
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import pickle
import time
import sys

from robotica_core.utils.yml_parser import load_dh_params
from robotica_core.utils.robotica_networking import RoboticaPublisher
from robotica_core.kinematics.robot_model import DH_parameters
from robotica_core.trajectory_planning.trajectory import Trajectory

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


class ExecutionManager():
    def __init__(self):
        self.current_trajectory = Trajectory()

        # Joint Service 
        context = zmq.Context()
        self.data_socket = context.socket(zmq.REP)
        self.data_socket.bind("tcp://127.0.0.1:5555")

        self.poller = zmq.Poller()
        self.poller.register(self.data_socket, zmq.POLLIN)

        self.curr_pt = None

    def consume_wpt(self):
        joint_wpt = self.current_trajectory.joint_traj.pop(0)
        cartesian_wpt = self.current_trajectory.cartesian_traj.pop(0)

        if len(self.current_trajectory.joint_traj) == 0:
            self.data_socket.send_string("Move Completed")

        # check for speed
        if cartesian_wpt.velocity:
            self.ctl_speed(cartesian_wpt.point, cartesian_wpt.velocity)

        elif len(joint_wpt.velocities) > 0:
            # Take the largest joint velo
            pass
        else:
            default_velo = 0.5
            self.ctl_speed(cartesian_wpt.point, default_velo)
            
        return joint_wpt.positions
    
    def ctl_speed(self, next_wpt, speed):
        if self.curr_pt:
            dist = ((next_wpt[0] - self.curr_pt[0])**2 + (next_wpt[1] - self.curr_pt[1])**2)**0.5
            print(dist/speed)
            time.sleep(dist/speed)
            
        self.curr_pt = next_wpt

    def is_executing_path(self):
        if len(self.current_trajectory.joint_traj) > 0:
            return True
        else:
            return False
        
    def wait_for_joint_trajectory(self):
        if self.poller.poll(1000):
            serialized_data = self.data_socket.recv()
            self.current_trajectory = pickle.loads(serialized_data)
            # self.data_socket.send_string("Data received successfully") 

            return self.current_trajectory.cartesian_traj
        else:
            return None

class SimEnv():
    def __init__(self, DH_params):

        self.vis_scene = Visualization(DH_params)
        self.execution_manager = ExecutionManager()
        self.joint_publisher = RoboticaPublisher(port="5153", topic="joint_publisher")
        self.curr_joints = None

    def run(self):
        self.animation = FuncAnimation(self.vis_scene.figure, self.update, init_func=self.vis_scene.init_space, frames=np.arange(0, 10000, 1), interval=0.01, blit=False)
        self.vis_scene.run()
        
    def update(self, frame):
        try:
            if not self.execution_manager.is_executing_path():
                traj = self.execution_manager.wait_for_joint_trajectory()
                if traj:
                    self.vis_scene.visualize_cartesian_path(traj)
                else:
                    return

            # Get next wpts to execute 
            theta1, theta2 = self.execution_manager.consume_wpt()

            # Move Arm
            self.vis_scene.visualize_arm(theta1, theta2)

            self.joint_publisher.publish([theta1, theta2])

        except (zmq.error.Again, ValueError) as e:
            print(f"{e}")
            pass


if __name__ == "__main__":

    if len(sys.argv) != 2:
        print("Usage: python sim_env.py <file_path>")
        sys.exit(1)  # Exit with an error code

    # Get the yml_path from the command-line argument
    yml_path = sys.argv[1]
    theta, a, d, alpha = load_dh_params(yml_path)
    DH_params = DH_parameters(theta, a, d, alpha)
    
    sim = SimEnv(DH_params)
    sim.run()

