import zmq
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import pickle
import time
import sys

from robotica_core.utils.yml_parser import load_dh_params
from robotica_core.kinematics.robot_model import DH_parameters
from robotica_core.trajectory_planning.trajectory import Trajectory


# def update(frame, data_socket, arm_line):
#     try:
#         joint_data = data_socket.recv_string().split()  # Extract joint_data from the received message
#         theta1 = float(joint_data[1])
#         theta2 = float(joint_data[2])

#         link_length = 1.0

#         x_end = link_length * np.sin(theta1) + link_length * np.sin(theta1 + theta2)
#         y_end = link_length * np.cos(theta1) + link_length * np.cos(theta1 + theta2)

#         arm_line.set_data([0, link_length * np.sin(theta1), x_end], [0, -link_length * np.cos(theta1), -y_end])

#     except (zmq.error.Again, ValueError) as e:
#         print(f"{e}")
#         pass


class SimEnv():
    def __init__(self, DH_params):

        self.plt = plt
        self.fig, ax = self.plt.subplots()
        ax.set_xlim(-1, 1)
        ax.set_ylim(-1, 1)
        self.arm_line, = ax.plot([], [], lw=5)
        self.path, = ax.plot([], [], lw=2)

        # context = zmq.Context()
        # self.data_socket = context.socket(zmq.SUB)
        # self.data_socket.connect("tcp://127.0.0.1:5555")  # Use the same address as the publisher
        # self.data_socket.setsockopt_string(zmq.SUBSCRIBE, "joint_data")

        context = zmq.Context()
        self.data_socket = context.socket(zmq.REP)
        self.data_socket.bind("tcp://127.0.0.1:5555")

        self.poller = zmq.Poller()
        self.poller.register(self.data_socket, zmq.POLLIN)

        self.link_length_1 = DH_params.a[0]
        self.link_length_2 = DH_params.a[1]
        self.trajectory = Trajectory()
        self.x = []
        self.y = []

    def init_animation(self):
        theta1 = 0.4
        theta2 = 0.2
        
        y_end = self.link_length_1 * np.sin(theta1) + self.link_length_2 * np.sin(theta1 + theta2)
        x_end = self.link_length_1 * np.cos(theta1) + self.link_length_2 * np.cos(theta1 + theta2)

        # [origin_x, x_pos_first_joint, x_pos_ee], [origin.y, y_pos_first_joint, y_pos_ee]
        self.arm_line.set_data([0, self.link_length_1 * np.cos(theta1), x_end], [0, self.link_length_1 * np.sin(theta1), y_end])

    def plot_path(self):
        #self.plt.plot(self.x, self.y, color='red', marker='o', markersize=5)
        self.path.set_data(self.x, self.y)

    def format_path_data(self):
        self.x = [p[0] for p in self.trajectory.cartesian_traj]
        self.y = [p[1] for p in self.trajectory.cartesian_traj]

    def run(self):
        self.animation = FuncAnimation(self.fig, self.update, init_func=self.init_animation, frames=np.arange(0, 10000, 1), interval=50, blit=False)
        self.plt.show()

    def consume_wpt(self):
        wpt = self.trajectory.joint_traj.pop(0)
        return wpt
        
    def update(self, frame):
        try:
            if self.poller.poll(100):
                serialized_data = self.data_socket.recv()
                self.trajectory = pickle.loads(serialized_data)
                self.data_socket.send_string("Data received successfully") 
                self.format_path_data()
                self.plot_path() 

            if len(self.trajectory.joint_traj) == 0:
                return

            theta1, theta2 = self.consume_wpt()
            print(theta1, theta2)

            y_end = self.link_length_1 * np.sin(theta1) + self.link_length_2 * np.sin(theta1 + theta2)
            x_end = self.link_length_1 * np.cos(theta1) + self.link_length_2 * np.cos(theta1 + theta2)

            self.arm_line.set_data([0, self.link_length_1 * np.cos(theta1), x_end], [0, self.link_length_1 * np.sin(theta1), y_end])

            time.sleep(0.5)

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

