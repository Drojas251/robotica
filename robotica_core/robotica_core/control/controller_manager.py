import zmq
import pickle
import time

from robotica_datatypes.trajectory_datatypes.trajectory import Trajectory


class ControllerManager():
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
            time.sleep(0.02)
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
        
    def wait_for_joint_trajectory(self, delay):
        if self.poller.poll(delay):
            serialized_data = self.data_socket.recv()
            self.current_trajectory = pickle.loads(serialized_data)
            return self.current_trajectory.cartesian_traj
        else:
            return None