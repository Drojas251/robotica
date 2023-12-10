import zmq
import pickle

class ControllerInterface:
    def __init__(self):
        self.joint_traj_req_port = "5555"

    def execute_move(self, trajectory):
        # This is a blocking function 

        context = zmq.Context()
        socket = context.socket(zmq.REQ)
        socket.connect(f"tcp://127.0.0.1:{self.joint_traj_req_port}")

        message = pickle.dumps(trajectory)

        socket.send(message)
        print("Sent Move Request")

        reply_message = socket.recv_string()
        print(f"Controller: {reply_message}")
