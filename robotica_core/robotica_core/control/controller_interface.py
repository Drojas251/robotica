import zmq
import pickle
from robotica_core.utils.yml_parser import get_serv_req_info


class ControllerInterface:
    def __init__(self):
        self.move_req_port = get_serv_req_info("move_request")

    def execute_move(self, trajectory):
        # This is a blocking function 

        context = zmq.Context()
        socket = context.socket(zmq.REQ)
        socket.connect(f"tcp://127.0.0.1:{self.move_req_port}")

        message = pickle.dumps(trajectory)

        socket.send(message)
        print("Sent Move Request")

        reply_message = socket.recv_string()
        print(f"Controller: {reply_message}")
