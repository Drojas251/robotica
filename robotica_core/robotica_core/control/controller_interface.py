import zmq
import pickle
from robotica_core.utils.yml_parser import NetworkingParams
from robotica_core.utils.robotica_networking import RoboticaClient
from robotica_core.control.controller_manager import ControllerStatus


class ControllerInterface:
    def __init__(self):
        networking_params = NetworkingParams()
        move_serv_port = networking_params.get_serv_req_info("move_service")
        self.move_client = RoboticaClient(port=move_serv_port)

        self.controller_valid = True

    def execute_move(self, trajectory):
        # This is a blocking function 
        if self.controller_valid:
            print("[Controller Interface]: Sent Move Request")
            resp = self.move_client.send_req(trajectory)
            print(f"[Controller]: {resp}")

            if resp == ControllerStatus.ERROR:
                self.controller_valid = False
        else:
            print("Controller is in state_error - Cannot process move")
