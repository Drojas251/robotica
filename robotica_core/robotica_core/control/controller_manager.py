import time

from robotica_datatypes.trajectory_datatypes.trajectory import Trajectory
from robotica_core.utils.yml_parser import get_serv_req_info
from robotica_core.utils.robotica_networking import RoboticaService


class ControllerManager():
    def __init__(self):
        self.current_trajectory = Trajectory()

        # Move Service 
        move_serv_port = get_serv_req_info("move_service")
        self.move_service = RoboticaService(port=move_serv_port)

        self.curr_pt = None

    def consume_wpt(self):
        joint_wpt = self.current_trajectory.joint_traj.pop(0)
        cartesian_wpt = self.current_trajectory.cartesian_traj.pop(0)

        if len(self.current_trajectory.joint_traj) == 0:
            self.move_service.send_response("Move Completed")

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
        if self.move_service.received_request(delay):
            self.current_trajectory = self.move_service.unpack_request()
            return self.current_trajectory.cartesian_traj
        else:
            return None