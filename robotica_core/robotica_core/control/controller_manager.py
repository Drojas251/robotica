import time
from enum import Enum

from robotica_datatypes.trajectory_datatypes.trajectory import Trajectory
from robotica_core.utils.yml_parser import NetworkingParams
from robotica_core.utils.robotica_networking import RoboticaPublisher
from robotica_core.utils.robotica_networking import RoboticaService

class ControllerStatus(Enum):
    SUCCESS = "move_succeeded"
    FAILED_MOVE = "move_failed"
    ERROR = "controller_in_error"


class ControllerManager():
    def __init__(self, init_joints):
        self.current_trajectory = Trajectory()
        
        networking_params = NetworkingParams() 
        # Move Service 
        move_serv_port = networking_params.get_serv_req_info("move_service")
        self.move_service = RoboticaService(port=move_serv_port)

        # Joint Publisher
        topic, port = networking_params.get_pub_sub_info("joint_publisher")
        self.joint_publisher = RoboticaPublisher(port=port, topic=topic)

        self.curr_pt = None
        self.curr_joints = init_joints

    def consume_wpt(self):
        joint_wpt = self.current_trajectory.joint_traj.pop(0)
        cartesian_wpt = self.current_trajectory.cartesian_traj.pop(0)

        if len(self.current_trajectory.joint_traj) == 0:
            self.move_service.send_response(ControllerStatus.SUCCESS)

        # check for speed
        if cartesian_wpt.velocity:
            self.ctl_speed(cartesian_wpt.point, cartesian_wpt.velocity)

        elif len(joint_wpt.velocities) > 0:
            time.sleep(0.02)
        else:
            default_velo = 0.5
            self.ctl_speed(cartesian_wpt.point, default_velo)
            
        # Publish Joint Positions
        self.curr_joints = joint_wpt.positions
        self.publish_joints()

        return joint_wpt.positions
    
    def ctl_speed(self, next_wpt, speed):
        if self.curr_pt:
            dist = ((next_wpt[0] - self.curr_pt[0])**2 + (next_wpt[1] - self.curr_pt[1])**2)**0.5
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
            # Publish Joint Positions
            self.publish_joints()
            return None

    def publish_joints(self):
        self.joint_publisher.publish([self.curr_joints[0], self.curr_joints[1]])

    def set_joint(self, joint_num, value):
        self.curr_joints[joint_num - 1] = value
        self.publish_joints()

    def set_joints(self, joints):
        self.curr_joints = joints
        self.publish_joints()

    def collision_detected(self):
        self.current_trajectory.cartesian_traj = []
        self.current_trajectory.joint_traj = []
        self.move_service.send_response(ControllerStatus.ERROR)