from robotica_core.utils.yml_parser import RobotParamsLoader
from robotica_core.utils.robotica_networking import RoboticaSubscriber
from robotica_datatypes.kinematic_datatypes.DH_params import DH_parameters
from robotica_datatypes.kinematic_datatypes.joint_limits import JointLimits
    

class RobotModel():
    def __init__(self, yml_path):
        param_loader = RobotParamsLoader(yml_path)
        self.robot_name = param_loader.load_robot_name()
        
        limits = param_loader.load_joint_limits()
        self.joint_limits = []
        for limit in limits.values():
            joint_limit = JointLimits(limit)
            self.joint_limits.append(joint_limit)

        theta, a, d, alpha = param_loader.load_dh_params()
        self.DH_params = DH_parameters(theta, a, d, alpha)

        self.thetas = self.DH_params.theta

        self.joint_listener = RoboticaSubscriber(port="5153", topic="joint_publisher")
        self.joint_listener.subscribe(callback=self.joint_listener_callback)

    def joint_listener_callback(self, data):
        self.thetas = data
    
    def get_current_pos(self):
        return self.thetas

