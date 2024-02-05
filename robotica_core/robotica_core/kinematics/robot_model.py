import numpy as np
import time
from robotica_core.utils.yml_parser import RobotParamsLoader
from robotica_core.utils.robotica_networking import RoboticaSubscriber
from robotica_datatypes.kinematic_datatypes.DH_params import DH_parameters
from robotica_datatypes.kinematic_datatypes.joint_limits import JointLimits
from robotica_core.utils.yml_parser import NetworkingParams


class RobotModelBase():
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

        # Work envelope
        self.x_p, self.y_p = self._generate_workspace()

    def _generate_workspace(self):
        # Generate linearly spaced vectors for the each of joints.
        theta_1 = np.linspace(self.joint_limits[0].min_joint_pos, self.joint_limits[0].max_joint_pos, 50)
        theta_2 = np.linspace(self.joint_limits[1].min_joint_pos, self.joint_limits[1].max_joint_pos, 50)

        # Return coordinate matrices from coordinate vectors.
        [theta_1_mg, theta_2_mg] = np.meshgrid(theta_1, theta_2)

        # Find the points x, y in the workspace using the equations FK.
        x_p = (self.DH_params.a[0]*np.cos(theta_1_mg) + self.DH_params.a[1]*np.cos(theta_1_mg + theta_2_mg))
        y_p = (self.DH_params.a[0]*np.sin(theta_1_mg) + self.DH_params.a[1]*np.sin(theta_1_mg + theta_2_mg))

        return x_p, y_p

class RobotModel(RobotModelBase):
    def __init__(self, yml_path):
        super().__init__(yml_path)

        networking_params = NetworkingParams() 

        self.thetas = None

        # Current Joint Angles
        topic, port = networking_params.get_pub_sub_info("joint_publisher")
        self.joint_listener = RoboticaSubscriber(port=port, topic=topic)
        self.joint_listener.subscribe(callback=self.joint_listener_callback)

        # Wait for thetas to be published
        for i in range(10):
            if self.thetas is not None:
                break
            time.sleep(0.1)

        else:
            self.thetas = self.DH_params.theta

    def joint_listener_callback(self, data):
        self.thetas = data
    
    def get_current_pos(self):
        return self.thetas
