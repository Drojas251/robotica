from robotica_core.utils.yml_parser import load_dh_params, load_robot_name, load_joint_limits

class DH_parameters(object):
    # << DH (Denavit-Hartenberg) parameters structure >> #
    def __init__(self, theta, a, d, alpha):
        # Angle about previous z, from old x to new x
        # Unit [radian]
        self.theta = theta
        # Length of the common normal. Assuming a revolute joint, this is the radius about previous z
        # Unit [metres]
        self.a = a
        # Offset along previous z to the common normal 
        # Unit [metres]
        self.d = d
        # Angle about common normal, from old z axis to new z axis
        # Unit [radian]
        self.alpha = alpha

class JointLimits():
    def __init__(self, limits):
        self._limits = limits

    @property
    def min_joint_pos(self):
        return self._limits["position"]["min"]
    
    @property
    def max_joint_pos(self):
        return self._limits["position"]["max"]
    
    @property
    def min_joint_velo(self):
        return self._limits["velocity"]["min"]
    
    @property
    def max_joint_velo(self):
        return self._limits["velocity"]["max"]        

class RobotModel():
    def __init__(self, yml_path):
        self.robot_name = load_robot_name(yml_path)
        
        limits = load_joint_limits(yml_path)
        self.joint_limits = []
        for limit in limits.values():
            joint_limit = JointLimits(limit)
            self.joint_limits.append(joint_limit)

        theta, a, d, alpha = load_dh_params(yml_path)
        self.DH_params = DH_parameters(theta, a, d, alpha)

