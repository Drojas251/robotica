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