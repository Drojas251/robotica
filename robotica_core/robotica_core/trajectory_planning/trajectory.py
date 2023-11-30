

class Trajectory():
    def __init__(self, joint_traj=[], cartesian_traj=[]):
        self.joint_traj = joint_traj
        self.cartesian_traj = cartesian_traj

    def set_joint_traj(self, traj):
        self.joint_traj = traj
        
    def set_cartesian_traj(self, traj):
        self.cartesian_traj = traj
