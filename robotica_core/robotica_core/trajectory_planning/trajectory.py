

class Trajectory():
    def __init__(self, joint_traj=[], cartesian_traj=[]):
        self.joint_traj = joint_traj
        self.cartesian_traj = cartesian_traj

    def set_joint_traj(self, traj):
        self.joint_traj = traj
        
    def set_cartesian_traj(self, traj):
        self.cartesian_traj = traj

class JointTrajectoryPoint():
    def __init__(self, joint_positions=[], joint_velocities=[], joint_accelerations=[]):
        self.positions = joint_positions
        self.velocities = joint_velocities
        self.accelerations = joint_accelerations

class CartesianTrajectoryPoint():
    def __init__(self, point=[], velocity=None, acceleration=None):
        self.point = point
        self.velocity = velocity
        self.acceleration = acceleration
