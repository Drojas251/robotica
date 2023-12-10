# Numpy (Array computing Lib.) [pip3 install numpy]
import numpy as np
from abc import abstractmethod
from robotica_datatypes.trajectory_datatypes.trajectory import Trajectory, JointTrajectoryPoint, CartesianTrajectoryPoint

class CartesianTrajectoryBase():
    def __init__(self, kinematics):
        self.kinematics = kinematics

    def cartesian_trajectory_generator(self, wpts):
        """ Cartesian Trajectory Generator
        
        Args:
            wpts ([WayPoint]): List of WayPoint objects

        Returns:
            x[List]: ee x points 
            y[List]: ee y points 
            speeds[List]: speeds at each point 
        """

        pass
    
    def cartesian_trajectory(self, wpts):

        x, y, speeds = self.cartesian_trajectory_generator(wpts)

        joint_trajectory = []
        cartesian_trajectory = []

        for i in range(len(x)):
            joint_value = self.kinematics.inverse_kinematics((x[i], y[i]), 1)

            # Define joint trajectory
            joint_traj_point = JointTrajectoryPoint(joint_value)
            joint_trajectory.append(joint_traj_point)

            # Define cartesian trajectory
            cart_traj_point = CartesianTrajectoryPoint((x[i], y[i]), speeds[i])
            cartesian_trajectory.append(cart_traj_point)

        return Trajectory(joint_traj=joint_trajectory, cartesian_traj=cartesian_trajectory)

        
