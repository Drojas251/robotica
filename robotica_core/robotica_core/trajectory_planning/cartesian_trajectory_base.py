# Numpy (Array computing Lib.) [pip3 install numpy]
import numpy as np
from abc import abstractmethod
from robotica_core.trajectory_planning.trajectory import Trajectory, JointTrajectoryPoint, CartesianTrajectoryPoint

class PathPoint():
    def __init__(self, point):
        self.point = point


class CartesianTrajectoryBase():
    def __init__(self, kinematics):
        self.kinematics = kinematics

    def cartesian_trajectory_generator(self, start_pt, target_pt, max_speed, num_steps=10):
        """ Cartesian Trajectory Generator
        
        Args:
            start_pt ([List]): Initial start position 
            target_pt([List]): Initial goal position 
            max_speed(Float): Max end effector speed 
            num_steps(Int): Number of interpolated points between start-goal points

        Returns:
            x[List]: ee x points 
            y[List]: ee y points 
            speeds[List]: speeds at each point 
        """

        pass
    
    def cartesian_trajectory(self, start_pt, target_pt, max_speed, num_steps=10):

        x, y, speeds = self.cartesian_trajectory_generator(start_pt, target_pt, max_speed, num_steps=num_steps)

        joint_trajectory = []
        cartesian_trajectory = []

        for i in range(num_steps):
            joint_value = self.kinematics.inverse_kinematics((x[i], y[i]), 1)

            # Define joint trajectory
            joint_traj_point = JointTrajectoryPoint(joint_value)
            joint_trajectory.append(joint_traj_point)

            # Define cartesian trajectory
            cart_traj_point = CartesianTrajectoryPoint((x[i], y[i]), speeds[i])
            cartesian_trajectory.append(cart_traj_point)

        return Trajectory(joint_traj=joint_trajectory, cartesian_traj=cartesian_trajectory)

        
