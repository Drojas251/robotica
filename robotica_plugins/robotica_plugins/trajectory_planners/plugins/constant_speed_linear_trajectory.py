import numpy as np
from robotica_plugins.trajectory_planners.trajectory_planning_interface import CartesianTrajectoryPluginInterface



class ConstSpeedLinearTraj(CartesianTrajectoryPluginInterface):
    def __init__(self, kinematics):
        CartesianTrajectoryPluginInterface.__init__(self, kinematics)

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

        time = np.linspace(0.0, 1.0, num_steps)

        # Linear Bezier Curve
        # p(t) = (1 - t)*p_{0} + t*p_{1}, t ∈ [0, 1]
        x = (1 - time) * start_pt[0] + time * target_pt[0]
        y = (1 - time) * start_pt[1] + time * target_pt[1]

        speeds = [max_speed] * len(time)

        return x, y, speeds