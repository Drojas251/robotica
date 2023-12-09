import numpy as np
from robotica_plugins.trajectory_planners.trajectory_planning_interface import CartesianTrajectoryPluginInterface



class ConstSpeedLinearTraj(CartesianTrajectoryPluginInterface):
    def __init__(self, kinematics):
        CartesianTrajectoryPluginInterface.__init__(self, kinematics)
    
    def cartesian_trajectory_generator(self, wpts):
        """ Cartesian Trajectory Generator
        
        Args:
            wpts ([WayPoint]): List of WayPoint objects

        Returns:
            x[List]: ee x points 
            y[List]: ee y points 
            speeds[List]: speeds at each point 
        """
        num_steps = 15
        x_pts = []
        y_pts = []
        speeds = []

        for i in range(len(wpts) - 1):
            start_pt = wpts[i].point
            target_pt = wpts[i+1].point

            time = np.linspace(0.0, 1.0, num_steps)

            # Linear Bezier Curve
            # p(t) = (1 - t)*p_{0} + t*p_{1}, t ∈ [0, 1]
            x = (1 - time) * start_pt[0] + time * target_pt[0]
            y = (1 - time) * start_pt[1] + time * target_pt[1]

            speed = [wpts[i+1].speed] * len(time)

            x_pts.extend(x)
            y_pts.extend(y)
            speeds.extend(speed)

        return x_pts, y_pts, speeds
