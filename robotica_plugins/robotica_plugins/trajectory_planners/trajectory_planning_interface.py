from robotica_core.trajectory_planning.cartesian_trajectory_base import CartesianTrajectoryBase
from abc import abstractmethod



class CartesianTrajectoryPluginInterface(CartesianTrajectoryBase):
    def __init__(self, kinematics):
        CartesianTrajectoryBase.__init__(self, kinematics)

    @abstractmethod
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