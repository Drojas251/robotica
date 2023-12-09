from robotica_core.trajectory_planning.cartesian_trajectory_base import CartesianTrajectoryBase
from abc import abstractmethod



class CartesianTrajectoryPluginInterface(CartesianTrajectoryBase):
    def __init__(self, kinematics):
        CartesianTrajectoryBase.__init__(self, kinematics)

    @abstractmethod
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