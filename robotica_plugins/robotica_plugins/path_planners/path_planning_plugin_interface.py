from abc import abstractmethod
from robotica_core.path_planning.path_planner_base import PathPlannerBase

class PathPlannerPluginInterface(PathPlannerBase):
    def __init__(self, collision_checker):
        PathPlannerBase.__init__(self, collision_checker)
        
        self.collision_checker = collision_checker

    @abstractmethod
    def planner(self, start, goal):
        """ Define Planner Here

        Planner can be any planner of your choosing. 
        Planner must take in a start position and end position, 
        and return a list of WayPoint's. 

        Args: 
            start (tuple): (x, y)
            goal (tuple): (x, y)

        Return:
            path (List): [Waypoint]
        """
        pass