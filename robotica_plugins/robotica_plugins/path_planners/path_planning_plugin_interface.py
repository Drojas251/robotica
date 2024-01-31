from abc import abstractmethod
from robotica_core.path_planning.path_planner_base import PathPlannerBase

class PathPlannerPluginInterface(PathPlannerBase):
    def __init__(self, *args):
        PathPlannerBase.__init__(self, *args)

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

        if path cannot be found
        
        Return:
            path: None
        """
        pass