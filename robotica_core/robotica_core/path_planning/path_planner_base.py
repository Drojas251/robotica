class PathPlannerBase:
    def __init__(self, collision_checker):
        self.collision_checker = collision_checker

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

    def plan(self, start, goal):
        path = self.planner(start, goal)
        return path

    
