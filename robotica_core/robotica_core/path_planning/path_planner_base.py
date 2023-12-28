from robotica_core.utils.robotica_networking import RoboticaPublisher
from robotica_core.utils.yml_parser import NetworkingParams


class PathPlannerBase:
    def __init__(self, collision_checker):
        self.collision_checker = collision_checker

        networking_params = NetworkingParams() 
        topic, port = networking_params.get_pub_sub_info("path_publisher")
        self.path_publisher = RoboticaPublisher(port=port, topic=topic)

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
        self.path_publisher.publish(path)
        return path

    
