from enum import Enum
from robotica_core.utils.robotica_networking import RoboticaPublisher, RoboticaClient
from robotica_core.utils.yml_parser import NetworkingParams

class PathVisManagerCmds(Enum):
    CLEAR = "clear"

class PathPlannerBase:
    def __init__(self, collision_checker, kinematics):
        self.collision_checker = collision_checker
        self.kinematics = kinematics

        networking_params = NetworkingParams() 
        topic, port = networking_params.get_pub_sub_info("path_publisher")
        self.path_publisher = RoboticaPublisher(port=port, topic=topic)

        topic, port = networking_params.get_pub_sub_info("sample_planning_publisher")
        self.sample_planning_publisher = RoboticaPublisher(port=port, topic=topic)

        path_vis_serv_port = networking_params.get_serv_req_info("path_vis_service")
        self.path_vis_client = RoboticaClient(port=path_vis_serv_port)

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

    def validate_point(self, point):
        """ Checks if point is valid

        Args:
            point (tuple): (x, y)

        Return:
            valid (bool)
        """
        try:
            joints = self.kinematics.inverse_kinematics(point, 1)
        except:
            return False

        collision = self.collision_checker.check_collision(joints)

        return not collision

    def plan(self, start, goal):
        self.path_vis_client.send_req(PathVisManagerCmds.CLEAR)
        path = self.planner(start, goal)

        if path is None:
            raise Exception("Path Planner returned None")

        self.path_publisher.publish(path)
        return path

    def show_sample_segment(self, start, goal):
        """ Publishes the path segment

        Args:
            start (tuple): (x, y)
            goal (tuple): (x, y)
        """
        self.sample_planning_publisher.publish([start, goal])

    
