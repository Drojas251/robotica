from robotica_plugins.robot_api import RobotAPI
from robotica_datatypes.path_datatypes.waypoint import WayPoint


yml_path = "/home/drojas/robot_arm/robotica/2_link_robot.yml"
robot = RobotAPI(yml_path)

path1 = [WayPoint((0.24, -0.24), 0.3)]
robot.execute_path(path1)

path = [WayPoint((0.35, 0.0), 0.4), WayPoint((0.04, 0.24), 0.2), WayPoint((0.49, 0.24), 0.6)]
robot.execute_path(path)


# path = [WayPoint((0.35, 0.0), 0.4), WayPoint((0.24, -0.24), 0.2), WayPoint((0.09, 0.24), 0.6)]
# robot.execute_path(path)

# path = [WayPoint((0.35, 0.0), 0.4), WayPoint((0.24, 0.24), 0.2), WayPoint((0.49, 0.24), 0.6)]
# robot.execute_path(path)





