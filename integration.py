from robotica_plugins.robot_api import RobotAPI
from robotica_datatypes.path_datatypes.waypoint import WayPoint


yml_path = "/home/drojas/robot_arm/robotica/2_link_robot.yml"
robot = RobotAPI(yml_path)

robot.plan_and_execute((0.54, 0.14))
robot.plan_and_execute((0.1, 0.24))
robot.plan_and_execute((0.4, -0.14))




