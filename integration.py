from robotica_plugins.robot_api import RobotAPI
from robotica_datatypes.path_datatypes.waypoint import WayPoint


yml_path = "/home/drojas/robot_arm/robotica/2_link_robot.yml"
env_path = "/home/drojas/robot_arm/robotica/env.yml"
robot = RobotAPI(yml_path, env_yml_file=env_path)

robot.plan_and_execute((0.05, 0.54))
robot.plan_and_execute((0.1, 0.24))
robot.plan_and_execute((0.25, 0.5))




