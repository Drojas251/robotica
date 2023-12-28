from robotica_plugins.robot_api import RobotAPI
from robotica_datatypes.path_datatypes.waypoint import WayPoint


yml_path = "/home/drojas/robot_arm/robotica/2_link_robot.yml"
robot = RobotAPI(yml_path)

robot.plan_and_execute((0.54, 0.14))
robot.plan_and_execute((0.1, 0.24))
robot.plan_and_execute((0.2, -0.14))
# path1 = [WayPoint((0.4, 0.14), 0.3)]
# robot.execute_path(path1)

# path = [WayPoint((0.5, 0.0), 0.4), WayPoint((0.1, 0.24), 1.8), WayPoint((0.49, 0.24), 0.6)]
# robot.execute_path(path)


# path = [WayPoint((0.35, 0.0), 0.4), WayPoint((0.24, -0.24), 0.2), WayPoint((0.09, 0.24), 0.6)]
# robot.execute_path(path)

# path = [WayPoint((0.35, 0.0), 0.4), WayPoint((0.24, 0.24), 0.2), WayPoint((0.49, 0.24), 0.6)]
# robot.execute_path(path)



