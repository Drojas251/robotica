from robotica_plugins.robot_launch import Robot
from robotica_datatypes.path_datatypes.waypoint import WayPoint


yml_path = "/home/drojas/robot_arm/robotica/2_link_robot.yml"
robot = Robot(yml_path)

#joint_traj = [(0.4, 0.13), (0.4, 0.23), (0.4, 0.33), (0.4, 0.43), (0.4, 0.53), (0.4, 0.63), (0.4, 0.73), (0.3, 0.73), (0.2, 0.73), (0, 0.73), (-0.2, 0.73)]
# joint_traj = [(0.1, 0.0), (0.2, 0.3)]
# robot.joint_move(joint_traj)

# robot.cartesian_move((0.49, 0.24), (0.35, 0.0), 0.1)
# robot.cartesian_move((0.35, 0.0), (0.24, 0.24), 0.3)

#robot.cartesian_move( (0.24, 0.24), (0.49, 0.24), 0.9)

path1 = [WayPoint((0.24, -0.24), 0.3)]
robot.execute_path(path1)

path = [WayPoint((0.35, 0.0), 0.4), WayPoint((0.24, 0.24), 0.2), WayPoint((0.49, 0.24), 0.6)]
robot.execute_path(path)


path = [WayPoint((0.35, 0.0), 0.4), WayPoint((0.24, -0.24), 0.2), WayPoint((0.09, 0.24), 0.6)]
robot.execute_path(path)
# robot.cartesian_move((0.49, 0.24), (0.24, 0.24), 0.1)
# robot.cartesian_move((0.24, 0.24), (0.35, 0.0), 0.5)
# robot.cartesian_move((0.35, 0.0), (0.49, 0.24), 0.9)




