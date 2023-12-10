from robotica_core.kinematics.robot_model import RobotModel
from robotica_core.utils.yml_parser import load_kinematics_class, load_trjectory_planner_class
from robotica_plugins.kinematics.definition import Kinematics_Plugins
from robotica_plugins.trajectory_planners.definition import Trajectory_Planners_Plugins
from robotica_core.simulation.joint_publisher import publish_joint_data
from robotica_datatypes.trajectory_datatypes.trajectory import Trajectory, JointTrajectoryPoint, CartesianTrajectoryPoint
from robotica_datatypes.path_datatypes.waypoint import WayPoint

class Robot:
    def __init__(self, robot_yml_file):
        self.robot_model = RobotModel(robot_yml_file)

        kinematics_cls_name = load_kinematics_class(robot_yml_file)
        kinematics_cls = Kinematics_Plugins[kinematics_cls_name]
        self.kinematics = kinematics_cls(self.robot_model)

        traj_planner_cls_name = load_trjectory_planner_class(robot_yml_file)
        traj_planner_cls = Trajectory_Planners_Plugins[traj_planner_cls_name]
        self.cartesian_trajectory_planner = traj_planner_cls(self.kinematics)

    def joint_move(self, joint_trajectory):
        cartesian_traj = []
        joint_traj = []
        for joint in joint_trajectory:
            ee_point = self.kinematics.forward_kinematics(joint)
            cartesian_traj_point = CartesianTrajectoryPoint((ee_point[0], ee_point[1]), 0.2)
            cartesian_traj.append(cartesian_traj_point)

            joint_traj_point = JointTrajectoryPoint(joint)
            joint_traj.append(joint_traj_point)

        trajectory = Trajectory(joint_traj, cartesian_traj)
        publish_joint_data(trajectory)

    def get_current_joint_positions(self):
        return self.robot_model.get_current_pos()
    
    def execute_path(self, path):
        """ Move arm along a path 

        Args: 
            path[List]: A list of wpts
        """

        curr_joints = self.get_current_joint_positions()
        curr_pos = self.kinematics.forward_kinematics(curr_joints)
        cur_wpt = WayPoint(curr_pos, 0)
        path.insert(0, cur_wpt)

        trajectory = self.cartesian_trajectory_planner.cartesian_trajectory(path)
        publish_joint_data(trajectory)


