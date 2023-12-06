from robotica_core.kinematics.robot_model import RobotModel
from robotica_core.utils.yml_parser import load_kinematics_class, load_trjectory_planner_class
from robotica_plugins.kinematics.definition import Kinematics_Plugins
from robotica_plugins.trajectory_planners.definition import Trajectory_Planers_Plugins
from robotica_core.simulation.joint_publisher import publish_joint_data
from robotica_core.trajectory_planning.trajectory import Trajectory, CartesianTrajectoryPoint, JointTrajectoryPoint

class Robot:
    def __init__(self, robot_yml_file):
        self.robot_model = RobotModel(robot_yml_file)

        kinematics_cls_name = load_kinematics_class(robot_yml_file)
        kinematics_cls = Kinematics_Plugins[kinematics_cls_name]
        self.kinematics = kinematics_cls(self.robot_model)

        traj_planner_cls_name = load_trjectory_planner_class(robot_yml_file)
        traj_planner_cls = Trajectory_Planers_Plugins[traj_planner_cls_name]
        self.cartesian_trajectory_planner = traj_planner_cls(self.kinematics)

    def joint_move(self, joint_trajectory):
        cartesian_traj = []
        joint_traj = []
        for joint in joint_trajectory:
            ee_point = self.kinematics.forward_kinematics(joint)
            cartesian_traj_point = CartesianTrajectoryPoint(ee_point)
            cartesian_traj.append(cartesian_traj_point)

            joint_traj_point = JointTrajectoryPoint(joint)
            joint_traj.append(joint_traj_point)

        trajectory = Trajectory(joint_traj, cartesian_traj)
        publish_joint_data(trajectory)

    def cartesian_move(self, start_point, target_point, speed):
        trajectory = self.cartesian_trajectory_planner.cartesian_trajectory(start_point, target_point, speed, num_steps=30)
        publish_joint_data(trajectory)

    def get_current_joint_positions(self):
        return self.robot_model.get_current_pos()
    
    def execute_path(self, path, speed):
        """ Move arm along a path 

        Args: 
            path[List]: A list of wpts
        """

        for wpt in path:
            curr_joints = self.get_current_joint_positions()
            curr_pos = self.kinematics.forward_kinematics(curr_joints)
            self.cartesian_move(curr_pos, wpt, speed)
