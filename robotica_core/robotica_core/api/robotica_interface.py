import os
import yaml
from robotica_core.kinematics.robot_model import RobotModel
from robotica_core.control.controller_interface import ControllerInterface
from robotica_core.planning_scene.collision_checker import CollisionChecker
from robotica_datatypes.trajectory_datatypes.trajectory import Trajectory, JointTrajectoryPoint, CartesianTrajectoryPoint
from robotica_datatypes.path_datatypes.waypoint import WayPoint


class RoboticaCore:
    PLUGIN_FILE_PATH = "~/.robotica/plugins"
    SELECTED_PLUGIN_FILE = "run_plugins.yml"

    def __init__(self, robot_yml_file, env_yml_file, kinematics_plugins=None, cartesian_traj_plugins=None, path_planner_plugins=None):
        self.robot_model = RobotModel(robot_yml_file)
        self._collision_checker = CollisionChecker(robot_yml_file, env_yml_file)
        self.kinematics = None
        self.cartesian_trajectory_planner = None
        self.path_planner = None

        shared_plugin_path = os.path.expanduser(self.PLUGIN_FILE_PATH)
        run_plugin_path = os.path.join(shared_plugin_path, "run_plugins.yml")
        with open(run_plugin_path, "r") as yaml_file:
            self.plugin_dict = yaml.safe_load(yaml_file)

        self._load_kinematics_plugins(kinematics_plugins)
        self._load_cartesian_traj_plugins(cartesian_traj_plugins)
        self._load_path_planner_plugins(path_planner_plugins)

        self._controller = ControllerInterface()

    def _load_kinematics_plugins(self, kin_plugins):
        if kin_plugins == None:
            raise Exception("No Kinematics Plugin Loaded")

        kinematics = None
        for plugin_cls_name in self.plugin_dict.values():
            if plugin_cls_name in kin_plugins:
                print(f"Using Kinematics Plugin: {plugin_cls_name}")
                kinematics_cls = kin_plugins[plugin_cls_name]
                kinematics = kinematics_cls(self.robot_model)
        
        if kinematics == None:
            raise Exception("Kinematics Plugin Not Found")

        self.kinematics = kinematics

    def _load_cartesian_traj_plugins(self, traj_plugins):
        if traj_plugins == None:
            raise Exception("No Cartesian Trajectory Plugin Loaded")

        traj_planner = None
        for plugin_cls_name in self.plugin_dict.values():
            if plugin_cls_name in traj_plugins:
                print(f"Using Trajectory Plugin: {plugin_cls_name}")
                traj_planner_cls = traj_plugins[plugin_cls_name]
                traj_planner = traj_planner_cls(self.kinematics)

        if traj_planner == None:
            raise Exception("Cartesian Trajectory Plugin Not Found")

        self.cartesian_trajectory_planner = traj_planner

    def _load_path_planner_plugins(self, path_planner_plugins):
        if path_planner_plugins == None:
            raise Exception("No Path Planner Plugin Loaded")

        path_planner = None
        for plugin_cls_name in self.plugin_dict.values():
            if plugin_cls_name in path_planner_plugins:
                print(f"Using Path Planner Plugin: {plugin_cls_name}")
                path_planner_cls = path_planner_plugins[plugin_cls_name]
                path_planner = path_planner_cls(self._collision_checker, self.kinematics)

        if path_planner == None:
            raise Exception("Path Planner Plugin Not Found")

        self.path_planner = path_planner

    #############################
    # Robot Application Interface
    #############################
    
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
        self._controller.execute_move(trajectory)

    def get_current_joint_positions(self):
        return self.robot_model.get_current_pos()
    
    def plan_and_execute(self, goal):
        curr_joints = self.get_current_joint_positions()
        curr_pos = self.kinematics.forward_kinematics(curr_joints)

        path = self.path_planner.plan(curr_pos, goal)
        self.execute_path(path)


    def execute_path(self, path):
        """ Move arm along a path 

        Args: 
            path[List]: A list of wpts
        """
        trajectory = self.cartesian_trajectory_planner.cartesian_trajectory(path)
        self._controller.execute_move(trajectory)

