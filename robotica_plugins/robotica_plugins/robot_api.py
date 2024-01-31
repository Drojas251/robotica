from robotica_core.api.robotica_interface import RoboticaCore
from robotica_plugins.kinematics.definition import Kinematics_Plugins
from robotica_plugins.trajectory_planners.definition import Trajectory_Planners_Plugins
from robotica_plugins.path_planners.definition import Path_Planners_Plugins
from robotica_datatypes.trajectory_datatypes.trajectory import Trajectory, JointTrajectoryPoint, CartesianTrajectoryPoint
from robotica_datatypes.path_datatypes.waypoint import WayPoint


class RobotAPI(RoboticaCore):
    def __init__(self, robot_yml_file, env_yml_file=None):
        RoboticaCore.__init__(
            self, 
            robot_yml_file, 
            env_yml_file,
            kinematics_plugins = Kinematics_Plugins, 
            cartesian_traj_plugins = Trajectory_Planners_Plugins,
            path_planner_plugins = Path_Planners_Plugins,
        )

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
        self.controller.execute_move(trajectory)

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
        self.controller.execute_move(trajectory)
    
    ######################
    # Robot Core Interface
    ######################
    @property
    def robot_model(self):
        return self._robot_model

    @property
    def kinematics(self):
        """ Returns Loaded Kinematics Class

        Available Class Methods:
            forward_kinematics
                args: 
                    joints (list): Joint angles
                return: 
                    point (list): (x,y) point
            
            inverse_kinematics
                args: 
                    point (list): (x,y) point
                    cfg (int): robot config
                return:
                    joints (list): Joint angles
        """
        return self._kinematics

    @property
    def cartesian_trajectory_planner(self):
        """ Returns Loaded Trajectory Planner Class

        Available Class Methods:
            cartesian_trajectory
                args: 
                    wpts (list): List WayPoint
                return: 
                    trajectory (Trajectory)
        """
        return self._cartesian_trajectory_planner

    @property
    def controller(self):
        """ Returns Controller Interface

        Available Class Methods:
            execute_move
                args: 
                    trajectory (Trajectory)
        """
        return self._controller

    @property
    def path_planner(self):
        """ Returns Path Planner Interface

        Available Class Methods:
            plan
                args: 
                    start (tuple)
                    goal (tuple)
        """
        return self._path_planner


