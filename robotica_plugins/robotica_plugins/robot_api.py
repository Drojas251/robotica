from robotica_core.api.robotica_interface import RoboticaCore
from robotica_plugins.kinematics.definition import Kinematics_Plugins
from robotica_plugins.trajectory_planners.definition import Trajectory_Planners_Plugins
from robotica_plugins.path_planners.definition import Path_Planners_Plugins


class RobotAPI:
    def __init__(self, robot_yml_file, env_yml_file=None):
        self._robot = RoboticaCore(
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
        self._robot.joint_move(joint_trajectory)

    def get_current_joint_positions(self):
        return self._robot.get_current_joint_positions()
    
    def plan_and_execute(self, goal):
        self._robot.plan_and_execute(goal)

    def execute_path(self, path):
        self._robot.execute_path(path)
    
    ######################
    # Robot Core Interface
    ######################
    @property
    def robot_model(self):
        return self._robot.robot_model

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
        return self._robot.kinematics

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
        return self._robot.cartesian_trajectory_planner

    @property
    def controller(self):
        """ Returns Controller Interface

        Available Class Methods:
            execute_move
                args: 
                    trajectory (Trajectory)
        """
        return self._robot.controller

    @property
    def path_planner(self):
        """ Returns Path Planner Interface

        Available Class Methods:
            plan
                args: 
                    start (tuple)
                    goal (tuple)
        """
        return self._robot.path_planner


