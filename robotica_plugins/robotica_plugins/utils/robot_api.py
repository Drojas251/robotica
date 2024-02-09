from robotica_core.api.robotica_interface import RoboticaCore

class RobotAPI:
    def __init__(self, 
                 robot_yml_file, 
                 env_yml_file=None, 
                 kinematics_plugins=None, 
                 cartesian_traj_plugins=None, 
                 path_planner_plugins=None,
        ):

        self._robot = RoboticaCore(
            robot_yml_file, 
            env_yml_file,
            kinematics_plugins = kinematics_plugins, 
            cartesian_traj_plugins = cartesian_traj_plugins,
            path_planner_plugins = path_planner_plugins,
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
        
    def inverse_kinematics(self, point, cfg=0):
        return self._robot.kinematics.inverse_kinematics(point, cfg)
    
    def forward_kinematics(self, joints):
        return self._robot.kinematics.forward_kinematics(joints)
    
    def plan(self, start, goal):
        return self._robot.path_planner.plan(start, goal)


