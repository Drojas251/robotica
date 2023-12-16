import os
import yaml
from robotica_core.kinematics.robot_model import RobotModel
from robotica_core.control.controller_interface import ControllerInterface


class RoboticaCore:
    PLUGIN_FILE_PATH = "~/.robotica/plugins"
    SELECTED_PLUGIN_FILE = "run_plugins.yml"

    def __init__(self, robot_yml_file, kinematics_plugins=None, cartesian_traj_plugins=None):
        self._robot_model = RobotModel(robot_yml_file)
        self._kinematics = None
        self._cartesian_trajectory_planner = None

        shared_plugin_path = os.path.expanduser(self.PLUGIN_FILE_PATH)
        run_plugin_path = os.path.join(shared_plugin_path, "run_plugins.yml")
        with open(run_plugin_path, "r") as yaml_file:
            self.plugin_dict = yaml.safe_load(yaml_file)

        self._load_kinematics_plugins(kinematics_plugins)
        self._load_cartesian_traj_plugins(cartesian_traj_plugins)

        self._controller = ControllerInterface()

    def _load_kinematics_plugins(self, kin_plugins):
        if kin_plugins == None:
            raise Exception("No Kinematics Plugin Loaded")

        kinematics = None
        for plugin_cls_name in self.plugin_dict.values():
            if plugin_cls_name in kin_plugins:
                print(f"Using Kinematics Plugin: {plugin_cls_name}")
                kinematics_cls = kin_plugins[plugin_cls_name]
                kinematics = kinematics_cls(self._robot_model)
        
        if kinematics == None:
            raise Exception("Kinematics Plugin Not Found")

        self._kinematics = kinematics

    def _load_cartesian_traj_plugins(self, traj_plugins):
        if traj_plugins == None:
            raise Exception("No Cartesian Trajectory Plugin Loaded")

        traj_planner = None
        for plugin_cls_name in self.plugin_dict.values():
            if plugin_cls_name in traj_plugins:
                print(f"Using Trajectory Plugin: {plugin_cls_name}")
                traj_planner_cls = traj_plugins[plugin_cls_name]
                traj_planner = traj_planner_cls(self._kinematics)

        if traj_planner == None:
            raise Exception("Cartesian Trajectory Plugin Not Found")

        self._cartesian_trajectory_planner = traj_planner

