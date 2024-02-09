""" Plugin Builder 

THIS FILE MUST BE IN robotica_plugins/robotica_plugins for the plugin builder to work
"""

import os
from robotica_core.utils.plugin_builder import RoboticaPlugin, PluginBuilder
from robotica_plugins.kinematics.kinematics_plugin_interface import KinematicPluginInterface
from robotica_plugins.trajectory_planners.trajectory_planning_interface import CartesianTrajectoryPluginInterface
from robotica_plugins.path_planners.path_planning_plugin_interface import PathPlannerPluginInterface

def confirm_robotica_ws(dir_path):
    if os.path.exists(dir_path):
        filepath = os.path.join(dir_path, "robotica_ws.txt")
        return os.path.exists(filepath)
    else:
        return False

def build_plugins(robotica_ws_path):
    check_ws = confirm_robotica_ws(robotica_ws_path)

    if not check_ws:
        print("Not a valid robotica workspace")
        return
    
    # Build Plugins
    kinematics_plugin = RoboticaPlugin(ws_path=robotica_ws_path, plugin_type="kinematics", group_name="Kinematics_Plugins", interface_cls=KinematicPluginInterface)
    traj_planner_plugin = RoboticaPlugin(ws_path=robotica_ws_path, plugin_type="trajectory_planners", group_name="Trajectory_Planners_Plugins", interface_cls=CartesianTrajectoryPluginInterface)
    path_planner_plugin = RoboticaPlugin(ws_path=robotica_ws_path, plugin_type="path_planners", group_name="Path_Planners_Plugins", interface_cls=PathPlannerPluginInterface)

    PluginBuilder([kinematics_plugin, traj_planner_plugin, path_planner_plugin])

