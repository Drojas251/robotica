""" Plugin Builder 

THIS FILE MUST BE IN robotica_plugins/robotica_plugins for the plugin builder to work
"""

import os
from robotica_core.utils.plugin_builder import Plugin, PluginBuilder

# Get path to plugin in directory 
plugin_path = os.getcwd()
os.environ["ROBOTICA_PLUGIN_PATH"] = plugin_path

# Build Plugins
kinematics_plugin = Plugin(robotica_plugin_dir="kinematics", group_name="Kinematics_Plugins")
traj_planner_plugin = Plugin(robotica_plugin_dir="trajectory_planners", group_name="Trajectory_Planners_Plugins")
path_planner_plugin = Plugin(robotica_plugin_dir="path_planners", group_name="Path_Planners_Plugins")

PluginBuilder([kinematics_plugin, traj_planner_plugin, path_planner_plugin])