""" Plugin Builder 

THIS FILE MUST BE IN robotica_plugins/robotica_plugins for the plugin builder to work
"""

import os
from robotica_core.utils.plugin_loader import PluginLoader

# Get path to plugin in directory 
plugin_path = os.getcwd()
os.environ["ROBOTICA_PLUGIN_PATH"] = plugin_path

# Build Plugins
PluginLoader(robotica_plugin_dir="kinematics", group_name="Kinematics_Plugins")
PluginLoader(robotica_plugin_dir="trajectory_planners", group_name="Trajectory_Planners_Plugins")
PluginLoader(robotica_plugin_dir="path_planners", group_name="Path_Planners_Plugins")