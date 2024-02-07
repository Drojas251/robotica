""" Plugin Builder 

THIS FILE MUST BE IN robotica_plugins/robotica_plugins for the plugin builder to work
"""

import os
from robotica_core.utils.plugin_builder import Plugin, PluginBuilder
from robotica_plugins.kinematics.kinematics_plugin_interface import KinematicPluginInterface
from robotica_plugins.trajectory_planners.trajectory_planning_interface import CartesianTrajectoryPluginInterface
from robotica_plugins.path_planners.path_planning_plugin_interface import PathPlannerPluginInterface


def build():
    # Build Plugins
    kinematics_plugin = Plugin(robotica_plugin_dir="kinematics", group_name="Kinematics_Plugins", interface_cls=KinematicPluginInterface)
    traj_planner_plugin = Plugin(robotica_plugin_dir="trajectory_planners", group_name="Trajectory_Planners_Plugins", interface_cls=CartesianTrajectoryPluginInterface)
    path_planner_plugin = Plugin(robotica_plugin_dir="path_planners", group_name="Path_Planners_Plugins", interface_cls=PathPlannerPluginInterface)

    PluginBuilder([kinematics_plugin, traj_planner_plugin, path_planner_plugin])

if __name__ == "__main__":
    build()
