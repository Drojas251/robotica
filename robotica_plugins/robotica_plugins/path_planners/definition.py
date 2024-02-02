from robotica_plugins.path_planners.plugins.rrt import RRT
from robotica_plugins.path_planners.plugins.test_planner import TestPlanner

Path_Planners_Plugins = {
RRT.__name__: RRT,
TestPlanner.__name__: TestPlanner,
}
