from robotica_plugins.path_planners.path_planning_plugin_interface import PathPlannerPluginInterface
from robotica_datatypes.path_datatypes.waypoint import WayPoint

class TestPlanner(PathPlannerPluginInterface):
    def __init__(self, collision_checker):
        PathPlannerPluginInterface.__init__(self, collision_checker)

    def planner(self, start, goal):
        return [WayPoint(start, 0.3), WayPoint(goal, 0.3)]

