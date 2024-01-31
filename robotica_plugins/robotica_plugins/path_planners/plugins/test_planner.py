from robotica_plugins.path_planners.path_planning_plugin_interface import PathPlannerPluginInterface
from robotica_datatypes.path_datatypes.waypoint import WayPoint

class TestPlanner(PathPlannerPluginInterface):
    def __init__(self, *args):
        PathPlannerPluginInterface.__init__(self, *args)

    def planner(self, start, goal):
        if not self.validate_point(goal):
            print("Not a valid goal point")
            return None

        return [WayPoint(start, 0.3), WayPoint(goal, 0.3)]

