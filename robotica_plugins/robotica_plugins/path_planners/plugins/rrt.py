from robotica_plugins.path_planners.path_planning_plugin_interface import PathPlannerPluginInterface
from robotica_datatypes.path_datatypes.waypoint import WayPoint
import random
import numpy as np

class treeNode():
    def __init__(self, locationX, locationY):
        self.locationX = locationX
        self.locationY = locationY
        self.children = []
        self.parent = None

class RRTAlgorithm():
    def __init__(self, start, goal, numIterations, stepSize, path_validate_func, vis_path_seg=None):
        self.randomTree = treeNode(start[0], start[1])
        self.start = start
        self.goal_pt = goal
        self.goal = treeNode(goal[0], goal[1])
        self.nearestNode = None          
        self.iterations = min(numIterations, 5000)
        self.rho = stepSize
        self.path_distance = 0
        self.nearestDist = 10000
        self.numWaypoints = 0
        self.Waypoints = []
        self.path_validate_func = path_validate_func
        self.vis_path_seg = vis_path_seg

    def addChild(self, locationX, locationY):
        if (locationX == self.goal.locationX):
            self.nearestNode.children.append(self.goal)
            self.goal.parent = self.nearestNode
        else:
            tempNode = treeNode(locationX, locationY)
            self.nearestNode.children.append(tempNode)
            tempNode.parent = self.nearestNode

        if self.vis_path_seg:
            start = (locationX, locationY)
            end = (self.nearestNode.locationX, self.nearestNode.locationY)
            self.vis_path_seg(start, end)
        
    def sampleAPoint(self):
        x = random.uniform(-1, 1)
        y = random.uniform(-1, 1)

        point = np.array([x, y])
        return point
    
    def steerToPoint(self, locationStart, locationEnd):
        offset = self.rho*self.unitVector(locationStart, locationEnd)
        point = np.array([locationStart.locationX + offset[0], locationStart.locationY + offset[1]])
        if point[0] >= 1:
            point[0] = 1
        if point[1] >= 1:
            point[1] = 1
        if point[0] <= -1:
            point[0] = -1
        if point[1] <= -1:
            point[1] = -1
        return point
    
    def isInObstacle(self, locationStart, locationEnd):
        try:
            x = locationEnd[0]
            y = locationEnd[1]
            valid = self.path_validate_func((x, y))

            if valid:
                return False
            else:
                return True
        except:
            return True

    def unitVector(self, locationStart, locationEnd):
        v = np.array([locationEnd[0] - locationStart.locationX, locationEnd[1] - locationStart.locationY])
        u_hat = v/np.linalg.norm(v)
        return u_hat
    
    def findNearest(self, root, point):
        if not root:
            return
        dist = self.distance(root, point)
        if dist < self.nearestDist:
            self.nearestDist = dist
            self.nearestNode = root
        for child in root.children:
            self.findNearest(child, point)

    def distance(self, node1, point):
        dist = np.sqrt((node1.locationX - point[0])**2 + (node1.locationY - point[1])**2)
        return dist
    
    def goalFound(self,point):
        if self.distance(self.goal, point) <= self.rho:
            return True
    
    def resetNearestValues(self):
        self.nearestNode = None
        self.nearestDist = 10000
        
    def retraceRRTPath(self,goal):
        if goal.locationX == self.randomTree.locationX:
            return

        self.numWaypoints += 1
        currentPoint = np.array([goal.locationX, goal.locationY])
        self.Waypoints.insert(0, currentPoint)
        self.path_distance += self.rho
        
        self.retraceRRTPath(goal.parent)   

    def plan(self):
        for i in range(self.iterations):
            self.resetNearestValues()
            point = self.sampleAPoint()
            self.findNearest(self.randomTree, point)
            new = self.steerToPoint(self.nearestNode, point) 

            if not self.isInObstacle(self.nearestNode, new):
                self.addChild(new[0], new[1]) 

                if (self.goalFound(new)):
                    self.addChild(self.goal_pt[0], self.goal_pt[1])
                    print("Goal Found")

                    self.retraceRRTPath(self.goal)
                    self.Waypoints.insert(0,self.start)
                    print("Number of waypoints: ", self.numWaypoints)
                    print("Path Distance (m): ", self.path_distance)    
                    print("Waypoints: ", self.Waypoints)

                    return self.Waypoints
        return None


class RRT(PathPlannerPluginInterface):
    def __init__(self, *args):
        PathPlannerPluginInterface.__init__(self, *args)

    def planner(self, start, goal):
        start_pt = np.array([start[0], start[1]])
        goal_pt = np.array([goal[0], goal[1]])
        numIterations = 1200
        stepSize = 0.1

        rrt = RRTAlgorithm(start_pt, goal_pt, numIterations, stepSize, self.validate_point, self.show_sample_segment)
        plan = rrt.plan()

        if not plan:
            print("Not a valid goal point")
            return None

        wpts = []
        for pt in plan:
            wpts.append(WayPoint((pt[0], pt[1]), 0.3))

        return wpts

