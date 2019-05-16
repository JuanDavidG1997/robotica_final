#!/usr/bin/env python
# License removed for brevity

# Necessary imports
import math as m
import networkx as nx
import numpy as np
import rospy
from simple_rrt import RRT
from Graph import Graph
from robotica_final.srv import PathService


class PathPlanner:
    def __init__(self):
        self.obstacle_list = None
        self.x_pos = 0
        self.y_pos = 0

    # Callback that updates the position of the pioneerRobot
    def pioneer_position_callback(self, data):
        self.x_pos = data.linear.x
        self.y_pos = data.linear.y

    # Service Callback
    # Returns the path according to goal and algorithm
    def path_planning(self, dict):
        goal = (dict.x, dict.y)
        algorithm = dict.algorithm
        pathx = None
        pathy = None
        self.obstacle_list = dict.obstacle_list
        # Calls the method according to the requested algorithm
        if algorithm == 'RRT':
            pathx, pathy = self.rrt_planning(goal)
        elif algorithm == 'Astar':
            pathx, pathy = self.a_star_path_planning(goal)
        p = [pathx, pathy]
        print("Service response: " + str(p))
        return p

    # Executes RRT path planning method
    def rrt_planning(self, goal):
        print("RRT requested. Goal is: " + str(goal))
        start = (self.x_pos, self.y_pos)
        goal = (goal[0], goal[1])
        # RRT object creation
        path_object = RRT(start, goal, self.obstacle_list, [-4, 11], expandDis=1.2, goalSampleRate=5, maxIter=500)
        # Requesting path
        path = path_object.Planning(False)
        # Organizing path
        path.reverse()
        path_x = []
        path_y = []
        # Building service response
        for i in range(0, len(path)):
            path_x.append(path[i][0])
            path_y.append(path[i][1])
        return path_x, path_y

    # Executes Astar path planning method
    def a_star_path_planning(self, goal):
        print("Astar requested. Goal is: " + str(goal))
        # Graph creation
        G = graph_building(self.obstacle_list)
        # Rounding goal and start to known nodes
        distORigin = [m.sqrt((xy[0] - x_pos) ** 2 + (xy[1] - y_pos) ** 2) for xy in G.nodes()]
        distGoal = [m.sqrt((xy[0] - goal[0]) ** 2 + (xy[1] - goal[1]) ** 2) for xy in G.nodes()]
        nodeOrigin = np.argmin(distORigin)
        nodeGoal = np.argmin(distGoal)
        nodes = list(G.nodes())
        # Requesting path
        path = nx.astar_path(G, nodes[nodeOrigin], nodes[nodeGoal], heuristic=self.heuristic)
        path_x = []
        path_y = []
        # Building service response
        for i in range(0, len(path)):
            path_x.append(path[i][0])
            path_y.append(path[i][1])
        return path_x, path_y

    # Euclidean distance calculation
    def heuristic(self, node_a, node_b):
        (x1, y1) = node_a
        (x2, y2) = node_b
        h_value = m.sqrt((x1-x2)**2+(y1-y2)**2)
        return h_value

    # Principal method
    def main(self):
        # Node initialization
        rospy.init_node('path_planner', anonymous=True)
        # Service proxy declaration
        path = rospy.Service('path_planning', PathService, self.path_planning)
        rospy.spin()