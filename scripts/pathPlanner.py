#!/usr/bin/env python
# License removed for brevity

# Necessary imports
import math as m
import networkx as nx
import numpy as np
import rospy
from taller3.srv import *
from geometry_msgs.msg import Point, Twist
from simple_rrt import RRT
from graph_2 import graph_building

# Global variables
# Obstacle list
obstacle_list = np.empty((5, 3))
# Pioneer Position
x_pos = 0
y_pos = 0


# Callback to get the position and radius of each obstacle
# Updating obstacle list
def c1_position_callback(data):
    global obstacle_list
    obstacle_list[0, 0] = data.x
    obstacle_list[0, 1] = data.y
    obstacle_list[0, 2] = data.z


def c2_position_callback(data):
    global obstacle_list
    obstacle_list[1, 0] = data.x
    obstacle_list[1, 1] = data.y
    obstacle_list[1, 2] = data.z


def c3_position_callback(data):
    global obstacle_list
    obstacle_list[2, 0] = data.x
    obstacle_list[2, 1] = data.y
    obstacle_list[2, 2] = data.z


def c4_position_callback(data):
    global obstacle_list
    obstacle_list[3, 0] = data.x
    obstacle_list[3, 1] = data.y
    obstacle_list[3, 2] = data.z


def c5_position_callback(data):
    global obstacle_list
    obstacle_list[4, 0] = data.x
    obstacle_list[4, 1] = data.y
    obstacle_list[4, 2] = data.z


# Callback that updates the position of the pioneerRobot
def pioneer_position_callback(data):
    global x_pos, y_pos
    x_pos = data.linear.x
    y_pos = data.linear.y


# Service Callback
# Returns the path according to goal and algorithm
def path_planning(dict):
    goal = (dict.x, dict.y)
    algorithm = dict.algorithm
    pathx = None
    pathy = None
    # Calls the method according to the requested algorithm
    if algorithm == 'RRT':
        pathx, pathy = rrt_planning(goal)
    elif algorithm == 'Astar':
        pathx, pathy = a_star_path_planning(goal)
    p = [pathx, pathy]
    print("Service response: " + str(p))
    return p

# Executes RRT path planning method
def rrt_planning(goal):
    print("RRT requested. Goal is: " + str(goal))
    start = (x_pos, y_pos)
    goal = (goal[0], goal[1])
    # RRT object creation
    path_object = RRT(start, goal, obstacle_list, [-4, 11], expandDis=1.2, goalSampleRate=5, maxIter=500)
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
def a_star_path_planning(goal):
    print("Astar requested. Goal is: " + str(goal))
    # Graph creation
    G = graph_building()
    # Rounding goal and start to known nodes
    distORigin = [m.sqrt((xy[0] - x_pos) ** 2 + (xy[1] - y_pos) ** 2) for xy in G.nodes()]
    distGoal = [m.sqrt((xy[0] - goal[0]) ** 2 + (xy[1] - goal[1]) ** 2) for xy in G.nodes()]
    nodeOrigin = np.argmin(distORigin)
    nodeGoal = np.argmin(distGoal)
    nodes = list(G.nodes())
    # Requesting path
    path = nx.astar_path(G, nodes[nodeOrigin], nodes[nodeGoal], heuristic=heuristic)
    path_x = []
    path_y = []
    # Building service response
    for i in range(0, len(path)):
        path_x.append(path[i][0])
        path_y.append(path[i][1])
    return path_x, path_y


# Euclidean distance calculation
def heuristic(node_a, node_b):
    (x1, y1) = node_a
    (x2, y2) = node_b
    h_value = m.sqrt((x1-x2)**2+(y1-y2)**2)
    return h_value


# Principal method
def path_planner():
    # Node initialization
    rospy.init_node('path_planner', anonymous=True)
    # Topic subscription
    rospy.Subscriber('c1Position', Point, c1_position_callback)
    rospy.Subscriber('c2Position', Point, c2_position_callback)
    rospy.Subscriber('c3Position', Point, c3_position_callback)
    rospy.Subscriber('c4Position', Point, c4_position_callback)
    rospy.Subscriber('c5Position', Point, c5_position_callback)
    rospy.Subscriber('pioneerPosition', Twist, pioneer_position_callback)
    # Service proxy declaration
    path = rospy.Service('path_planning', pathService, path_planning)
    rospy.spin()


if __name__ == '__main__':
    try:
        path_planner()
    except rospy.ROSInterruptException:
        pass