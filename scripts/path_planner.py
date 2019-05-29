#!/usr/bin/env python
# License removed for brevity

# Necessary imports
import numpy as np
import rospy
from simple_rrt import RRT
from robotica_final.srv import PathService


class PathPlanner:
    def __init__(self):
        self.obstacle_list = None
        self.x_pos = 0
        self.y_pos = 0

    # Service Callback
    # Returns the path according to goal and algorithm
    def path_planning(self, dict):
        self.x_pos = dict.x_start
        self.y_pos = dict.y_start
        goal = (dict.x_goal, dict.y_goal)
        obstacle_list_srv = dict.obstacle_list
        algorithm = dict.algorithm
        pathx = None
        pathy = None
        n_obstacles = len(obstacle_list_srv)
        obstacle_list = np.zeros((n_obstacles, 3))
        for i in range(0, n_obstacles):
            obstacle = obstacle_list_srv[i]
            obstacle_list[i, 0] = obstacle.x
            obstacle_list[i, 1] = obstacle.y
            obstacle_list[i, 2] = obstacle.r
        self.obstacle_list = obstacle_list
        # Calls the method according to the requested algorithm
        if algorithm == 'RRT':
            pathx, pathy = self.rrt_planning(goal)
        p = [pathx, pathy]
        print("Service response: " + str(p))
        return p

    # Executes RRT path planning method
    def rrt_planning(self, goal):
        print("RRT requested. Goal is: " + str(goal))
        start = (self.x_pos, self.y_pos)
        goal = (goal[0], goal[1])
        # RRT object creation
        path_object = RRT(start, goal, self.obstacle_list, [0, 2500], expandDis=200, goalSampleRate=5, maxIter=400)
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

    # Principal method
    def main(self):
        # Node initialization
        rospy.init_node('path_planner', anonymous=False)
        # Service proxy declaration
        path = rospy.Service('path_planning', PathService, self.path_planning)
        rospy.spin()
        pass


if __name__ == '__main__':
    try:
        master = PathPlanner()
        master.main()
    except rospy.ROSInterruptException:
        pass
