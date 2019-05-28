#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Pose, Point, Twist
from std_msgs.msg import Int32
from master_msgs_iele3338.srv import AckService, EndService, StartService
from robotica_final.srv import MoveService, ReadService, PathService, EstimationService
from robotica_final.msg import obs

# Constants
GROUP = 15
IP = "192.168.2.3"

# Possible states
ACK_SERVICE = 1
READY_TO_START = 2
PATH_PLANNING = 3
MOVING = 4
READING_NUMBERS = 5
EMERGENCY_STOP = 6
FINISHED_TEST = 7


class RhapsodyMaster:

    def __init__(self):
        self.actual_state = 0
        self.start = None
        self.goal = None
        self.n_obstacles = None
        self.obstacles = None
        self.mov_state = None
        self.estimated_pos = None
        self.x_position = 0
        self.y_position = 0
        self.password = 0
        self.request_path = False
        self.start_orientation = 0
        self.goal_orientation = 0

# ----------------------------------------------------------------------------------------------------------------------
# -----------------------------------------------Topic Callbacks--------------------------------------------------------
# ----------------------------------------------------------------------------------------------------------------------
    def estimated_pos_callback(self, data):
        self.estimated_pos = data

    def mov_state_callback(self, data):
        self.mov_state = data

# ----------------------------------------------------------------------------------------------------------------------
# -----------------------------------------------Service handler--------------------------------------------------------
# ----------------------------------------------------------------------------------------------------------------------
    def handle_start_service(self, data):
        self.start = (data.start.position.x, data.start.position.y)
        self.start_orientation = data.start.orientation.w
        self.goal = (data.goal.position.x, data.goal.position.y)
        self.goal_orientation = data.goal.orientation.w
        self.n_obstacles = data.n_obstacles
        self.obstacles = data.obstacles
        self.request_path = True

# ----------------------------------------------------------------------------------------------------------------------
# ----------------------------------------------Service requests--------------------------------------------------------
# ----------------------------------------------------------------------------------------------------------------------

    def ask_for_ack_service(self):
        print("Requesting ack_service...")
        # rospy.wait_for_service('ack_service')
        try:
            print("Trying request")
            ack = rospy.ServiceProxy('ack_service', AckService)
            request = ack(GROUP, IP)
            print(request)
            return request.state
        except rospy.ServiceException:
            print("Service call to ack_service failed")

    def ask_for_path(self):
        print("Requesting path_planning...")
        self.change_state(PATH_PLANNING)
        x_start = self.start[0]
        y_start = self.start[1]
        x_goal = self.goal[0]
        y_goal = self.goal[1]
        algorithm = 'RRT'
        obstacle_list = []
        for i in range(0, self.n_obstacles):
            obstacle = self.obstacles[i]
            obstacle_append = obs(obstacle.position.position.x, obstacle.position.position.y, obstacle.radius)
            obstacle_list.append(obstacle_append)
        rospy.wait_for_service('path_planning')
        try:
            path = rospy.ServiceProxy('path_planning', PathService)
            request = path(x_start, y_start, x_goal, y_goal, obstacle_list, algorithm)
            return request
        except rospy.ServiceException:
            print("Service call to path_planner failed")

    def ask_for_move(self, path):
        print("Requesting movement...")
        k = 0
        while k < 1000000:
            msg = Twist()
            msg.linear.x = 1
            msg.angular.x = 1
            self.cmd_vel.publish(msg)
            self.change_state(MOVING)
            k = k + 1
        rospy.wait_for_service('move')
        try:
            move = rospy.ServiceProxy('move', MoveService)
            request = move(path, self.goal_orientation)

        except rospy.ServiceException:
            print("Service call to move failed")


    def ask_for_read(self):
        print("Requesting number reading...")
        self.change_state(READING_NUMBERS)
        rospy.wait_for_service('read')
        try:
            read = rospy.ServiceProxy('read', ReadService)
            request = read()
            return request.data.password
        except rospy.ServiceException:
            print("Service call to read failed")

    def ask_for_end_service(self):
        print("Requesting end service...")
        rospy.wait_for_service('end_service')
        try:
            end_service = rospy.ServiceProxy('end_service', EndService)
            request = end_service(self.password)
            return request.correct
        except rospy.ServiceException:
            print("Service call to end service failed")

    def ask_for_estimation_service(self):
        print('Requesting estimation service...')
        rospy.wait_for_service('start_estimation')
        try:
            start_estimation = rospy.ServiceProxy('start_estimation', EstimationService)
            pose_estimation = Pose()
            pose_estimation.position.x = self.start.x
            pose_estimation.position.y = self.start.y
            pose_estimation.orientation.w = self.start_orientation
            request = start_estimation(pose_estimation)
        except rospy.ServiceException:
            print("Service call to pose estimation failed")

# ----------------------------------------------------------------------------------------------------------------------
# -----------------------------------------------Additional methods-----------------------------------------------------
# ----------------------------------------------------------------------------------------------------------------------
    def change_state(self, p_state):
        self.actual_state = p_state
        return self.actual_state

# ----------------------------------------------------------------------------------------------------------------------
# ----------------------------------------------------Main--------------------------------------------------------------
# ----------------------------------------------------------------------------------------------------------------------

    def main(self):
        # Node init
        rospy.init_node('rhapsody_master', anonymous=False)
        # Service provider
        start = rospy.Service('start_service', StartService, self.handle_start_service)
        # Topic subscriber
        rospy.Subscriber('estimated_pos', Point, self.estimated_pos_callback)
        rospy.Subscriber('mov_state', Int32, self.mov_state_callback)
        # Topic publisher
        state_publisher = rospy.Publisher('state', Int32, queue_size=10)
        self.cmd_vel = rospy.Publisher('cmd_vel', Twist, queue_size=10)
        # Local variables
        ready_to_start = False
        correct_password = 0
        # Frequency set
        rate = rospy.Rate(10)
        # Local cycle
        while not rospy.is_shutdown():
            state_publisher.publish(self.actual_state)
            # Ready to start. ACK_SERVICE
            while not ready_to_start:
                ready_to_start = self.ask_for_ack_service()
                if ready_to_start == 1:
                    self.change_state(READY_TO_START)
            #self.ask_for_estimation_service()
            # Received Start_Service
            # Asking for path. PATH_SERVICE
            if self.request_path:
                path = self.ask_for_path()
                # Asking for movement. ASK_MOVE
                self.ask_for_move(path)
            # If arrived. ASK_READ
            if self.mov_state == 3:
                self.password = self.ask_for_read()
                # Checking received password. ASK_END_SERVICE
                correct_password = self.ask_for_end_service()
            # FINISHED_TEST
            if correct_password == 1:
                self.change_state(FINISHED_TEST)
                print("Ended test")
            rate.sleep()


if __name__ == '__main__':
    try:
        master = RhapsodyMaster()
        master.main()
    except rospy.ROSInterruptException:
        pass