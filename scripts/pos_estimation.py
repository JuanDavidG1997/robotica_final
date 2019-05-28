#!/usr/bin/env python

import rospy
import math as m
from geometry_msgs.msg import Pose, Point, Twist
from std_msgs.msg import String, float32MultiArray
from robotica_final.srv import *
from master_msgs_iele3338.msg import Covariance

class posEstimator:

    def __init__(self):
        # Velocidades
        self.velLeft = 0
        self.velRight = 0
        # Pub Messages
        self.estimatedPos = Pose()
        self.estimatedUncertainty = Covariance()

    def robot_velocity_callback(self, data):
        self.velLeft = data.left
        self.velRight = data.right

    def main(self):
        # --------------------------- ROS ---------------------------
        # Node init
        rospy.init_node('pos_estimation', anonymous=False)
        # Topic subscriber
        rospy.Subscriber('robot_vel', float32MultiArray, self.robot_velocity_callback)
        # Topic publisher
        pubUncertainty = rospy.Publisher('robot_uncertainty', Covariance, queue_size=10)
        pubPos = rospy.Publisher('robot_position', Pose, queue_size=10)
        # Frequency rate
        rate = rospy.Rate(10)
        # --------------------- Local variables ---------------------

if __name__ == '__main__':
    try:
        estimator = posEstimator()
        estimator.main()
    except rospy.ROSInterruptException:
        pass