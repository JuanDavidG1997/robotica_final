#!/usr/bin/env python

import rospy
import math as m
import numpy as np
from geometry_msgs.msg import Pose, Point, Twist
from std_msgs.msg import String, float32MultiArray
from robotica_final.srv import *
from master_msgs_iele3338.msg import Covariance

class posEstimator:

    def __init__(self):
        # Velocidades
        self.velLeft = 0.0
        self.velRight = 0.0
        self.timeStamp = 0.0
        # Pub Messages
        self.estimatedPos = Pose()
        self.estimatedUncertainty = Covariance()

    def real_vel_callback(self, data):
        self.velLeft = data.left
        self.velRight = data.right
        self.timeStamp = data.time

    def main(self):
        # --------------------------- ROS ---------------------------
        # Node init
        rospy.init_node('pos_estimation', anonymous=False)
        # Topic subscriber
        rospy.Subscriber('real_vel', float32MultiArray, self.real_vel_callback)
        # Topic publisher
        pubUncertainty = rospy.Publisher('robot_uncertainty', Covariance, queue_size=10)
        pubPos = rospy.Publisher('robot_position', Pose, queue_size=10)
        # Frequency rate
        rate = rospy.Rate(10)
        # --------------------- Local variables ---------------------
        xPos = 0.0
        yPos = 0.0

        # Local while cycle
        while not rospy.is_shutdown():
            l = 20
            r_theta = np.array([[m.cos(theta), m.sin(theta), 0], [-1 * m.sin(theta), m.cos(theta), 0], [0, 0, 1]])
            alpha1 = m.pi / 2
            beta1 = 0
            alpha2 = -m.pi / 2
            beta2 = m.pi
            J1 = np.array([[m.sin(alpha1 + beta1), -1 * m.cos(alpha1 + beta1), -1 * l * m.cos(beta1)],
                           [m.sin(alpha2 + beta2), -1 * m.cos(alpha2 + beta2), -1 * l * m.cos(beta2)],
                           [m.cos(alpha1 + beta1), m.sin(alpha1 + beta1), l * m.sin(beta1)]])
            phi = np.array([v1, v2, 0])
            phi.shape = (3, 1)
            global_vel = np.matmul(inv(r_theta), np.matmul(inv(J1), phi))
            theoretical_x = (x_theoretical_position[-1] + global_vel[0] * time)
            theoretical_y = (y_theoretical_position[-1] + global_vel[1] * time)
            x_theoretical_position.append(theoretical_x)
            y_theoretical_position.append(theoretical_y)



if __name__ == '__main__':
    try:
        estimator = posEstimator()
        estimator.main()
    except rospy.ROSInterruptException:
        pass