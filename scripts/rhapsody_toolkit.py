#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist


class RhapsodyToolkit():
	

	def __init__(self):
		
		rospy.init_node('rhapsody_toolkit', Anonymous=True)
		
		rospy.Subscriber("motor_vel", Twist, speedCallback)

		rospy.spin()

	def speedCallback(motorData):








