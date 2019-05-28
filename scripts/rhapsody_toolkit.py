#!/usr/bin/env python

import rospy
import RPi.GPIO as GPIO
from geometry_msgs.msg import Twist





class RhapsodyToolkit():
	

	def __init__(self):

		# Speed variables
		self.linear_vel	 = 0.0
		self.angular_vel = 0.0
		
		# GPIO Pins config
		GPIO.setmode(GPIO.BCM)
		GPIO.setwarnings(False)
		
		# Motors pin setup
		GPIO.setup(11, GPIO.OUT)
		GPIO.setup(12, GPIO.OUT)
		GPIO.setup(13, GPIO.OUT)
		GPIO.setup(15, GPIO.OUT)

		# PWM objects
		MA1 = GPIO.PWM(11, 500)
		MA2 = GPIO.PWM(12, 500)
		MB1 = GPIO.PWM(13, 500)
		MB2 = GPIO.PWM(15, 500)

		# Motor PWM objects init
		MA1.start(0)
		MA2.start(0)
		MB1.start(0)
		MB2.start(0)

		# ROS node initialization
		rospy.init_node('rhapsody_toolkit', Anonymous=True)
		
		# ROS Subscriber object
		rospy.Subscriber("motor_vel", Twist, self.speedCallback)

		# ROS spin
		rospy.spin()


	def speedCallback(self, motorData):
		self.linear_vel  = motorData.linear.x
		self.angular_vel = motorData.angular.x

	
	def setSpeed(self):
		




