#!/usr/bin/env python

import rospy
import RPi.GPIO as GPIO
from geometry_msgs.msg import Twist
from std_msgs.msg import Int32
from robotica_final.msg import realVel
import numpy as np


PWM_FREQUENCY_LED = 100
RED_LED = 16
GREEN_LED = 20
BLUE_LED = 21
MA1_PIN = 17
MA2_PIN = 18
MB1_PIN = 27
MB2_PIN = 22
ACK_SERVICE = 1
READY_TO_START = 2
PATH_PLANNING = 3
MOVING = 4
READING_NUMBERS = 5
EMERGENCY_STOP = 6
FINISHED_TEST = 7
ACK_SERVICE_COLOR = [255, 255, 255]
READY_TO_START_COLOR = [255, 255, 255]
PATH_PLANNING_COLOR = [255, 255, 255]
MOVING_COLOR = [255, 255, 255]
READING_NUMBERS_COLOR = [255, 255, 255]
EMERGENCY_STOP_COLOR = [255, 255, 255]
FINISHED_TEST_COLOR = [255, 255, 255]


class RhapsodyToolkit():

	"""docstring for RhapsodyToolkit"""
	def __init__(self):

		# Speed variables
		self.linear_vel = 0.0
		self.angular_vel = 0.0

		# Motor angular velocity
		self.omegaL_real = 0.0
		self.omegaR_real = 0.0

		# PI controller variables
		self.KP = 1.0
		self.KI = 2.0
		self.omegaRError = 0.0
		self.omegaLError = 0.0
		self.omegaRIntegralError = 0.0
		self.omegaLIntegralError = 0.0
		self.lastTimeLowLevel = 0.0

		# State variable
		self.state = ACK_SERVICE

		# Color variable start
		self.r_color = 0
		self.g_color = 0
		self.b_color = 0

		# GPIO Pins config
		GPIO.setmode(GPIO.BCM)
		#GPIO.setwarnings(False)

		# Motors pin setup
		GPIO.setup(MA1_PIN, GPIO.OUT)
		GPIO.setup(MA2_PIN, GPIO.OUT)
		GPIO.setup(MB1_PIN, GPIO.OUT)
		GPIO.setup(MB2_PIN, GPIO.OUT)

		# LED output definition
		GPIO.setup(RED_LED, GPIO.OUT)
		GPIO.setup(GREEN_LED, GPIO.OUT)
		GPIO.setup(BLUE_LED, GPIO.OUT)

		# PWM objects
		self.MA1 = GPIO.PWM(MA1_PIN, 500)
		self.MA2 = GPIO.PWM(MA2_PIN, 500)
		self.MB1 = GPIO.PWM(MB1_PIN, 500)
		self.MB2 = GPIO.PWM(MB2_PIN, 500)

		# Frequency definition
		self.red = GPIO.PWM(RED_LED, PWM_FREQUENCY_LED)
		self.green = GPIO.PWM(GREEN_LED, PWM_FREQUENCY_LED)
		self.blue = GPIO.PWM(BLUE_LED, PWM_FREQUENCY_LED)

		# Motor PWM objects init
		self.MA1.start(0)
		self.MA2.start(0)
		self.MB1.start(0)
		self.MB2.start(0)

		# Starting pins
		self.red.start(100)
		self.green.start(100)
		self.blue.start(100)


	def speedCallback(self, motorData):
		self.linear_vel = motorData.linear.x
		self.angular_vel = motorData.angular.x


	def calculateLowLevelControl(self):
		omegaR_setpoint = 1
		omegaL_setpoint = 1

		self.omegaRError = omegaR_setpoint - self.omegaR_real
		self.omegaLError = omegaL_setpoint - self.omegaL_real
		
		self.omegaRIntegralError += self.omegaRError*(rospy.get_time() - self.lastTimeLowLevel)
		self.omegaLIntegralError += self.omegaLError*(rospy.get_time() - self.lastTimeLowLevel)

		self.lastTimeLowLevel = rospy.get_time()


		omegaRControlAction = np.clip(self.KP * self.omegaRError + self.KI * self.omegaRIntegralError, -100, 100)
		omegaLControlAction = np.clip(self.KP * self.omegaLError + self.KI * self.omegaLIntegralError, -100, 100)

		if omegaRControlAction>0:
			self.MA1.ChangeDutyCycle(0)
			self.MA2.ChangeDutyCycle(int(omegaRControlAction))
		else:
			self.MA2.ChangeDutyCycle(0)
			self.MA1.ChangeDutyCycle(int(omegaRControlAction))


		if omegaLControlAction>0:
			self.MB1.ChangeDutyCycle(0)
			self.MB2.ChangeDutyCycle(int(omegaLControlAction))
		else:
			self.MB2.ChangeDutyCycle(0)
			self.MB1.ChangeDutyCycle(int(omegaLControlAction))

		print(omegaL_real, "\t", omegaR_real)


		

	def realVelCallback(self, velData):
		self.omegaR_real = velData.right
		self.omegaL_real = velData.left


	def state_callback(self, data):
		self.state = data


	def color_definition(self):
		#Check actual state
		if self.state == ACK_SERVICE:
			self.r_color = ACK_SERVICE_COLOR[0]
			self.g_color = ACK_SERVICE_COLOR[1]
			self.b_color = ACK_SERVICE_COLOR[2]

		elif self.state == READY_TO_START:
			self.r_color = READY_TO_START_COLOR[0]
			self.g_color = READY_TO_START_COLOR[1]
			self.b_color = READY_TO_START_COLOR[2]

		elif self.state == PATH_PLANNING:
			self.r_color = PATH_PLANNING_COLOR[0]
			self.g_color = PATH_PLANNING_COLOR[1]
			self.b_color = PATH_PLANNING_COLOR[2]

		elif self.state == MOVING:
			self.r_color = MOVING_COLOR[0]
			self.g_color = MOVING_COLOR[1]
			self.b_color = MOVING_COLOR[2]

		elif self.state == READING_NUMBERS:
			self.r_color = READING_NUMBERS_COLOR[0]
			self.g_color = READING_NUMBERS_COLOR[1]
			self.b_color = READING_NUMBERS_COLOR[2]

		elif self.state == EMERGENCY_STOP:
			self.r_color = EMERGENCY_STOP_COLOR[0]
			self.g_color = EMERGENCY_STOP_COLOR[1]
			self.b_color = EMERGENCY_STOP_COLOR[2]

		elif self.state == FINISHED_TEST:
			self.r_color = FINISHED_TEST_COLOR[0]
			self.g_color = FINISHED_TEST_COLOR[1]
			self.b_color = FINISHED_TEST_COLOR[2]

		# PWM generation
		self.red.ChangeDutyCycle(self.r_color/255*100)
		self.green.ChangeDutyCycle(self.g_color/255*100)
		self.blue.ChangeDutyCycle(self.b_color/255*100)

	def main(self):
		
		# Node init
		rospy.init_node('rhapsody_toolkit', anonymous=False)
		self.lastTimeLowLevel = rospy.get_time()
		
		# Topic Subscription
		rospy.Subscriber('state', Int32, self.state_callback)
		rospy.Subscriber('cmd_vel', Twist, self.speedCallback)
		rospy.Subscriber('real_vel', realVel, self.realVelCallback)
		rate = rospy.Rate(10)
		
		while not rospy.is_shutdown():
			self.color_definition()
			self.calculateLowLevelControl()
			rate.sleep()


if __name__ == '__main__':
	try:
		master = RhapsodyToolkit()
		master.main()
	except rospy.ROSInterruptException:
		pass


		




