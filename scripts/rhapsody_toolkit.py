#!/usr/bin/env python

import rospy
import RPi.GPIO as GPIO
from std_msgs.msg import Int32

GPIO.setmode(GPIO.BCM)

PWM_FREQUENCY_LED = 255
RED_LED = 36
GREEN_LED = 38
BLUE_LED = 40
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
		super(RhapsodyToolkit, self).__init__()
		self.state = None
		# OutPut definition
		GPIO.setup(RED_LED, GPIO.OUT)
		GPIO.setup(GREEN_LED, GPIO.OUT)
		GPIO.setup(BLUE_LED, GPIO.OUT)
		# Frequency definition
		self.red = GPIO.PWM(RED_LED, PWM_FREQUENCY_LED)
		self.green = GPIO.PWM(GREEN_LED, PWM_FREQUENCY_LED)
		self.blue = GPIO.PWM(BLUE_LED, PWM_FREQUENCY_LED)
		# Color variable start
		self.r_color = 0
		self.g_color = 0
		self.b_color = 0
		# Starting pins
		self.red.start(255)
		self.green.start(255)
		self.blue.start(255)

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
		self.red.ChangeDutyCycle(self.r_color)
		self.green.ChangeDutyCycle(self.g_color)
		self.blue.ChangeDutyCycle(self.b_color)

	def main(self):
		# Node init
		rospy.init_node('rhapsody_toolkit', anonymous=False)
		# Topic Subscription
		rospy.Subscriber('state', Int32, self.state_callback)
		rate = rospy.Rate(10)
		while not rospy.is_shutdown():
			self.color_definition()
			rate.sleep()


if __name__ == '__main__':
	try:
		master = RhapsodyToolkit()
		master.main()
	except rospy.ROSInterruptException:
		pass






