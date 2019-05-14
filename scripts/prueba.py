#!/usr/bin/env python

import rospy
import RPi.GPIO as GPIO
from geometry_msgs.msg import Pose
from master_msgs_iele3338.srv import AckService, EndService, StartService

# CONSTANTES
GRUPO = 15
PIN_MOTOR_L = 24
PIN_MOTOR_R = 25

class Prueba():
	# Constructor
	def __init__(self):
		# Atributes
		# States
		self.readyToStart = False
		self.moving = False
		# Info from the map
		self.startPoint = Pose()
		self.finalPoint = Pose()
		self.nObstacle = 0
		self.obstacles = []
		self.obstacleDiamters = []
		# IP
		self.ipAddress = "192.168.2.3"

		# Movement
		# Output ports definition
		GPIO.setmode(GPIO.BCM)
		GPIO.setwarnings(False)
		GPIO.setup(PIN_MOTOR_R, GPIO.OUT)
		GPIO.setup(PIN_MOTOR_L, GPIO.OUT)
		# Motor vel initialization
		self.velMotorR = GPIO.PWM(PIN_MOTOR_R, 100)
		self.velMotorR.start(0)
		self.velMotorL = GPIO.PWM(PIN_MOTOR_L, 100)
		self.velMotorL.start(0)

	# Ack Service request
	def askAckService(self, ip):
		print("Esperando el servicio...")
		rospy.wait_for_service('ack_service')
		try:
			ack = rospy.ServiceProxy('ack_service', AckService)
			response = ack(GRUPO, ip)
                        print(response)
			return response.state
		except rospy.ServiceException, e:
			print "Service call to ack service failed: %s" %e

	def handleStartService(self, data):
		self.startPoint = data.start
		self.finalPoint = data.goal
		self.nObstacle = data.n_obstacles
		self.obstacles = data.obstacles_array
		self.moving = True

	def move(self, velR, velL):
		self.velMotorR.ChangeDutyCycle(velR)
		self.velMotorL.ChangeDutyCycle(velL)

        def stop(self):
                self.velMotorR.ChangeDutyCycle(0)
                self.velMotorL.ChangeDutyCycle(0)
                self.moving = False
                

	# Main method
	def main(self):
		# ------- ROS -------
		# Node init
		rospy.init_node('test_node', anonymous=False)
		# Services
		start = rospy.Service('start_service', StartService, self.handleStartService)
		# Rate
		rate = rospy.Rate(10)

		# Loop Cycle
		while not rospy.is_shutdown():
			# Ask for ack service
			while not self.readyToStart:
				self.readyToStart = self.askAckService(self.ipAddress)
			# Move if start service was asked
			if self.moving:
				self.move(20,20)
                                rospy.Timer(rospy.Duration(5),self.stop,True)
			else:
				self.move(0,0)

		rate.sleep()

if __name__ == '__main__':
    try:
        p = Prueba()
        p.main()
    except rospy.ROSInterruptException:
        pass
