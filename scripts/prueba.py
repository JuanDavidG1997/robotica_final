#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Pose
from master_msgs_iele3338.srv import AckService, EndService, StartService


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

	# Ack Service request
	def askAckService(self, ip):
		print("Esperando el servicio...")
		rospy.wait_for_service('ack_service')
		try:
			ack = rospy.ServiceProxy('ack_service', AckService)
			response = ack(15, ip)
			return response
		except rospy.ServiceException, e:
			print "Service call to ack service failed: %s" %e

	def handleStartService(self, data):
		self.startPoint = data.start
		self.finalPoint = data.goal
		self.nObstacle = data.n_obstacles
		self.obstacles = data.obstacles_array
		self.moving = True

	# Main method
	def main(self):
		ipAddress = ""
		# Node init
		rospy.init_node('test_node', anonymous=False)
		# Services
		start = rospy.Service('start_service', StartService, self.handleStartService)

		# Rate
		rate = rospy.Rate(10)

		while not rospy.is_shutdown():

			while not self.readyToStart:
				self.readyToStart = self.askAckService(ipAddress)

		rate.sleep()

if __name__ == '__main__':
    try:
        p = Prueba()
        p.main()
    except rospy.ROSInterruptException:
        pass