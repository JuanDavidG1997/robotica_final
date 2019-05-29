#!/usr/bin/env python

import rospy
from robotica_final.srv import ReadService


class NumberRecognition():
	"""Number Recognition class"""
	def __init__(self):
		pass

	# Service callback
	def serviceCallback(self, srv):
		return 0000


	def main(self):
		rospy.init_node('number_recognition')
		s = rospy.Service('password_service', ReadService, self.serviceCallback)
		while not rospy.is_shutdown():
			rospy.spin()


if __name__ == '__main__':
    try:
        num = NumberRecognition()
        num.main()
    except rospy.ROSInterruptException:
        pass

