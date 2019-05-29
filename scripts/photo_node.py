#!/usr/bin/env python

import cv2
import rospy
import threading

from cv_bridge import CvBridge
from robotica_final.srv import PhotoService


class PhotoNode():
	"""PhotoNode string"""
	def __init__(self):
		self.vc = cv2.VideoCapture(0)
		self.img_thread = threading.Thread(target=self.updateFrame, args=())
		self.img_thread.daemon = True
		self.img_thread.start()
		self.bridge = CvBridge()


	def updateFrame(self):
		while True:
			if not self.vc.isOpened():
				raise Exception("Could not open video device")
			ret, self.frame = self.vc.read()


	def sendPhoto(self, args):
		return self.bridge.cv2_to_imgmsg(self.frame, "bgr8")


	def main(self):
		rospy.init_node('photo_node')
		s = rospy.Service('photo', PhotoService, self.sendPhoto)
		while not rospy.is_shutdown():
			rospy.spin()


if __name__ == '__main__':
    try:
        ph = PhotoNode()
        ph.main()
    except rospy.ROSInterruptException:
        pass