#!/usr/bin/env python

import rospy
import cv2
import numpy as np

from cv_bridge import CvBridge
from skimage.feature import hog
from sklearn.externals import joblib
from robotica_final.srv import ReadService
from robotica_final.srv import PhotoService


class NumberRecognition():
	"""Number Recognition class"""
	def __init__(self):
		self.frame = 0
		self.img = 0
		self.processed = 0
		self.clf = joblib.load("clasificador.pkl")
		self.bridge = CvBridge()

	# Service callback
	def serviceCallback(self, srv):
		self.takePhoto()
		self.imageContours()
		self.getDigits()
		return 3

	# Take a photo and do some filtering
	def takePhoto(self):
		image_srv = rospy.ServiceProxy('photo', PhotoService)
		data = image_srv()
		self.frame = self.bridge.imgmsg_to_cv2(data.imagen, "bgr8")
		grayed = cv2.cvtColor(self.frame, cv2.COLOR_BGR2GRAY)
		blurred = cv2.medianBlur(grayed, 13)
		ret, self.img = cv2.threshold(blurred, 100, 120, cv2.THRESH_BINARY)

	# Find relevant data from image
	def imageContours(self):
		x, y = self.img.shape
		arr = np.zeros((x, y, 3), np.uint8)
		self.final_contours = []
		image, contours, hierarchy = cv2.findContours(self.img.copy(), cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
		self.processed = self.img.copy()
		cnt= contours[1]
		cv2.drawContours(self.processed, [cnt], -1, color = (255, 255, 255), thickness = 20)
		for i in range(len(contours)):
		    cnt = contours[i]
		    if cv2.contourArea(cnt) > 3000 and cv2.contourArea(cnt) < 25000:
		        cv2.drawContours(self.processed, [cnt], -1, [0, 255, 255])
		        cv2.fillConvexPoly(arr, cnt, [255, 255, 255])
		        self.final_contours.append(cnt)

		# cv2.imshow('prueba2',self.processed)
		# cv2.waitKey(0)
		# cv2.destroyAllWindows()

	def getDigits(self):
		rects = [cv2.boundingRect(ctr) for ctr in self.final_contours]
		print(rects)
		for rect in rects:
			# Draw the rectangles
			cv2.rectangle(self.processed, (rect[0], rect[1]), (rect[0] + rect[2], rect[1] + rect[3]), (0, 255, 0), 3) 
			# Make the rectangular region around the digit
			leng = int(rect[3] * 1.2)
			pt1 = int(rect[1] + rect[3] // 2 - leng // 2)
			pt2 = int(rect[0] + rect[2] // 2 - leng // 2)
			roi = self.processed[pt1:pt1+leng, pt2:pt2+leng]
			# Resize the image
			roi = cv2.resize(roi, (784, 1), interpolation=cv2.INTER_AREA)
			roi = cv2.dilate(roi, (3, 3))
			# Calculate the HOG features
			nbr = self.clf.predict(roi)
			cv2.putText(self.processed, str(int(nbr[0])), (rect[0], rect[1]),cv2.FONT_HERSHEY_DUPLEX, 2, (0, 255, 255), 3)

		cv2.imshow("Resulting Image with Rectangular ROIs", self.processed)
		cv2.waitKey()


	def main(self):
		rospy.init_node('number_recognition')
		rospy.wait_for_service('photo')
		s = rospy.Service('password_service', ReadService, self.serviceCallback)
		while not rospy.is_shutdown():
			rospy.spin()


if __name__ == '__main__':
    try:
        num = NumberRecognition()
        num.main()
    except rospy.ROSInterruptException:
        pass

