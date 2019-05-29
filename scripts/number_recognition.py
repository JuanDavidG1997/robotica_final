#!/usr/bin/env python

import rospy
import cv2
import threading

from robotica_final.srv import ReadService

from PIL import Image
from sklearn.externals import joblib
from skimage.feature import hog
import numpy as np


class NumberRecognition():
	"""Number Recognition class"""
	def __init__(self):
		self.video_capture = cv2.VideoCapture(0)
		self.frame = 0
		self.img = 0
		self.img_thread = threading.Thread(target=self.updateFrame, args=())
		self.img_thread.daemon = True
		self.img_thread.start()

		self.clf = joblib.load("clasificador.pkl")

	# Service callback
	def serviceCallback(self, srv):
		self.takePhoto()
		self.imageContours()
		self.getDigits()
		return 1234

	def updateFrame(self):
		while True:
			if not self.video_capture.isOpened():
				raise Exception("Could not open video device")
			ret, self.frame = self.video_capture.read()

	# Take a photo and do some filtering
	def takePhoto(self):
		self.img = cv2.cvtColor(self.frame, cv2.COLOR_BGR2GRAY)
		self.img = cv2.GaussianBlur(self.img, (5, 5), 0)
		self.img = cv2.adaptiveThreshold(self.img, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY, 11, 2)

	# Find relevant data from image
	def imageContours(self):
		#
		for x in range(0, 1):
				# Get contours from binary image
				self.img, contours, hier = cv2.findContours(self.img.copy(), cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

				mask = np.zeros(self.img.shape, dtype='uint8')

				for contour in contours:
					area = cv2.contourArea(contour)
					if area < 5000.0:
						mask = cv2.drawContours(mask, [contour], -1, (255 , 255 , 255), thickness=cv2.FILLED)

				mask = cv2.bitwise_not(mask)
				self.img = cv2.bitwise_and(self.img, self.img, mask=mask)

		self.img, contours, hier = cv2.findContours(self.img.copy(), cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
		rects = [cv2.boundingRect(ctr) for ctr in contours]
		for rect in rects:
		    # Draw the rectangles
		    cv2.rectangle(self.img, (rect[0], rect[1]), (rect[0] + rect[2], rect[1] + rect[3]), (0, 0, 0), 3) 
		    # Make the rectangular region around the digit
		    leng = int(rect[3] * 1.6)
		    pt1 = int(rect[1] + rect[3] // 2 - leng // 2)
		    pt2 = int(rect[0] + rect[2] // 2 - leng // 2)
		    roi = self.img[pt1:pt1+leng, pt2:pt2+leng]
		    #roi = cv2.resize(roi, (784, 1), interpolation=cv2.INTER_AREA)
		    roi = cv2.resize(roi, (28, 28), interpolation=cv2.INTER_AREA)
		    kernel = np.ones((5,5), np.uint8)
		    roi = cv2.dilate(roi, kernel)
		    print(roi)
		    #cv2.waitKey()
		    try:
		    	nbr = self.clf.predict(roi)
		    	print(nbr)
		    	cv2.putText(im, str(int(nbr[0])), (rect[0], rect[1]),cv2.FONT_HERSHEY_DUPLEX, 2, (0, 255, 255), 1)
		    except:
		    	pass
		

	def getDigits(self):
		self.img = Image.fromarray(self.img)
		#self.img.show()
		


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

