#!/usr/bin/env python

import rospy
import cv2
from std_msgs.msg import Int32
from robotica_final.srv import ReadService


class number_reading:

    def __init__(self):
        self.photo_path = 'image.png'
        self.symbol = None
        self.password = None

    def open_cam(self):
        cv2.namedWindow("preview")
        vc = cv2.VideoCapture(0)

        if vc.isOpened(): # try to get the first frame
            rval, frame = vc.read()
        else:
            rval = False

        while rval:
            cv2.imshow("preview", frame)
            rval, frame = vc.read()
            key = cv2.waitKey(20)
            if key == 27: # exit on ESC
                break
        cv2.destroyWindow("preview")

    def ask_for_code(self):

        return self.symbol


    def convert_to_int(self):
        self.password=int(self.symbol)
        return self.password

    def main(self):
        #Node init
        rospy.init_node('number_reading', anonymous=False)
        #Topic publisher
        state_publisher = rospy.Publisher('state', Int32, queue_size=10)
        #Frequency set
        rate = rospy.Rate(10)
        picture = rospy.Service('read_service', ReadService, self.open_cam())


        self.ask_for_code()
        self.convert_to_int()
        state_publisher.publish(self.password)
        rospy.spin()

if __name__ == '__main__':
    try:
        master = number_reading()
        master.main()
    except rospy.ROSInterruptException:
        pass
