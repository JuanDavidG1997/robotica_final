#!/usr/bin/env python

import rospy
import math as m
import numpy as np
from geometry_msgs.msg import Pose, Point, Twist
from std_msgs.msg import String, Float32MultiArray
from robotica_final.srv import *
from master_msgs_iele3338.msg import Covariance
from robotica_final.msg import realVel

# Constantes
L = 80.0
radius = 29.3/2

class posEstimator:

    def __init__(self):
        # Velocidades
        self.angularVelLeft = 0.0
        self.angularVelRight = 0.0
        self.timeStamp = 0.0
        # Pub Messages
        self.estimated = Pose()
        self.actualPos = [0.0, 0.0, 0.0]
        self.prevPos = [0.0, 0.0, 0.0]
        self.estimatedUncertainty = Covariance()

        self.sigmaP = np.matrix([[0.0, 0.0, 0.0 ],[0.0, 0.0, 0.0],[0.0, 0.0, 0.0]])

    def real_vel_callback(self, data):
        self.angularVelLeft = data.left
        self.angularVelRight = data.right
        self.timeStamp = data.time

    def handle_estimate_service(self, data):
        self.actualPos = [data.position.x, data.position.y, data.orientation.w]

    def main(self):
        # --------------------------- ROS ---------------------------
        # Node init
        rospy.init_node('pos_estimation', anonymous=False)
        # Topic subscriber
        rospy.Subscriber('real_vel', realVel, self.real_vel_callback)
        # Topic publisher
        pubUncertainty = rospy.Publisher('robot_uncertainty', Covariance, queue_size=10)
        pubPos = rospy.Publisher('robot_position', Pose, queue_size=10)
        # Service provider
        estimate = rospy.Service('start_estimation', EstimationService, self.handle_estimate_service)
        # Frequency rate
        rate = rospy.Rate(10)
        # --------------------- Local variables ---------------------
        # Pos estimation
        prevVelR = 0.0
        prevVelL = 0.0
        prevTime = 0.1
        actualVelR = 0.0
        actualVelL = 0.0
        actualTime = 0.0
        # Covariance estimation
        Fp = np.matrix([[0.0, 0.0, 0.0],[0.0, 0.0, 0.0],[0.0, 0.0, 0.0]])
        FdS = np.matrix([[0.0, 0.0, 0.0],[0.0, 0.0, 0.0],[0.0, 0.0, 0.0]])
        sigmadS = np.matrix([[0.0, 0.0, 0.0],[0.0, 0.0, 0.0],[0.0, 0.0, 0.0]])

        # Local while cycle
        while not rospy.is_shutdown():
            # --------------------- Estimate Position ---------------------
            # Set actual Vel
            actualVelL = self.angularVelLeft*radius
            actualVelR = self.angularVelRight*radius
            actualTime = self.timeStamp


            # Calculate delta for each wheel
            deltaSL = (actualVelL)*(actualTime - prevTime)
            deltaSR = (actualVelR)*(actualTime - prevTime)
            # Calculate local delta
            deltaTheta = (deltaSR - deltaSL)/(2*L)
            deltaS = (deltaSL + deltaSR)/2

            # Update Pos
            theta = self.actualPos[2]
            self.actualPos[0] += deltaS * m.cos(theta + deltaTheta/2)
            self.actualPos[1] += deltaS * m.sin(theta + deltaTheta/2)
            self.actualPos[2] += deltaTheta
            # Build and publish message
            self.estimated.position.x = self.actualPos[0]
            self.estimated.position.y = self.actualPos[1]
            self.estimated.orientation.w = self.actualPos[2]
            pubPos.publish(self.estimated)


            # Update prev variables
            prevVelL = actualVelL
            prevVelR = actualVelR
            prevTime = actualTime

            # --------------------- Estimate Covariance ---------------------
            # Intermediate terms
            Fp = np.matrix([[1.0, 0.0, -deltaS*m.sin(theta + deltaTheta/2)],[0.0, 1.0, -deltaS*m.cos(theta + deltaTheta/2)],[0.0, 0.0, 1.0]])
            ang = theta + deltaTheta/2
            fila1 = [0.5*m.cos(ang) - (deltaS/(4*L))*m.sin(ang), 0.5*m.cos(ang) + (deltaS/(4*L))*m.sin(ang)]
            fila2 = [0.5*m.sin(ang) + (deltaS/(4*L))*m.cos(ang), 0.5*m.sin(ang) - (deltaS/(4*L))*m.cos(ang)]
            fila3 = [1/(2*L), 1/(2*L)]
            FdS = np.matrix([fila1,fila2,fila3])
            sigmadS = np.matrix([[0.1*deltaSR, 0.0 ],[0.0, 0.1*deltaSL]])
            # Build Covariance
            self.sigmaP = Fp*self.sigmaP*np.transpose(Fp) + FdS*sigmadS*np.transpose(FdS)

            # Build and publish message
            self.estimatedUncertainty.sigma11 = self.sigmaP[0,0]
            self.estimatedUncertainty.sigma12 = self.sigmaP[0,1]
            self.estimatedUncertainty.sigma13 = self.sigmaP[0,2]
            self.estimatedUncertainty.sigma21 = self.sigmaP[1,0]
            self.estimatedUncertainty.sigma22 = self.sigmaP[1,1]
            self.estimatedUncertainty.sigma23 = self.sigmaP[1,2]
            self.estimatedUncertainty.sigma31 = self.sigmaP[2,0]
            self.estimatedUncertainty.sigma32 = self.sigmaP[2,1]
            self.estimatedUncertainty.sigma33 = self.sigmaP[2,2]
            #pubUncertainty.publish(self.estimatedUncertainty)

            rate.sleep()


if __name__ == '__main__':
    try:
        estimator = posEstimator()
        estimator.main()
    except rospy.ROSInterruptException:
        pass
