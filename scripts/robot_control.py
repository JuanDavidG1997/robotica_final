#!/usr/bin/env python

import rospy
import math as m
from geometry_msgs.msg import Pose, Point, Twist
from std_msgs.msg import Int16, String
from robotica_final.srv import MoveService, PathService
from robotica_final.srv import *

# Constants
THRESHOLD_LIN = 0.1
THRESHOLD_ANG = 0.2

# Control variables
K_RHO = 1.25
K_ALPHA = 1.5
K_BETA = -0.5
K = [K_RHO, K_ALPHA, K_BETA]

class RobotControl:

    def __init__(self):
        # Position Atributes
        self.xPos = 0.0
        self.yPos = 0.0
        self.theta = 0.0
        # Path Atributes
        self.path = []
        self.currentGoal = [0, 0, 0]
        # State Atributes
        self.moving = False
        self.positioning = False
        self.movementState = Int16()
        # Vel atribute
        self.vel = Twist()

    def handle_move_service(self, msg):
        self.path[:][0] = msg.pathx
        self.path[:][1] = msg.pathy
        self.moving = True

    def estimated_pos_callback(self, msg):
        self.xPos = msg.position.x
        self.yPos = msg.position.y
        self.theta = msg.orientation.w

    def verify_if_obstacle(self):
        obstacle = False
        return obstacle

    def calculate_direction(self):
        # Direction Variables calculation
        deltay = self.currentGoal[1] - self.yPos
        deltax = self.currentGoal[0] - self.xPos
        rho = m.sqrt((deltax) ** 2 + (deltay) ** 2)
        alpha = m.atan2(deltay, deltax) - self.theta
        beta = - self.theta - alpha
        # Angles only between -pi and pi
        if beta > m.pi:
            beta = beta - 2 * m.pi
        if beta < -m.pi:
            beta = beta + 2 * m.pi
        if alpha > m.pi:
            alpha = alpha - 2 * m.pi
        if alpha < -m.pi:
            alpha = alpha + 2 * m.pi
        # Print varaibles
        print("------ Calculating Direction... ----")
        print("Pos: (" + str(self.xPos) + ", " + str(self.yPos) + ", " + str(self.theta * 180 / m.pi) + ")")
        print("Current Goal: (" + str(self.currentGoal[0]) + ", " + str(self.currentGoal[1]) + ", " + str(self.currentGoal[2] * 180 / m.pi) + ")")
        print("Distance to goal: " + str(rho))
        print("Alpha: " + str(alpha * 180 / m.pi) + " grad")
        print("Beta: " + str(beta * 180 / m.pi) + " grad")
        return [rho, alpha, beta]

    def calculate_velocities(self, rho, alpha, beta):
        # Linear Vel
        if (-m.pi / 2 < alpha and alpha < m.pi / 2):
            v = K[0] * rho
        else:
            v = -K[0] * rho
        # Angular Vel
        w = K[1] * alpha + K[2] * beta
        # Verify if finished
        if self.positioning:
            # Verify orientation
            if (abs(rho) < THRESHOLD_LIN) and not (abs(self.theta - self.currentGoal[2]) < THRESHOLD_ANG):
                orientationError = self.theta - self.currentGoal[2]
                if abs(orientationError) > m.pi / 2:
                    w = orientationError / m.pi
                    v = 0
                else:
                    w = - orientationError / m.pi
                    v = 0

        # Verify if finish
        finished = abs(rho) < THRESHOLD_LIN and abs(self.theta - self.currentGoal[2]) < THRESHOLD_ANG
        if finished:
            self.movementState = 2
            v = 0
            w = 0
        # Return velocities
        return [v, w]


    def main(self):
        # --------------------------- ROS ---------------------------
        # Node init
        rospy.init_node('robot_control', anonymous=False)
        # Service provider
        move = rospy.Service('move', MoveService, self.handle_move_service)
        # Topic subscriber
        rospy.Subscriber('estimated_pos', Pose, self.estimated_pos_callback)
        # Topic publisher
        pubState = rospy.Publisher('mov_state', Int16, queue_size=10)
        pubVel = rospy.Publisher('cmd_vel', Twist, queue_size=10)
        # Frequency set
        rate = rospy.Rate(10)
        # --------------------- Local variables ---------------------
        # Iteration varaible
        p = 1
        # State variables
        arrived = False

        # Local while cycle
        while not rospy.is_shutdown():
            # Verify if there is an obstacle near
            obstacle = self.verify_if_obstacle()
            # Verify moving condition is true
            if self.moving and not obstacle:
                self.movementState = 1
                # --------------------- Next Goal ---------------------
                # Finish condition
                if p == len(self.path[1][:]):
                    self.positioning = True
                # Arrived to Next point condition
                if arrived and (not self.positioning):
                    self.currentGoal = [self.path[0][p], self.path[1][p], 0]
                    p = p + 1
                # --------------------- CONTROL ---------------------
                # Calculate directions
                [rho, alpha, beta] = self.calculate_direction()
                # Verify if arrived to actual goal
                arrived = (abs(rho) < THRESHOLD_LIN)
                # V and W calculations
                [v, w] = self.calculate_velocities(rho, alpha, beta)
            # If not moving set vel and state to 0
            else:
                self.movementState = 0
                v = 0
                w = 0
            # Set vel message
            self.vel.linear.x = v
            self.vel.angular.x = w
            # Print vel and state
            print("Vel Linear: " + str(v))
            print("Vel Angular: " + str(w))
            print("State: " + str(self.movementState))
            # Publish vel and state
            pubVel.publish(self.vel)
            pubState.publish(self.movementState)

            rate.sleep()

if __name__ == '__main__':
    try:
        rControl = RobotControl()
        rControl.main()
    except rospy.ROSInterruptException:
        pass