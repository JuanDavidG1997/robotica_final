#!/usr/bin/env python

import rospy
import math as m
from geometry_msgs.msg import Pose, Twist
from std_msgs.msg import Int32
from robotica_final.srv import *

# Threshold
THRESHOLD_LIN = 300
THRESHOLD_ANG = 0.2
# States
WAITING = 0
MOVING = 1
EMERGENCY_STOP = 2
FINISHED = 3
POSITIONING = 4
# Control variables
K_RHO = 0.3
K_ALPHA = 0.8
K_BETA = -0.01
K = [K_RHO, K_ALPHA, K_BETA]


# Main Class for the robot control
class RobotControl:

    def __init__(self):
        # Position Attributes
        self.xPos = 0.0
        self.yPos = 0.0
        self.theta = 0.0
        self.finalTheta = 0.0
        # Path Attributes
        self.path = []
        self.currentGoal = [0.0, 0.0, 0.0]
        # State Attributes
        self.state = WAITING
        # self.moving = False
        # self.positioning = False
        # Pub Messages
        self.vel = Twist()
        self.movementState = Int32()

    def handle_move_service(self, msg):
        self.path = []
        self.path.append(msg.pathx)
        self.path.append(msg.pathy)
        self.finalTheta = msg.w
        self.state = MOVING

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
        rho = m.sqrt(deltax** 2 + deltay** 2)
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
        # Return velocities
        return [v, w]

    def main(self):
        # --------------------------- ROS ---------------------------
        # Node init
        rospy.init_node('robot_control', anonymous=False)
        # Service provider
        move = rospy.Service('move', MoveService, self.handle_move_service)
        # Topic subscriber
        rospy.Subscriber('robot_position', Pose, self.estimated_pos_callback)
        # Topic publisher
        pubState = rospy.Publisher('mov_state', Int32, queue_size=10)
        pubVel = rospy.Publisher('cmd_vel', Twist, queue_size=10)
        # Frequency rate
        rate = rospy.Rate(10)
        # --------------------- Local variables ---------------------
        # Iteration variables
        p = 1
        arrivedToCurrentGoal = True
        # Velocities
        v = 0
        w = 0

        # Local while cycle
        while not rospy.is_shutdown():
            # Verify if there is an obstacle near
            if self.verify_if_obstacle():
                self.state = EMERGENCY_STOP
                v = 0
                w = 0

            # Positioning State
            if self.state == POSITIONING:
                # Verify if finish
                finishCondition = abs(self.theta - self.currentGoal[2]) < THRESHOLD_ANG
                if finishCondition:
                    self.state = FINISHED
                    v = 0
                    w = 0
                else:
                    # Verify orientation
                    oriented = (abs(self.theta - self.currentGoal[2]) < THRESHOLD_ANG)
                    if not oriented:
                        orientationError = self.theta - self.currentGoal[2]
                        if abs(orientationError) > m.pi / 2:
                            w = orientationError / m.pi
                            v = 0
                        else:
                            w = - orientationError / m.pi
                            v = 0

            # Moving State
            if self.state == MOVING:
                # --------------------- Next Goal ---------------------
                # Verify if arrived to last goal
                if p == len(self.path[0]):
                    self.state = POSITIONING
                # Arrived to Next point condition
                if arrivedToCurrentGoal:
                    self.currentGoal = [self.path[0][p], self.path[1][p], self.finalTheta]
                    p = p + 1
                # --------------------- CONTROL ---------------------
                # Calculate directions
                [rho, alpha, beta] = self.calculate_direction()
                # V and W calculations
                [v, w] = self.calculate_velocities(rho, alpha, beta)
                # Verify if arrived to actual goal
                arrivedToCurrentGoal = (abs(rho) < THRESHOLD_LIN)

            # Set Messages
            self.vel.linear.x = v
            self.vel.angular.x = w
            self.movementState = self.state
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
