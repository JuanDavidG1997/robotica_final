#!/usr/bin/env python
import RPi.GPIO as GPIO
import rospy
import math as m
GPIO.setmode(GPIO.BCM)
from robotica_final.msg import realVel

# Constantes para los GPIO
LEFT_CHANNEL_A = 5
LEFT_CHANNEL_B = 6
RIGHT_CHANNEL_A = 19
RIGHT_CHANNEL_B = 13
# Constante de cuentas
COUNT_CTE = 1000.0


class encodReader:

    def __init__(self):
        # Counts
        self.leftEncoderCounts = 0.0
        self.rightEncoderCounts = 0.0
        # Inicializacion de los GPIO: Entrada y con pull-up
        GPIO.setup(LEFT_CHANNEL_A, GPIO.IN, pull_up_down=GPIO.PUD_UP)
        GPIO.setup(LEFT_CHANNEL_B, GPIO.IN, pull_up_down=GPIO.PUD_UP)
        GPIO.setup(RIGHT_CHANNEL_A, GPIO.IN, pull_up_down=GPIO.PUD_UP)
        GPIO.setup(RIGHT_CHANNEL_B, GPIO.IN, pull_up_down=GPIO.PUD_UP)
        # Pub Message
        self.vel = realVel()

    #Callbacks para cada senial de encoders (logica de cuadratura)
    def ChannelA_left(self, channel):
        if GPIO.input(LEFT_CHANNEL_A) == GPIO.input(LEFT_CHANNEL_B):
            self.leftEncoderCounts-=1
        else:
            self.leftEncoderCounts+=1

    def ChannelB_left(self, channel):
        if GPIO.input(LEFT_CHANNEL_A) == GPIO.input(LEFT_CHANNEL_B):
            self.leftEncoderCounts+=1
        else:
            self.leftEncoderCounts-=1

    def ChannelA_right(self, channel):
        if GPIO.input(RIGHT_CHANNEL_A) == GPIO.input(RIGHT_CHANNEL_B):
            self.rightEncoderCounts-=1
        else:
            self.rightEncoderCounts+=1

    def ChannelB_right(self, channel):
        if GPIO.input(RIGHT_CHANNEL_A) == GPIO.input(RIGHT_CHANNEL_B):
            self.rightEncoderCounts+=1
        else:
            self.rightEncoderCounts-=1

    def main(self):
        #Asignacion de interrupciones
        GPIO.add_event_detect(LEFT_CHANNEL_A, GPIO.BOTH, callback=self.ChannelA_left)
        GPIO.add_event_detect(LEFT_CHANNEL_B, GPIO.BOTH, callback=self.ChannelB_left)
        GPIO.add_event_detect(RIGHT_CHANNEL_A, GPIO.BOTH, callback=self.ChannelA_right)
        GPIO.add_event_detect(RIGHT_CHANNEL_B, GPIO.BOTH, callback=self.ChannelB_right)
        # --------------------------- ROS ---------------------------
        # Node init
        rospy.init_node('encod_reader', anonymous=False)
        # Topic publisher
        pubVel = rospy.Publisher('real_vel', realVel, queue_size=10)
        # Frequency rate
        rate = rospy.Rate(10)
        # --------------------- Local variables ---------------------
        prevTime = 0.0
        prevRrad = 0.0
        prevLrad = 0.0

        actualTime = 0.0
        actualRrad = 0.0
        actualLVrad = 0.0
        while not rospy.is_shutdown():
            # Calculate actual rad and time
            actualRrad = (self.rightEncoderCounts/COUNT_CTE)*2*m.pi
            actualLrad = (self.leftEncoderCounts/COUNT_CTE)*2*m.pi
            actualTime = rospy.get_time()
            #print('Cuentas Izquierda: {} \t Cuentas Derecha: {}'.format(self.leftEncoderCounts, self.rightEncoderCounts))
            # Calculate velocities
            self.vel.right = (actualRrad - prevRrad)/ (actualTime - prevTime)
            self.vel.left = (actualLrad - prevLrad) / (actualTime - prevTime)
            self.vel.time = actualTime
            # Publish  Message
            pubVel.publish(self.vel)

            # Update variables
            prevRrad = actualRrad
            prevLrad = actualLrad
            prevTime = actualTime

            #print('vel left: {} \t vel right: {} \t time: {}'.format(self.vel.left, self.vel.right, self.vel.time))
            # sleep
            rate.sleep()


if __name__ == '__main__':
    try:
        reader = encodReader()
        reader.main()
    except rospy.ROSInterruptException:
        pass
