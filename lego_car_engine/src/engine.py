#!/usr/bin/env python
import rospy
import time
import RPi.GPIO as GPIO
from lego_car_msgs.msg import EnginePower


#---- GPIO der LEGO Motoren. Pro Motor 3 Pins ---
# Pins werden mit BCM Beschriftung angegeben
# Enable Pin = Motor kann angesteuer werden
# Back Pin = Motor fährt rückwärts
# For Pin = Motor fährt vorwärts

left_for = 25
left_back = 12
left_enable = 5

right_for = 20
right_back = 16
right_enable = 21



def engine_output(data):
    print(data.left_engine)
    print(data.right_engine)

    if data.left_engine >= 1:
        GPIO.output(left_enable, 1)
        GPIO.output(left_back, 0)
        GPIO.output(left_for, 1)
    else if data.left_engine <= -1:
        GPIO.output(left_enable, 1)
        GPIO.output(left_back, 1)
        GPIO.output(left_for, 0)
    else:
        GPIO.output(left_enable, 0)


    if data.right_engine >= 1:
        GPIO.output(right_enable, 1)
        GPIO.output(right_back, 0)
        GPIO.output(right_for, 1)
    else if data.right_engine <= -1:
        GPIO.output(right_enable, 1)
        GPIO.output(right_back, 1)
        GPIO.output(right_for, 0)
    else:
        GPIO.output(right_enable, 0)


        


def init_engine():
    GPIO.setmode(GPIO.BCM)

    GPIO.setup(left_for, GPIO.output)
    GPIO.setup(left_back, GPIO.output)
    GPIO.setup(left_enable, GPIO.output)

    GPIO.setup(right_for, GPIO.output)
    GPIO.setup(right_back, GPIO.output)
    GPIO.setup(right_enable, GPIO.output)


    GPIO.output(left_enable, 0)
    GPIO.output(right_enable, 0)



def engine_controller():
    rospy.init_node('Engine Controller')
    rospy.Subscriber("enginePowerControl", EnginePower, engine_output)
    rospy.spin()



if __name__ == '__main__':
    init_engine()
    engine_controller()
