#!/usr/bin/env python
import rospy
import time
import RPi.GPIO as GPIO
from lego_car_msgs.msg import EnginePower


#---- GPIO of the LEGO electric motor. 3 pins for each motor ---
# Pins are named with the BCM overlay
# the Enable Pin enables the Engine, ot
# Back Pin = Motor goes backwars
# For Pin = Motor goes forwards
left_for = 25
left_back = 12
left_enable = 5

right_for = 20
right_back = 16
right_enable = 21



#Callback function for the Subscriber 
#TODO implementing output if the CARMODE is used.
def engine_output(data):
    print(data.left_engine)
    print(data.right_engine)

    if data.left_engine >= 1:
        #left engine full speed forwards
        GPIO.output(left_enable, 1)
        GPIO.output(left_back, 0)
        GPIO.output(left_for, 1)
    elif data.left_engine <= -1:
        #left engine full speed backwards
        GPIO.output(left_enable, 1)
        GPIO.output(left_back, 1)
        GPIO.output(left_for, 0)
    else:
        #left engine stop/disable
        GPIO.output(left_enable, 0)


    if data.right_engine >= 1:
        #right engine full speed forwards
        GPIO.output(right_enable, 1)
        GPIO.output(right_back, 0)
        GPIO.output(right_for, 1)
    elif data.right_engine <= -1:
        #right engine full speed backwards
        GPIO.output(right_enable, 1)
        GPIO.output(right_back, 1)
        GPIO.output(right_for, 0)
    else:
        #right engine stop/disable
        GPIO.output(right_enable, 0)

     

#Initialising of the different GPIOs
def init_engine():
    GPIO.setmode(GPIO.BCM)

    GPIO.setup(left_for, GPIO.OUT)
    GPIO.setup(left_back, GPIO.OUT)
    GPIO.setup(left_enable, GPIO.OUT)

    GPIO.setup(right_for, GPIO.OUT)
    GPIO.setup(right_back, GPIO.OUT)
    GPIO.setup(right_enable, GPIO.OUT)


    GPIO.output(left_enable, 0)
    GPIO.output(right_enable, 0)


def engine_controller():
    rospy.init_node('EngineController')
    rospy.Subscriber("enginePowerControl", EnginePower, engine_output)
    rospy.spin()



if __name__ == '__main__':
    init_engine()
    engine_controller()
