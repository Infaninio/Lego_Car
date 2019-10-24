#!/usr/bin/env python
import rospy
import time
import RPi.GPIO as GPIO
from lego_car_msgs.msg import EnginePower


def engineOutput(data):
    print(data.left_engine)
    print(data.right_engine)


def engineController():
    rospy.init_node('engineController')
    rospy.Subscriber("enginePowerControl", EnginePower, engineOutput)
    rospy.spin()



if __name__ == '__main__':
    engineController()
