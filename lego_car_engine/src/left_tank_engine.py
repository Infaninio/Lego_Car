#!/usr/bin/env python3
import rospy
import math
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

MAXSLEEP = 0.05
MAXSPEED = math.sqrt(100)
MAXANGLE = math.pi / 2


class Engine():
    def __init__(self):
        GPIO.setmode(GPIO.BCM)

        GPIO.setup(left_for, GPIO.OUT)
        GPIO.setup(left_back, GPIO.OUT)
        GPIO.setup(left_enable, GPIO.OUT)

        GPIO.output(left_enable, 0)

        rospy.init_node('LeftEngineController')
        rospy.Subscriber("enginePowerControl", EnginePower, self.receive_data)

        self.left_sleep = MAXSLEEP
        self.mode = EnginePower.TANKMODE
        self.speed = 0


    def engine_carmode(self, data):
        #print("CARMODE")
        self.speed = data.speed
        # For easier Calculation, of the Sleep Timers the angel will be added to the Speed itself
        if data.steering_angle < 0:
            data.speed /= 1
        else:
            data.speed *= 1 - math.sqrt((abs(data.steering_angle) / MAXANGLE))
 
        if data.speed == 0:
            self.left_sleep = MAXSLEEP
            return

        self.left_sleep = abs((MAXSLEEP / MAXSPEED) * math.sqrt(abs(data.speed)))

        


    def engine_tankmode(self, data):
        # rospy.logerr("TANKMODE")
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


    #Callback function for the Subscriber 
    #TODO implementing output if the CARMODE is used.
    def receive_data(self, data):
        #Distinguish which Mode is used, and call the appropriat function
        if data.mode == EnginePower.TANKMODE:
            self.engine_tankmode(data)
            self.mode = EnginePower.TANKMODE
        else:
            self.engine_carmode(data)
            self.mode = EnginePower.CARMODE


    def output(self):
        if self.mode == EnginePower.TANKMODE:
            return
        
        #Enable Engines but without output
        GPIO.output(left_enable, 1)

        #Left Engine
        if self.speed > 0:
            GPIO.output(left_back, 0)

            GPIO.output(left_for,1)
            time.sleep(self.left_sleep)
            GPIO.output(left_for,0)
            time.sleep(MAXSLEEP - self.left_sleep)
        
        
        elif self.speed < 0:
            GPIO.output(left_for, 0)

            GPIO.output(left_back,1)
            time.sleep(self.left_sleep)
            GPIO.output(left_back,0)
            time.sleep(MAXSLEEP - self.left_sleep)
        else:
            GPIO.output(left_enable, 0)



    def run(self):
        while not rospy.is_shutdown():
            self.output()
    


#Initialising of the different GPIOs


if __name__ == '__main__':
    engine = Engine()
    engine.run()
