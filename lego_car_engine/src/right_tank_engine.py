#!/usr/bin/env python3
import rospy
import time
import math
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

        GPIO.setup(right_for, GPIO.OUT)
        GPIO.setup(right_back, GPIO.OUT)
        GPIO.setup(right_enable, GPIO.OUT)

        GPIO.output(right_enable, 0)

        rospy.init_node('RightEngineController')
        rospy.Subscriber("enginePowerControl", EnginePower, self.receive_data)

        self.right_sleep = MAXSLEEP
        self.mode = EnginePower.TANKMODE
        self.speed = 0


    def engine_carmode(self, data):
        #print("CARMODE")
        self.speed = data.speed
        # For easier Calculation, of the Sleep Timers the angel will be added to the Speed itself
        if data.steering_angle > 0:
            data.speed /= 1
        else:
            data.speed *= 1 - math.sqrt((abs(data.steering_angle) / MAXANGLE))
        
        if data.speed == 0:
            self.right_sleep = MAXSLEEP
            return

        self.right_sleep = abs((MAXSLEEP / MAXSPEED) * math.sqrt(abs(data.speed)))

        


    def engine_tankmode(self, data):
        #print("TANKMODE")
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
        GPIO.output(right_enable, 1)

        #Left Engine
        if self.speed > 0:
            GPIO.output(right_back, 0)
            GPIO.output(right_for,1)
            # rospy.logdebug("Right-Sleep: %f", MAXSLEEP - self.right_sleep)
            time.sleep(self.right_sleep)
            GPIO.output(right_for,0)
            time.sleep(MAXSLEEP - self.right_sleep)
        
        
        elif self.speed < 0:
            GPIO.output(right_for, 0)
            GPIO.output(right_back,1)
            time.sleep(self.right_sleep)
            GPIO.output(right_back,0)
            time.sleep(MAXSLEEP - self.right_sleep)
        else:
            GPIO.output(right_enable, 0)



    def run(self):
        while not rospy.is_shutdown():
            self.output()
    

if __name__ == '__main__':
    engine = Engine()
    engine.run()
