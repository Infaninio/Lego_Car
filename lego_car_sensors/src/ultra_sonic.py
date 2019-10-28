#!/usr/bin/env python
import rospy
import time
import RPi.GPIO as GPIO
from sensor_msgs.msg import Range



MOTA = 26
MOTB = 19
MOTC = 13
MOTD = 6
GPIO_TRIGGER = 23
GPIO_ECHO = 24




def stepper_step(self, a, b, c, d):
        GPIO.output(MOTA, a)
        GPIO.output(MOTB, b)
        GPIO.output(MOTC, c)
        GPIO.output(MOTD, d)
        #time.sleep(0.001)


def get_distance():
        # Trigger for the UltraSonic Sensor, now it starts measuring the distance 
        GPIO.output(GPIO_TRIGGER, True)
        # set Trigger for 0.00001 sec
        time.sleep(0.00001)
        GPIO.output(GPIO_TRIGGER, False)

        #Time between sending and receiving the sound impuls
        start_time = time.time()
        stop_time = time.time()

        # save Start Time
        while GPIO.input(GPIO_ECHO) == 0 and start_time - stop_time < 1:
            start_time = time.time()

        # save End Time
        while GPIO.input(GPIO_ECHO) == 1 and stop_time - start_time < 0.51:
            stop_time = time.time()

        # Difference between start and end
        time_elapsed = stop_time - start_time
        # multiply with the speed of sound (34300 cm/s
        # divide by two
        distance = (time_elapsed * 34300) / 2

        return distance



def init_sensor():
    #Initialising of the used GPIO Pins
    GPIO.setmode(GPIO.BCM)
    GPIO.setup(MOTA, GPIO.OUT)
    GPIO.setup(MOTB, GPIO.OUT)
    GPIO.setup(MOTC, GPIO.OUT)
    GPIO.setup(MOTD, GPIO.OUT)
    GPIO.setup(GPIO_TRIGGER, GPIO.OUT)
    GPIO.setup(GPIO_ECHO, GPIO.IN)




def node_run():
    rospy.init_node("UltraSonicSensor")
    pub = rospy.Publisher("UltraSonicRange", Range)
    
    #rate for measuring and publishing the distance 10 = 10Hz = 10 times per second
    rate = rospy.Rate(10)

    data = Range()

    while not rospy.is_shutdown():
        #measure and publish
        data.radiation_type = data.ULTRASOUND
        data.field_of_view = 0.0
        data.min_range = 0.01
        data.max_range = 10.0
        #devide by 100, Function returns cm, Message expects metres
        data.range = get_distance() / 100.0
        pub.publish(data)
        rate.sleep()


if __name__ == '__main__':
    init_sensor()
    
    try:
        node_run()
    except rospy.ROSInitException:
        pass

