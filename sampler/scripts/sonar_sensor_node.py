#!/usr/bin/env python3

import RPi.GPIO as gpio
import time
import sys
import signal
import rospy
from std_msgs.msg import Float32

def signal_handler(signal, frame):           # ctrl + c -> exit program
        print('You pressed Ctrl+C!')
        sys.exit(0)
signal.signal(signal.SIGINT, signal_handler)


class sonar():
    def __init__(self):
        rospy.init_node('sonar', anonymous=True)
        self.distance_publisher = rospy.Publisher('/watersampling/sonar_dist',Float32, queue_size=1)
        self.r = rospy.Rate(15)
    def dist_sendor(self,dist):
        data = Float32()
        data.data=dist
        self.distance_publisher.publish(data)
        
        
gpio.setmode(gpio.BCM)
gpio.setwarnings(False)                     # gives if a gpio pin is used as both input and output
#trig = 27 # 7th
#echo = 17 # 6th

trig = 12                                   #BCM pin 12, BOARD pin 32

#gpio.setup(trig, gpio.OUT)
#gpio.setup(echo, gpio.IN)

sensor=sonar()
time.sleep(0.5)
print ('-----------------------------------------------------------------sonar start')
try :
    while True :
        gpio.setup(trig, gpio.OUT)
        gpio.output(trig, 0)
        time.sleep(0.02)                    # The sonar sleeps for 20 miliseconds
        #time.sleep(1)
        gpio.output(trig, 1)
        time.sleep(0.000005)                # The sonar sends out a pin for 5 microseconds
        gpio.output(trig, 0)
        gpio.setup(trig, gpio.IN)
        while gpio.input(trig) == 0 :       # Records the time immediately after the Ping is sent. 
                                            #After the ping is sent, the trig pin stays LOW for an instant.
                                            #then it stays high until it receives the echo.
            pulse_start = time.time()       # time.time() returns time in seconds since the epoch as a floating point number
        while gpio.input(trig) == 1 :       # until the echo returns, it will record the time of flight.
            pulse_end = time.time()
        pulse_duration = pulse_end - pulse_start
        distance = pulse_duration * 170    # 2s = vt, therefore, s = (v/2)*t. Speed of sound is v=340 m/s or 34000 cm/s. Therefore, v/2 = 17000 cm/s
        if pulse_duration >=0.01746:
            #print('time out')
            continue
        elif distance > 3 or distance<=0:
            #print('out of range')
            continue
        distance = round(distance, 3)
        #distance = distance -0.34
        #distance = distance -0.037
        #print ('Distance : %f cm'%distance)
        sensor.dist_sendor(distance)
        
        sensor.r.sleep()                    # Takes a reading 15 times per second. That means duration for 1 data point is 66.7 ms.
        
except (KeyboardInterrupt, SystemExit):
    gpio.cleanup()
    sys.exit(0)
except:
    gpio.cleanup()
