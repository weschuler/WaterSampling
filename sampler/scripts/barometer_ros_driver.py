#!/usr/bin/python3
import os
import sys

# A convoluted way to get to the ms5837-python directory
cwd = os.getcwd()
cwd_path = cwd.split('/')

path_to_ms5837 = ''
for i in range(len(cwd_path) - 2):
    path_to_ms5837 += cwd_path[i] + '/'
path_to_ms5837 += 'ms5837-python/'

sys.path.append(path_to_ms5837)

import ms5837

import rospy as rp

class SensorNode():
    """
    A ROS node for the ms5837 pressure and temperature sensor

    Based on and using Blue Robotics python driver found at:
    https://github.com/bluerobotics/ms5837-python

    """

    def __init__(self, rate):
        rp.init_node("depth_temp_sensor_node")
        self.rate = rate

        # Sensor instance
        self.sensor = ms5837.MS5837_30BA()

        # Check that sensor works
        if not self.sensor.init():
            rp.logerr("MS5837 sensor could not be initialized")
            exit(1)

        if not self.sensor.read():
            rp.logerr("MS5837 sensor read failed")
            exit(1)

        rp.spin()

    def publish_sensor_data(self,):
        r = rp.Rate(self.rate)
        while not rp.is_shutdown():
            self.sensor.read()

            r.sleep()
