#!/usr/bin/python3
import sys

from watersampling_msgs.msg import ConductivityStamped

import rospy as rp

class SensorNode():
  """
  A ROS node for the  sensor

  """

  def __init__(self, rate):
    rp.init_node("_node")
    self.rate = rate

    # Sensor instance and setup

    # Setup Publisher
    self.sensor_pub = rp.Publisher("/watersampling/conductivity_sensor", ConductivityStamped, queue_size = 1)

    # Start publishing
    self.publish_sensor_data()

  def publish_sensor_data(self,):
    r = rp.Rate(self.rate)
    while not rp.is_shutdown():
      if self.sensor.read():
        sensor_msg = ConductivityStamped()
        sensor_msg.header.stamp = rp.Time.now()
        sensor_msg.conductivity
        sensor_msg.salenity
        sensor_msg.specific_gravity

        self.sensor_pub.publish(sensor_msg)

      else:
        rp.logwarn("Sensor read failed. Will try again")

      r.sleep()

SensorNode(20)