#!/usr/bin/python3
from watersampling_msgs.msg import ConductivityStamped

from conductivity_sensor_driver import ConductivitySensor
import rospy as rp

class ConductivitySensorNode():
  """
  A ROS node for the Atlas Scientific sensor

  """

  def __init__(self):
    rp.init_node("conductivity_sensor_node")

    # Sensor instance and setup
    self.sensor = ConductivitySensor()

    # Setup Publisher
    self.sensor_pub = rp.Publisher("/watersampling/conductivity_sensor", ConductivityStamped, queue_size = 1)

    # Start publishing
    self.publish_sensor_data()

  def publish_sensor_data(self,):
    while not rp.is_shutdown():
      if self.sensor.read():
        sensor_msg = ConductivityStamped()
        sensor_msg.header.stamp = rp.Time.now()
        sensor_msg.conductivity = self.sensor.conductivity()
        sensor_msg.total_dissolved_solids = self.sensor.totalDissolvedSolids()
        sensor_msg.salinity = self.sensor.salinity()

        self.sensor_pub.publish(sensor_msg)

      else:
        rp.logwarn("Sensor read failed. Will try again")

if __name__ == '__main__':
  ConductivitySensorNode()