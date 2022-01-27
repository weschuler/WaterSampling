#!/usr/bin/python3
import sys

import rospy as rp

from watersampling_msgs.msg import MS5837Stamped

class SensorNode():
  """
  A ROS node for the ms5837 pressure and temperature sensor

  Based on and using Blue Robotics python driver found at:
  https://github.com/bluerobotics/ms5837-python

  """

  def __init__(self, rate):
    rp.init_node("ms5837_node")
    self.rate = rate

    # A convoluted way to get to the ms5837-python directory
    cwd = rp.get_param('/ms5837_node/source_directory')
    cwd_path = cwd.split('/')
    path_to_ms5837 = ''
    for i in range(len(cwd_path) - 1):
        path_to_ms5837 += cwd_path[i] + '/'
    path_to_ms5837 += 'ms5837-python/'

    sys.path.append(path_to_ms5837)

    import ms5837

    # Sensor instance
    self.sensor = ms5837.MS5837_02BA()
    self.UNITS_Pa = ms5837.UNITS_Pa
    self.UNITS_Centigrade = ms5837.UNITS_Centigrade

    # Set fluid density to freshwater 
    self.sensor.setFluidDensity(ms5837.DENSITY_FRESHWATER)

    # Check that sensor works
    if not self.sensor.init():
      rp.logerr("MS5837 sensor could not be initialized")
      exit(1)

    if not self.sensor.read():
      rp.logerr("MS5837 sensor read failed")
      exit(1)

    # Setup Publisher
    self.sensor_pub = rp.Publisher("/watersampling/depth_sensor", MS5837Stamped, queue_size = 1)

    # Start publishing
    self.publish_sensor_data()

  def publish_sensor_data(self,):
    r = rp.Rate(self.rate)
    while not rp.is_shutdown():
      if self.sensor.read():
        sensor_msg = MS5837Stamped()
        sensor_msg.header.stamp = rp.Time.now()
        sensor_msg.pressure = self.sensor.pressure(self.UNITS_Pa)
        sensor_msg.temperature = self.sensor.temperature(self.UNITS_Centigrade)
        sensor_msg.depth = self.sensor.depth()

        self.sensor_pub.publish(sensor_msg)

      else:
        rp.logwarn("MS5837 sensor read failed. Will try again")

      r.sleep()

SensorNode(20)