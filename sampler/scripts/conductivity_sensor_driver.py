import os
import rospy as rp


cwd = rp.get_param('/conductivity_sensor_node/source_directory')
cwd_path = cwd.split('/')
path_to_atlasi2c = ''
for i in range(len(cwd_path) - 1):
  path_to_atlasi2c += cwd_path[i] + '/'
path_to_atlasi2c += 'Raspberry-Pi-sample-code'

print(path_to_atlasi2c)

import sys
sys.path.append(path_to_atlasi2c)

from AtlasI2C import AtlasI2C
from i2c import get_devices

import time

class ConductivitySensor():
  """
  A simple driver for the Atlas Scientific conductivity sensor

  Based on and using Atlas Scientific python driver found at:
  https://github.com/AtlasScientific/Raspberry-Pi-sample-code

  """

  def __init__(self,):
    # Initialize sensor
    device_list = get_devices()
    self.sensor = device_list[0]

    self.EC_meas = 0.0
    self.TDS_meas = 0
    self.S_meas = 0.0

    # Enable Conductivity, total dissolved solids and salinity measurements
    self.setConductivity()
    self.setDissolvedSolids()
    self.setSalinity()

  def setConductivity(self, flag=True):
    if flag:
      self.sensor.query('O,EC,1')
    else:
      self.sensor.query('O,EC,0')

  def setDissolvedSolids(self, flag=True):
    if flag:
      self.sensor.query('O,TDS,1')
    else:
      self.sensor.query('O,TDS,0')

  def setSalinity(self, flag=True):
    if flag:
      self.sensor.query('O,S,1')
    else:
      self.sensor.query('O,S,0')
  
  def conductivity(self,):
    return self.EC_meas
  
  def totalDissolvedSolids(self,):
    return self.TDS_meas

  def salinity(self,):
    return self.S_meas

  def read(self,):
    self.sensor.write('R')
    time.sleep(self.sensor.long_timeout)
    response = self.sensor.read()
    response_parts = response.split()

    if response_parts[0] == 'Success':
      values = response_parts[-1].split(',')
      self.EC_meas = float(values[0])
      self.TDS_meas = int(values[1])
      self.S_meas = float(values[2].rstrip('\x00'))

      return True
    elif response_parts[0] == 'Fail':
      return False
