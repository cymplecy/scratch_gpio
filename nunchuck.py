##########################################
## Python module to read a Wii nunchuck ##
##                                      ##
## Written by Jason - @Boeeerb          ##
## v0.1 03/05/14 - jase@boeeerb.co.uk   ##
##########################################
##
## v0.1 03/05/14 - Initital release
##
#modiefied 21Jun14 by Simon Walters - reteive one byte at a time

from smbus import SMBus
import RPi.GPIO as rpi
import time as time

bus = 0

class nunchuck:
  
  def __init__(self):
    if rpi.RPI_REVISION == 1:
      i2c_bus = 0
    elif rpi.RPI_REVISION == 2:
      i2c_bus = 1
    else:
      print "Unable to determine Raspberry Pi revision."
      exit
    self.bus = SMBus(i2c_bus)
    self.bus.write_byte_data(0x52,0x40,0x00)
    time.sleep(0.1)
    
  def read(self):
    self.bus.write_byte(0x52,0x00)
    time.sleep(0.2)
    data0 = 0x17 + (self.bus.read_byte(0x52) ^ 0x17)
    data1 = 0x17 + (self.bus.read_byte(0x52) ^ 0x17)
    data2 = 0x17 + (self.bus.read_byte(0x52) ^ 0x17)
    data3 = 0x17 + (self.bus.read_byte(0x52) ^ 0x17)
    data4 = 0x17 + (self.bus.read_byte(0x52) ^ 0x17)
    data5 = 0x17 + (self.bus.read_byte(0x52) ^ 0x17)

    return [data0,data1,data2,data3,data4,data5]

  def raw(self):
    data = self.read()
    return data

  def joystick(self):
    data = self.read()
    return data[0],data[1]

  def accelerometer(self):
    data = self.read()
    return data[2],data[3],data[4]

  def button_c(self):
    data = self.read()
    butc = (data[5] & 0x02)
    if butc == 0:
      return True
    else:
      return False

  def button_z(self):
    data = self.read()
    butc = (data[5] & 0x01)
    if butc == 0:
      return True
    else:
      return False    


  def joystick_x(self):
    data = self.read()
    return data[0]

  def joystick_y(self):
    data = self.read()
    return data[1]

  def accelerometer_x(self):
    data = self.read()
    return data[2]

  def accelerometer_y(self):
    data = self.read()
    return data[3]
  
  def accelerometer_z(self):
    data = self.read()
    return data[4]

  
  def scale(self,value,_min,_max,_omin,_omax):
    return (value - _min) * (_omax - _omin) // (_max - _min) + _omin
