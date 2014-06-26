##########################################
## Python module to read a Wii nunchuck ##
##                                      ##
## Written by Jason - @Boeeerb          ##
##  jase@boeeerb.co.uk                  ##
##########################################
##
## v0.1 03/05/14 - Initital release
## v0.2 21/06/14 - Retrieve one byte at a time [Simon Walters - @cymplecy]
## v0.3 22/06/14 - Minor Refactoring [Jack Wearden - @JackWeirdy]
## v0.32 25/6/14 - XOR each data byte with 0x17 and then add 0x17 to produce corrent values - Simon Walters @cymplecy
## v0.4 26/6/14 - Change method of XOR and add delay parameter - Simon Walters @cymplecy

from smbus import SMBus
import RPi.GPIO as rpi
import time as time

bus = 0

class nunchuck:

  def __init__(self,delay = 0.05):
    self.delay = delay
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
    time.sleep(self.delay)
    temp = [(0x17 + (0x17 ^ self.bus.read_byte(0x52))) for i in range(6)]
    return temp

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

    return butc == 0

  def button_z(self):
    data = self.read()
    butc = (data[5] & 0x01)

    return butc == 0

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
    
  def setdelay(self,delay):
    self.delay = delay


  def scale(self,value,_min,_max,_omin,_omax):
    return (value - _min) * (_omax - _omin) // (_max - _min) + _omin

