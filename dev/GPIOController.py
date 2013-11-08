#!/usr/bin/env python
# GPIOController - control Raspberry Pi GPIO ports 
#Copyright (C) 2013 by Simon Walters 

#This program is free software; you can redistribute it and/or
#modify it under the terms of the GNU General Public License
#as published by the Free Software Foundation; either version 2
#of the License, or (at your option) any later version.

#This program is distributed in the hope that it will be useful,
#but WITHOUT ANY WARRANTY; without even the implied warranty of
#MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#GNU General Public License for more details.

#You should have received a copy of the GNU General Public License
#along with this program; if not, write to the Free Software
#Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.

Version =  '0.0.1' # 07Nov13



import threading
import socket
import time
import sys
import struct
import datetime as dt
import shlex
import os
import math

import RPi.GPIO as GPIO

class sghGPIOController :

  @staticmethod
  def getPiRevision():
    "Gets the version number of the Raspberry Pi board"
    return GPIO.RPI_REVISION

  def __init__(self, debug=False):
    GPIO.setmode(GPIO.BOARD)
    GPIO.setwarnings(False)
    GPIO.cleanup()
    print "Board Revision" , GPIO.RPI_REVISION

    #Set some constants and initialise lists
    self.PINPUT = 4
    self.POUTPUT = 1
    self.PPWM = 2
    self.PUNUSED = 8

    self.INVERT = False
    self.PIN_NUM = [11,12,13,15,16,18,22, 7, 3, 5,24,26,19,21,23, 8,10]
    self.PIN_USE = [ self.POUTPUT, self.POUTPUT, self.POUTPUT, self.POUTPUT, self.POUTPUT, Pself.OUTPUT, self.PINPUT, self.PINPUT, self.PUNUSED, self.PUNUSED, self.PINPUT, self.PINPUT, self.PINPUT, self.PINPUT, self.PINPUT, self.PINPUT, self.PINPUT]
        

    self.PINS = len(self.PIN_NUM)
    self.PIN_NUM_LOOKUP=[int] * 27

    for i in range(self.PINS):
        self.PIN_NUM_LOOKUP[self.PIN_NUM[i]] = i
        #print i, PIN_NUM[i]


    self.PWM_OUT = [None] * PINS
    self.ULTRA_IN_USE = [False] * PINS
    self.ultraTotalInUse = 0
    self.ultraSleep = 1.0
    self.debug = debug
    # End init
    
#Procedure to set pin mode for each pin
def SetPinMode():
    for i in range(PINS):
        if (PIN_USE[i] == POUTPUT):
            print 'setting pin' , PIN_NUM[i] , ' to out'
            GPIO.setup(PIN_NUM[i],GPIO.OUT)
        elif (PIN_USE[i] == PINPUT):
            print 'setting pin' , PIN_NUM[i] , ' to in'
            GPIO.setup(PIN_NUM[i],GPIO.IN,pull_up_down=GPIO.PUD_UP)

        PIN_NUM_LOOKUP[PIN_NUM[i]] = i
        


class MyError(Exception):
    def __init__(self, value):
        self.value = value

    def __str__(self):
        return repr(self.value)



def index_pin_update(self, pin_index, value):
    if INVERT == True:
        if PIN_USE[pin_index] == POUTPUT:
            value = abs(value - 1)
    if (PIN_USE[pin_index] == PINPUT):
        PIN_USE[pin_index] = POUTPUT
        GPIO.setup(PIN_NUM[pin_index],GPIO.OUT)
        print 'pin' , PIN_NUM[pin_index] , ' changed to digital out from input'
    if (PIN_USE[pin_index] == PPWM):
        PIN_USE[pin_index] = POUTPUT
        PWM_OUT[pin_index].stop()
        GPIO.setup(PIN_NUM[pin_index],GPIO.OUT)
        print 'pin' , PIN_NUM[pin_index] , ' changed to digital out from PWM'
    if (PIN_USE[pin_index] == POUTPUT):
        #print 'setting gpio %d (physical pin %d) to %d' % (GPIO_NUM[pin_index],PIN_NUM[pin_index],value)
        GPIO.output(PIN_NUM[pin_index], value)
            
def index_pwm_update(self, pin_index, value):
    #print "pwm changed on pin index" , pin_index, "to", value
    #print "Actualy pin=" , PIN_NUM[pin_index]
    if PIN_USE[pin_index] != PPWM:
        PIN_USE[pin_index] = PPWM
        GPIO.PWM(PIN_NUM[pin_index],100)
        PWM_OUT[pin_index] = GPIO.PWM(PIN_NUM[pin_index],100)
        PWM_OUT[pin_index].start(max(0,min(100,abs(value))))
    else:
        PWM_OUT[pin_index].ChangeDutyCycle(max(0,min(100,abs(value))))
            




SetPinMode()



    except KeyboardInterrupt:
        #cleanup_threads((listener,sender))
        GPIO.cleanup()
        sys.exit()
        print "CleanUp complete"
        
#### End of main program

        
class Adafruit_I2C :

  @staticmethod
  def getPiRevision():
    "Gets the version number of the Raspberry Pi board"
    # Courtesy quick2wire-python-api
    # https://github.com/quick2wire/quick2wire-python-api
    try:
      with open('/proc/cpuinfo','r') as f:
        for line in f:
          if line.startswith('Revision'):
            return 1 if line.rstrip()[-1] in ['1','2'] else 2
    except:
      return 0

  @staticmethod
  def getPiI2CBusNumber():
    # Gets the I2C bus number /dev/i2c#
    return 1 if Adafruit_I2C.getPiRevision() > 1 else 0
 
  def __init__(self, address, busnum=-1, debug=False):
    self.address = address
    # By default, the correct I2C bus is auto-detected using /proc/cpuinfo
    # Alternatively, you can hard-code the bus version below:
    # self.bus = smbus.SMBus(0); # Force I2C0 (early 256MB Pi's)
    # self.bus = smbus.SMBus(1); # Force I2C1 (512MB Pi's)
    self.bus = smbus.SMBus(
      busnum if busnum >= 0 else Adafruit_I2C.getPiI2CBusNumber())
    self.debug = debug

  def reverseByteOrder(self, data):
    "Reverses the byte order of an int (16-bit) or long (32-bit) value"
    # Courtesy Vishal Sapre
    byteCount = len(hex(data)[2:].replace('L','')[::2])
    val       = 0
    for i in range(byteCount):
      val    = (val << 8) | (data & 0xff)
      data >>= 8
    return val

  def errMsg(self):
    print "Error accessing 0x%02X: Check your I2C address" % self.address
    raise Exception("Error accesing I2C Device") # Added by Simon Walters to raise an error if no device found
    return -1

  def write8(self, reg, value):
    "Writes an 8-bit value to the specified register/address"
    try:
      self.bus.write_byte_data(self.address, reg, value)
      if self.debug:
        print "I2C: Wrote 0x%02X to register 0x%02X" % (value, reg)
    except IOError, err:
      return self.errMsg()

  def write16(self, reg, value):
    "Writes a 16-bit value to the specified register/address pair"
    try:
      self.bus.write_word_data(self.address, reg, value)
      if self.debug:
        print ("I2C: Wrote 0x%02X to register pair 0x%02X,0x%02X" %
         (value, reg, reg+1))
    except IOError, err:
      return self.errMsg()

  def writeList(self, reg, list):
    "Writes an array of bytes using I2C format"
    try:
      if self.debug:
        print "I2C: Writing list to register 0x%02X:" % reg
        print list
      self.bus.write_i2c_block_data(self.address, reg, list)
    except IOError, err:
      return self.errMsg()

  def readList(self, reg, length):
    "Read a list of bytes from the I2C device"
    try:
      results = self.bus.read_i2c_block_data(self.address, reg, length)
      if self.debug:
        print ("I2C: Device 0x%02X returned the following from reg 0x%02X" %
         (self.address, reg))
        print results
      return results
    except IOError, err:
      return self.errMsg()

  def readU8(self, reg):
    "Read an unsigned byte from the I2C device"
    try:
      result = self.bus.read_byte_data(self.address, reg)
      if self.debug:
        print ("I2C: Device 0x%02X returned 0x%02X from reg 0x%02X" %
         (self.address, result & 0xFF, reg))
      return result
    except IOError, err:
      return self.errMsg()

  def readS8(self, reg):
    "Reads a signed byte from the I2C device"
    try:
      result = self.bus.read_byte_data(self.address, reg)
      if result > 127: result -= 256
      if self.debug:
        print ("I2C: Device 0x%02X returned 0x%02X from reg 0x%02X" %
         (self.address, result & 0xFF, reg))
      return result
    except IOError, err:
      return self.errMsg()

  def readU16(self, reg):
    "Reads an unsigned 16-bit value from the I2C device"
    try:
      result = self.bus.read_word_data(self.address,reg)
      if (self.debug):
        print "I2C: Device 0x%02X returned 0x%04X from reg 0x%02X" % (self.address, result & 0xFFFF, reg)
      return result
    except IOError, err:
      return self.errMsg()

  def readS16(self, reg):
    "Reads a signed 16-bit value from the I2C device"
    try:
      result = self.bus.read_word_data(self.address,reg)
      if (self.debug):
        print "I2C: Device 0x%02X returned 0x%04X from reg 0x%02X" % (self.address, result & 0xFFFF, reg)
      return result
    except IOError, err:
      return self.errMsg()

if __name__ == '__main__':
  try:
    bus = Adafruit_I2C(address=0)
    print "Default I2C bus is accessible"
  except:
    print "Error accessing default I2C bus"
