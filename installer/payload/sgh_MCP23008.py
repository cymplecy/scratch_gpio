#!/usr/bin/env python
# sgh_PiGlow - control PiGlow LEDS via ScratchGPIO.
#Copyright (C) 2013 by Simon Walters based on code from Pimoroni

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

Version =  '0.0.3' # 1Jan13


#try and inport smbus but don't worry if not installed
import subprocess
try:
    import smbus
except:
    pass
    


class sgh_MCP23008:

    def __init__(self, address,i2c_bus=1,debug=False):

        print "MCP init"
        self.address = address
        self.i2c_bus = i2c_bus
        self.debug = debug
        print "i2cbus:",self.i2c_bus
        #self.bus = smbus.SMBus(i2c_bus)
        self.bus = smbus.SMBus(i2c_bus)
        print "self.bus:", self.bus
        print "complete"
        
    def errMsg(self):
        print "Error accessing device at address 0x%02X" % self.address
        raise Exception("Error accesing I2C Device") # Added by Simon Walters to raise an error if no device found
        return -1


    def readU8(self, reg):
        "Read an unsigned byte from the I2C device"
        try:
            result = self.bus.read_byte_data(self.address, reg)
            if self.debug:
                print ("I2C: Device 0x%02X returned 0x%02X from reg 0x%02X" % (self.address, result & 0xFF, reg))
            return result
        except IOError, err:
            return self.errMsg()
#### end PiGlow ###############################################################



