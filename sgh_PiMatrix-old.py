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

Version =  '0.0.1' # 23Jan14


#try and inport smbus but don't worry if not installed
import subprocess
import math
import time
try:
    import smbus
except:
    pass
    
STATE_PORTA = 0x00
STATE_PORTB = 0x01
SET_PORTA = 0x12
SET_PORTB = 0x13

class sgh_PiMatrix:

    bus = None

    def __init__(self,address, i2c_bus=1):
        print "PiMatrix init"
        self.i2c_bus = i2c_bus
        print "i2cbus:",self.i2c_bus
        #self.bus = smbus.SMBus(i2c_bus)
        self.bus = smbus.SMBus(i2c_bus)
        self.i2c_addr = address
        print "PiMatrix self.bus:", self.bus
        self.enable_output()
        self.aval = 0x00
        self.bval = 0xFF
        
        print "PiMatrix intialised"

    def enable_output(self):
        self.write_i2c(STATE_PORTA, 0x00)
        self.write_i2c(STATE_PORTB, 0x00)

    def update_porta(self, value):
        self.write_i2c(SET_PORTA , value)

    def update_portb(self, value):
        self.write_i2c(SET_PORTB , value)


    def write_i2c(self, reg_addr, value):
        if not isinstance(value, list):
            value = [value];
        try:
            self.bus.write_i2c_block_data(self.i2c_addr, reg_addr, value)
        except IOError:
            subprocess.call(['i2cdetect', '-y', '0'])
            self.bus.write_i2c_block_data(self.i2c_addr, reg_addr, value)

    def setPixel(self,x,y):
        print "X and Y",x,y
        xbin = (0x80 >> x)
        ybin = (0x80 >> y)
        print " xbin,ybin" , xbin,ybin
        self.aval =  self.aval | (0x80 >> x)
        self.bval = 0xFF - (0x80 >> y)        
        #self.aval = 0x0F
        print self.aval,self.bval
        print bin(self.aval),bin(self.bval)
        self.aval = 0x0F
        self.update_porta(self.aval)
        self.update_portb(self.bval)
        time.sleep(0.5)
        
    def clear(self):
        print "clear"
        self.update_porta(0)
        self.update_portb(0)
        


