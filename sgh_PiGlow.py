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

Version =  '0.0.2' # 25Nov13


#try and inport smbus but don't worry if not installed
try:
    from smbus import SMBus
except:
    pass
    
CMD_ENABLE_OUTPUT = 0x00
CMD_ENABLE_LEDS = 0x13
CMD_SET_PWM_VALUES = 0x01
CMD_UPDATE = 0x16

class PiGlow:
    i2c_addr = 0x54 # fixed i2c address of SN3218 ic
    bus = None

    def __init__(self, i2c_bus=1):
        print "PiGlow init"
        print i2c_bus
        #self.bus = smbus.SMBus(i2c_bus)
        self.bus = SMBus(i2c_bus)
        self.enable_output()
        self.enable_leds()
        print "complete"

    def enable_output(self):
        self.write_i2c(CMD_ENABLE_OUTPUT, 0x01)

    def enable_leds(self):
        self.write_i2c(CMD_ENABLE_LEDS, [0xFF, 0xFF, 0xFF])

    def update_pwm_values(self, values=[0] * 18):
        #print "update pwm"
        self.write_i2c(CMD_SET_PWM_VALUES, values)
        self.write_i2c(CMD_UPDATE, 0xFF)

    def write_i2c(self, reg_addr, value):
        if not isinstance(value, list):
            value = [value];
        self.bus.write_i2c_block_data(self.i2c_addr, reg_addr, value)
#### end PiGlow ###############################################################



