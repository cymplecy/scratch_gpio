#!/usr/bin/env python
# sgh_GPIOController - control Raspberry Pi GPIO ports using RPi.GPIO by Ben Crosten
#                      and servod by Richard Hirst
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

Version =  '0.1.0' # 12Nov13

import RPi.GPIO as GPIO
import time
import os
import datetime as dt
import threading


def getPiRevision():
    "Gets the version number of the Raspberry Pi board"
    return GPIO.RPI_REVISION

def cleanup(self):
    GPIO.cleanup()

GPIO.setmode(GPIO.BOARD)
GPIO.setwarnings(False)
GPIO.cleanup()

GPIO.setup(11, GPIO.IN,pull_up_down=GPIO.PUD_DOWN)
GPIO.setup(12, GPIO.IN,pull_up_down=GPIO.PUD_DOWN)
GPIO.setup(13, GPIO.IN,pull_up_down=GPIO.PUD_DOWN)

#while GPIO.input(11) == GPIO.HIGH:
#    time.sleep(0.01)  # wait 10 ms to give CPU chance to do other things
while 1:
    print
    print GPIO.input(11) 
    print GPIO.input(12)
    print GPIO.input(13)
    time.sleep(0.2)  # wait 10 ms to give CPU chance to do other things
    
print "finished"


