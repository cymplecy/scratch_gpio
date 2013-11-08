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

import RPi.GPIO as GPIO

class sghGPIOController :

  @staticmethod
  def getPiRevision():
    "Gets the version number of the Raspberry Pi board"
    return GPIO.RPI_REVISION
    
    def cleanup(self):
        GPIO.cleanup()

  def __init__(self, debug=False):
    GPIO.setmode(GPIO.BOARD)
    GPIO.setwarnings(False)
    GPIO.cleanup()
    #print "Board Revision" , GPIO.RPI_REVISION

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
    if self.debug:
        print "Debug enabled"
    # End init
    
    #Procedure to set pin mode for each pin
    def SetPinMode(self):
        for i in range(self.PINS):
            if (self.PIN_USE[i] == self.POUTPUT):
                print 'setting pin' , self.PIN_NUM[i] , ' to out'
                GPIO.setup(self.PIN_NUM[i],GPIO.OUT)
            elif (self.PIN_USE[i] == self.PINPUT):
                print 'setting pin' , self.PIN_NUM[i] , ' to in'
                GPIO.setup(self.PIN_NUM[i],GPIO.IN,pull_up_down=GPIO.PUD_UP)

            self.PIN_NUM_LOOKUP[self.PIN_NUM[i]] = i

    def index_pin_update(self, pin_index, value):
        if self.INVERT == True:
            if self.PIN_USE[pin_index] == self.POUTPUT:
                value = abs(value - 1)
        if (self.PIN_USE[pin_index] == self.PINPUT):
            self.PIN_USE[pin_index] = self.POUTPUT
            GPIO.setup(self.PIN_NUM[pin_index],GPIO.OUT)
            print 'pin' , self.PIN_NUM[pin_index] , ' changed to digital out from input'
        if (self.PIN_USE[pin_index] == self.PPWM):
            self.PIN_USE[pin_index] = self.POUTPUT
            self.PWM_OUT[pin_index].stop()
            GPIO.setup(self.PIN_NUM[pin_index],GPIO.OUT)
            print 'pin' , self.PIN_NUM[pin_index] , ' changed to digital out from PWM'
        if (self.PIN_USE[pin_index] == self.POUTPUT):
            #print 'setting gpio %d (physical pin %d) to %d' % (GPIO_NUM[pin_index],PIN_NUM[pin_index],value)
            GPIO.output(self.PIN_NUM[pin_index], value)
                
    def index_pwm_update(self, pin_index, value):
        #print "pwm changed on pin index" , pin_index, "to", value
        #print "Actualy pin=" , PIN_NUM[pin_index]
        if self.PIN_USE[pin_index] != self.PPWM:
            self.PIN_USE[pin_index] = self.PPWM
            GPIO.PWM(self.PIN_NUM[pin_index],100)
            self.PWM_OUT[pin_index] = GPIO.PWM(self.PIN_NUM[pin_index],100)
            self.PWM_OUT[pin_index].start(max(0,min(100,abs(value))))
        else:
            self.PWM_OUT[pin_index].ChangeDutyCycle(max(0,min(100,abs(value))))
            
     
#### End of main program

        

