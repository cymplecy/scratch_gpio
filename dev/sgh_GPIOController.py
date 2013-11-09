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

class GPIOController :

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
        self.nunOfPins = 27
        self.PINPUT = 4
        self.POUTPUT = 1
        self.PPWM = 2
        self.PUNUSED = 8

        self.INVERT = False

        self.PWMFREQ = 100
        #self.PIN_NUM = [11,12,13,15,16,18,22, 7, 3, 5,24,26,19,21,23, 8,10]
        #self.PIN_USE = [self.POUTPUT, self.POUTPUT, self.POUTPUT, self.POUTPUT, self.POUTPUT, self.POUTPUT, self.PINPUT, self.PINPUT, self.PUNUSED, self.PUNUSED, self.PINPUT, self.PINPUT, self.PINPUT, self.PINPUT, self.PINPUT, self.PINPUT, self.PINPUT]

        #self.PINS = len(self.PIN_NUM)
        #self.PIN_NUM_LOOKUP=[int] * 27

        #for i in range(self.PINS):
            #self.PIN_NUM_LOOKUP[self.PIN_NUM[i]] = i
            #print i, PIN_NUM[i
            
        self.pinUse = [self.PUNUSED] * self.nunOfPins
        
        self.pwmRef = [None] * self.nunOfPins
        
        #self.ULTRA_IN_USE = [False] * self.PINS
        self.ultraTotalInUse = 0
        self.ultraSleep = 1.0
        self.debug = debug
        if self.debug:
            print "Debug enabled"
        # End init
    
    #Procedure to set pin mode for each pin
    def setPinMode(self):
        for pin in range(self.nunOfPins):
            print pin
            if (self.pinUse[pin] == self.POUTPUT):
                print 'setting pin' , pin , ' to out' 
                GPIO.setup(pin,GPIO.OUT)
            elif (self.pinUse[pin] == self.PINPUT):
                print 'setting pin' , pin , ' to in' 
                GPIO.setup(pin,GPIO.IN,pull_up_down=GPIO.PUD_UP)

    def pin_update(self, pin, value):
        if self.INVERT == True:
            if self.pinUse[pin] == self.POUTPUT:
                value = abs(value - 1)
        if (self.pinUse[pin] == self.PINPUT):
            self.pinUse[pin] = self.POUTPUT
            GPIO.setup(pin,GPIO.OUT)
            print 'pin' , pin , ' changed to digital out from input' 
        if (self.pinUse[pin] == self.PPWM):
            self.pinUse[pin] = self.POUTPUT
            self.pwmRef[pin].stop()
            GPIO.setup(pin,GPIO.OUT)
            print 'pin' , pin , ' changed to digital out from PWM' 
        if (self.pinUse[pin] == self.POUTPUT):
            #print 'setting gpio %d (physical pin %d) to %d' % (GPIO_NUM[pin_index],PIN_NUM[pin_index],value) 
            GPIO.output(pin, value)
                            
    def pwm_update(self, pin, value):
        if self.pinUse[pin] != self.PPWM:
            self.pinUse[pin] = self.PPWM
            if self.pwmRef[pin] == None:
                self.pwmRef[pin] = GPIO.PWM(pin,PWMFREQ)
            self.pwmRef[pin].start(max(0,min(100,abs(value))))
        else:
            self.pwmRef[pin].ChangeDutyCycle(max(0,min(100,abs(value))))
            

     
#### End of main program

        

