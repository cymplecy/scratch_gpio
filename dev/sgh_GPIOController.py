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
import time

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
        self.numOfPins = 27 #there are actually 26 but python can't count properly :)
        self.PINPUT = 4
        self.POUTPUT = 1
        self.PPWM = 2
        self.PUNUSED = 8
        self.SONAR = 16
        self.ULTRA = 32

        self.INVERT = False

        self.PWMFREQ = 100
        #self.PIN_NUM = [11,12,13,15,16,18,22, 7, 3, 5,24,26,19,21,23, 8,10]
        #self.PIN_USE = [self.POUTPUT, self.POUTPUT, self.POUTPUT, self.POUTPUT, self.POUTPUT, self.POUTPUT, self.PINPUT, self.PINPUT, self.PUNUSED, self.PUNUSED, self.PINPUT, self.PINPUT, self.PINPUT, self.PINPUT, self.PINPUT, self.PINPUT, self.PINPUT]

        #self.PINS = len(self.PIN_NUM)
        #self.PIN_NUM_LOOKUP=[int] * 27

        #for i in range(self.PINS):
            #self.PIN_NUM_LOOKUP[self.PIN_NUM[i]] = i
            #print i, PIN_NUM[i
            
        self.pinUse = [self.PUNUSED] * self.numOfPins
        
        self.pwmRef = [None] * self.numOfPins
        
        #self.ULTRA_IN_USE = [False] * self.PINS
        #self.ultraTotalInUse = 0
        #self.ultraSleep = 1.0
        self.debug = debug
        if self.debug:
            print "Debug enabled"
        # End init
    
    #Procedure to set pin mode for each pin
    def setPinMode(self):
        for pin in range(self.numOfPins):
            print pin
            if (self.pinUse[pin] == self.POUTPUT):
                print 'setting pin' , pin , ' to out' 
                GPIO.setup(pin,GPIO.OUT)
            elif (self.pinUse[pin] == self.PINPUT):
                print 'setting pin' , pin , ' to in' 
                GPIO.setup(pin,GPIO.IN,pull_up_down=GPIO.PUD_UP)

    def pinUpdate(self, pin, value,type = 'plain'):
        if type == "pwm": # 
            if self.pinUse[pin] == self.PPWM: # if already active as PWM 
                self.pwmRef[pin].ChangeDutyCycle(max(0,min(100,abs(value)))) # just update PWM value
            else:
                self.pinUse[pin] = self.PPWM # set pin use as PWM
                if self.pwmRef[pin] == None: #if not used previously used for PWM then 
                    self.pwmRef[pin] = GPIO.PWM(pin,self.PWMFREQ) # create new PWM instance 
                self.pwmRef[pin].start(max(0,min(100,abs(value)))) # update PWM value
        else:
            if self.INVERT == True: # Invert data value (useful for 7 segment common anode displays)
                if self.pinUse[pin] == self.POUTPUT:
                    value = abs(value - 1)
            if (self.pinUse[pin] == self.POUTPUT): # if already an output
                GPIO.output(pin, value) # set output to 1 ot 0
            elif (self.pinUse[pin] == self.PINPUT): # if pin is an input
                self.pinUse[pin] = self.POUTPUT # switch it to output
                GPIO.setup(pin,GPIO.OUT)
                GPIO.output(pin, value) # set output to 1 ot 0
                print 'pin' , pin , ' changed to digital out from input' 
            elif (self.pinUse[pin] == self.PPWM): #if pin in use for PWM
                self.pinUse[pin] = self.POUTPUT # switch it to output
                self.pwmRef[pin].stop() # stop PWM from running
                GPIO.setup(pin,GPIO.OUT)
                GPIO.output(pin, value) # set output to 1 ot 0
                print 'pin' , pin , ' changed to digital out from PWM' 

    def pinSonar(self, pin):
        self.pinUse[pin] = self.SONAR
        GPIO.setup(pin,GPIO.OUT)
        ti = time.time()
        # setup a list to hold 3 values and then do 3 distance calcs and store them
        #print 'sonar started'
        distlist = [0.0,0.0,0.0]
        ts=time.time()
        print
        for k in range(3):
            #print "sonar pulse" , k
            GPIO.output(pin, 1)    # Send Pulse high
            time.sleep(0.00001)     #  wait
            GPIO.output(pin, 0)  #  bring it back low - pulse over.
            t0=time.time() # remember current time
            GPIO.setup(pin,GPIO.IN)
            #PIN_USE[i] = PINPUT don't bother telling system
            
            t1=t0
            # This while loop waits for input pin (7) to be low but with a 0.04sec timeout 
            while ((GPIO.input(pin)==0) and ((t1-t0) < 0.02)):
                #time.sleep(0.00001)
                t1=time.time()
            t1=time.time()
            #print 'low' , (t1-t0).microseconds
            t2=t1
            #  This while loops waits for input pin to go high to indicate pulse detection
            #  with 0.04 sec timeout
            while ((GPIO.input(pin)==1) and ((t2-t1) < 0.02)):
                #time.sleep(0.00001)
                t2=time.time()
            t2=time.time()
            #print 'high' , (t2-t1).microseconds
            t3=(t2-t1)  # t2 contains time taken for pulse to return
            #print "total time " , t3
            distance=t3*343/2*100  # calc distance in cm
            distlist[k]=distance
            #print distance
            GPIO.setup(pin,GPIO.OUT)
        tf = time.time() - ts
        distance = sorted(distlist)[1] # sort the list and pick middle value as best distance
        
        #print "total time " , tf
        #for k in range(5):
            #print distlist[k]
        #print "pulse time" , distance*58
        #print "total time in microsecs" , (tf-ti).microseconds                    
        # only update Scratch values if distance is < 500cm
        if (distance > 280):
            distance = 299
        if (distance < 2):
            distance = 1

        return distance
        
    def pinRead(self, pin):
        return GPIO.input(pin)

#### End of main program