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

BIG_NUM = 2123456789

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
        self.piRevision = self.getPiRevision()
        print "Board Revision" , self.piRevision

        #Set some constants and initialise lists
        self.numOfPins = 27 #there are actually 26 but python can't count properly :)
        self.PINPUT = 4
        self.POUTPUT = 1
        self.PPWM = 2
        self.PUNUSED = 8
        self.PSONAR = 16
        self.PULTRA = 32
        self.PSERVOD = 64
        self.PSTEPPER = 128
        self.PCOUNT = 256
        self.PINPUTDOWN = 512

        self.INVERT = False

        self.PWMFREQ = 100

        self.pinUse = [self.PUNUSED] * self.numOfPins
        self.servodPins = None
        
        self.pinRef = [None] * self.numOfPins
        self.pinCount = [0] * self.numOfPins
        self.gpioLookup = [0] * self.numOfPins
        if self.piRevision == 1:
        #                       0  1  2  3  4  5  6  7  8  9 10 11 12 13 14 15 16 17 18 19 20 21 22 23 24 25 26
        #                     [ 3, 5,99,99, 7,99,99,26,24,21,19,23,99,99, 8,10,99,11,12,99,99,13,15,16,18,22,99]
            self.gpioLookup = [99,99,99, 0,99, 1,99, 4,14,99,15,17,18,21,99,22,23,99,24,10,99, 9,25,11, 8,99, 7]
        else:                #[99,99, 2,99, 7, 3,99,26,24,21,19,23,99,99, 8,10,99,11,12,99,99,13,15,16,18,22,99]
            self.gpioLookup = [99,99,99, 2,99, 3,99, 4,14,99,15,17,18,27,99,22,23,99,24,10,99, 9,25,11, 8,99, 7]
        
        
        #self.ULTRA_IN_USE = [False] * self.PINS
        #self.ultraTotalInUse = 0
        #self.ultraSleep = 1.0
        self.debug = debug
        if self.debug:
            print "Debug enabled"
        # End init
    
    def my_callback(self,pin):
        self.pinCount[pin] +=1
        #print('Edge detected on channel',channel,self.encoderCount) 
        

    #Procedure to set pin mode for each pin
    def setPinMode(self):
        for pin in range(self.numOfPins):
            #print pin
            if (self.pinUse[pin] == self.POUTPUT):
                print 'setting pin' , pin , ' to out' 
                GPIO.setup(pin,GPIO.OUT)
            elif (self.pinUse[pin] == self.PINPUT):
                print 'setting pin' , pin , ' to in with pull up' 
                GPIO.setup(pin,GPIO.IN,pull_up_down=GPIO.PUD_UP)
            elif (self.pinUse[pin] == self.PINPUTDOWN):
                print 'setting pin' , pin , ' to in with pull down' 
                GPIO.setup(pin,GPIO.IN,pull_up_down=GPIO.PUD_DOWN)
                self.pinUse[pin] = self.PINPUT
            elif (self.pinUse[pin] == self.PCOUNT):
                print 'setting pin' , pin , ' to count' 
                GPIO.setup(pin,GPIO.IN,pull_up_down=GPIO.PUD_DOWN)
                GPIO.add_event_detect(pin, GPIO.RISING, callback=self.my_callback)  # add rising edge detection on a channel

                
    def pinUpdate(self, pin, value,type = 'plain',stepDelay = 0.003):
        try:
            #print pin,value,type,self.pinUse[pin]
            if type == "pwm": # 
                #return
                if self.INVERT == True: # Invert data value (needed for active low devices)
                    value = 100 - abs(value)
                try: 
                    self.pinRef[pin].ChangeDutyCycle(max(0,min(100,abs(value)))) # just update PWM value
                except:
                    try:
                        print ("Stopping previous instance")
                        sghGC.pinRef[pin].stop()
                    except:
                        pass
                    GPIO.setup(pin,GPIO.OUT)
                    self.pinRef[pin] = GPIO.PWM(pin,self.PWMFREQ) # create new PWM instance
                    print "type of pwm:" ,self.pinRef[pin]
                    self.pinRef[pin].start(max(0,min(100,abs(value)))) # update PWM value
                    self.pinUse[pin] = self.PPWM # set pin use as PWM
                    print 'pin' , pin , ' changed to PWM' 
                    print ("pin",pin, "set to", value)              
            elif type == "plain":
                if self.INVERT == True: # Invert data value (useful for 7 segment common anode displays)
                    if self.pinUse[pin] == self.POUTPUT:
                        value = abs(value - 1)
                if (self.pinUse[pin] == self.POUTPUT): # if already an output
                    GPIO.output(pin, int(value)) # set output to 1 ot 0
                    #print ("pin",pin, "set to", value)
                elif (self.pinUse[pin] == self.PINPUT): # if pin is an input
                    self.pinUse[pin] = self.POUTPUT # switch it to output
                    GPIO.setup(pin,GPIO.OUT)
                    GPIO.output(pin, int(value)) # set output to 1 ot 0
                    print 'pin' , pin , ' changed to digital out from input' 
                    print ("pin",pin, "set to", value)
                elif (self.pinUse[pin] == self.PPWM): #if pin in use for PWM
                    self.pinUse[pin] = self.POUTPUT # switch it to output
                    self.pinRef[pin].stop() # stop PWM from running
                    self.pinRef[pin] = None
                    GPIO.setup(pin,GPIO.OUT)
                    GPIO.output(pin, int(value)) # set output to 1 or 0
                    print 'pin' , pin , ' changed to digital out from PWM' 
                    print ("pin",pin, "set to", value)
                elif (self.pinUse[pin] == self.PUNUSED): # if pin is not allocated
                    self.pinUse[pin] = self.POUTPUT # switch it to output
                    GPIO.setup(pin,GPIO.OUT)
                    GPIO.output(pin, int(value)) # set output to 1 ot 0
                    print 'pin' , pin , ' changed to digital out from unused' 
                    print ("pin",pin, "set to", value)
            #print pin,value,type,self.pinUse[pin]
        except ValueError:
            print "mistake made in trying to update an invalid pin"
            pass
        

    def pinSonar(self, pin):
        self.pinUse[pin] = self.PSONAR
        GPIO.setup(pin,GPIO.OUT)
        ti = time.time()
        # setup a list to hold 3 values and then do 3 distance calcs and store them
        #print 'sonar started'
        distlist = [0.0,0.0,0.0]
        ts=time.time()
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
        #print "pin",pin ,"set to", self.pinUse[pin]
        #print pin ," being read"
        try:
            return GPIO.input(pin)
        except:
            print "Some error reading pin" ,pin
            pass
        
    def startServod(self, pins):
        print ("Starting servod")
        os.system("sudo pkill -f servod")
        for pin in pins:
            self.pinUse[pin] = self.PSERVOD
        os.system('./sgh_servod --idle-timeout=20000 --p1pins="' + str(pins).strip('[]') + '"')
        self.servodPins = pins

    def pinServod(self, pin, value):
        #print ("echo " + str(self.servodPins.index(pin)) + "=" + str(value) + " > /dev/servoblaster")
        os.system("echo " + str(self.servodPins.index(pin)) + "=" + str(value) + " > /dev/servoblaster")
        
    def stopServod(self):
        os.system("sudo pkill -f servod")
        



#### End of main program

