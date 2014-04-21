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
#Last mod 22 Keep pwm mode for pin once used as pwm

import RPi.GPIO as GPIO
import time
import os
import datetime as dt
import threading

BIG_NUM = 2123456789
SCRIPTPATH = os.path.split(os.path.realpath(__file__))[0]

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
        self.PINPUTNONE = 1024

        self.INVERT = False
        self.ledDim = 100

        self.PWMFREQ = 100
        
        self.dsSensorId  = ""
        self.senderLoopDelay = 0.2
       
       
       

        self.pinUse = [self.PUNUSED] * self.numOfPins
        self.servodPins = None
        
        self.pinRef = [None] * self.numOfPins
        self.pinCount = [0] * self.numOfPins
        self.countDirection = [1] * self.numOfPins
        self.gpioLookup = [0] * self.numOfPins
        self.callbackInUse = [False] * self.numOfPins
        
        self.pinEventEnabled = True
		
        if self.piRevision == 1:
        #                       0  1  2  3  4  5  6  7  8  9 10 11 12 13 14 15 16 17 18 19 20 21 22 23 24 25 26
        #                     [ 3, 5,99,99, 7,99,99,26,24,21,19,23,99,99, 8,10,99,11,12,99,99,13,15,16,18,22,99]
            self.gpioLookup = [99,99,99, 0,99, 1,99, 4,14,99,15,17,18,21,99,22,23,99,24,10,99, 9,25,11, 8,99, 7]
        else:                #[99,99, 2,99, 7, 3,99,26,24,21,19,23,99,99, 8,10,99,11,12,99,99,13,15,16,18,22,99]
            self.gpioLookup = [99,99,99, 2,99, 3,99, 4,14,99,15,17,18,27,99,22,23,99,24,10,99, 9,25,11, 8,99, 7]
            
        self.validPins = [3,5,7,8,10,11,12,13,15,16,18,19,21,22,23,24,26]
        
        
        #self.ULTRA_IN_USE = [False] * self.PINS
        #self.ultraTotalInUse = 0
        #self.ultraSleep = 1.0
        self.debug = debug
        if self.debug:
            print "Debug enabled"
        # End init
    
    def my_callback(self,pin):
        self.pinCount[pin] += (self.countDirection[pin] * 1) # inc or dec count based on direction required
        #print('Edge detected on channel',pin,self.pinCount[pin]) 
        
    #reset pinmode
    def resetPinMode(self):
        for pin in self.validPins:
            try:
                self.pinRef[pin].stop() # stop PWM from running
                self.pinRef[pin] = None
                time.sleep(0.1)
            except:
                pass
            self.pinUse[pin] = self.PUNUSED
        self.stopServod()
            

    #Procedure to set pin mode for each pin
    def setPinMode(self):
        for pin in self.validPins:
            #print pin
            if (self.pinUse[pin] == self.POUTPUT):
                print 'setting pin' , pin , ' to out' 
                try:
                    GPIO.remove_event_detect(pin)
                except:
                    pass
                try:
                    self.callbackInUse[pin] = False
                except:
                    pass                    
                GPIO.setup(pin,GPIO.OUT)                    
            elif (self.pinUse[pin] == self.PINPUT):
                print 'setting pin' , pin , ' to in with pull up' 
                GPIO.setup(pin,GPIO.IN,pull_up_down=GPIO.PUD_UP)
                try:
                    GPIO.add_event_detect(pin, GPIO.BOTH)  # add  event detection on a channel
                except:
                    pass
            elif (self.pinUse[pin] == self.PINPUTDOWN):
                print 'setting pin' , pin , ' to in with pull down' 
                GPIO.setup(pin,GPIO.IN,pull_up_down=GPIO.PUD_DOWN)
                try:
                    GPIO.add_event_detect(pin, GPIO.BOTH)  # add  event detection on a channel
                except:
                    pass             
            elif (self.pinUse[pin] == self.PINPUTNONE):
                print 'setting pin' , pin , ' to in with pull down' 
                GPIO.setup(pin,GPIO.IN)
                try:
                    GPIO.add_event_detect(pin, GPIO.BOTH)  # add  event detection on a channel
                except:
                    pass             
            elif (self.pinUse[pin] == self.PCOUNT):
                if self.callbackInUse[pin] == False:
                    print 'setting pin' , pin , ' as counting pin' 
                    GPIO.setup(pin,GPIO.IN,pull_up_down=GPIO.PUD_DOWN)
                    try: # add event callback but use try block jsut in case its already set
                        GPIO.add_event_detect(pin, GPIO.RISING, callback=self.my_callback,bouncetime=2)  # add rising edge detection on a channel
                        self.callbackInUse[pin] = True
                    except Exception,e: 
                        print "Error on event detection setup on pin" ,pin
                        print str(e)
                else:
                    print ("Callback already in use")
                    
        print ("SetPinMode:",self.pinUse)
                
    def pinUpdate(self, pin, value,type = 'plain',stepDelay = 0.003):
        if (self.ledDim < 100) and (type == 'plain'):
            type = "pwm"
            value = value * self.ledDim
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
                        #print ("Stopping previous instance")
                        sghGC.pinRef[pin].stop()
                    except:
                        pass
                        
                    if (self.pinUse[pin] in [self.PINPUT,self.PINPUTNONE,self.PINPUTDOWN]): # if pin was an input then we need to clean up
                        self.pinUse[pin] = self.POUTPUT # switch it to output
                        GPIO.setup(pin,GPIO.OUT)
                        GPIO.output(pin, int(value)) # set output to 1 ot 0
                        try:
                            GPIO.remove_event_detect(pin)
                            self.callbackInUse[pin] = False
                        except:
                            pass  
                        
                    GPIO.setup(pin,GPIO.OUT) # Setup
                    self.pinRef[pin] = GPIO.PWM(pin,self.PWMFREQ) # create new PWM instance
                    #print "type of pwm:" ,self.pinRef[pin]
                    self.pinRef[pin].start(max(0,min(100,abs(value)))) # update PWM value
                    self.pinUse[pin] = self.PPWM # set pin use as PWM
                    #print 'pin' , pin , ' changed to PWM' 
                    #print ("pin",pin, "set to", value)              
            elif type == "plain":
                if self.INVERT == True: # Invert data value (useful for 7 segment common anode displays)
                    if (self.pinUse[pin] == self.POUTPUT) or (self.pinUse[pin] == self.PPWM):
                        value = abs(value - 1)
                if (self.pinUse[pin] == self.POUTPUT): # if already an output
                    #print ("pin,pinUse:%s,%s",pin,self.pinUse[pin])
                    GPIO.output(pin, int(value)) # set output to 1 ot 0
                    #print ("pin",pin, "set to", value)
                elif (self.pinUse[pin] in [self.PINPUT,self.PINPUTNONE,self.PINPUTDOWN]): # if pin is an input
                    self.pinUse[pin] = self.POUTPUT # switch it to output
                    GPIO.setup(pin,GPIO.OUT)
                    GPIO.output(pin, int(value)) # set output to 1 ot 0
                    try:
                        GPIO.remove_event_detect(pin)
                        self.callbackInUse[pin] = False
                    except:
                        pass                    
                    print 'pin' , pin , ' changed to digital out from input' 
                    print ("pin",pin, "set to", value)
                elif (self.pinUse[pin] == self.PPWM): #if pin in use for PWM
                    value = value * 100
                    self.pinRef[pin].ChangeDutyCycle(max(0,min(100,abs(value)))) # just update PWM value
                    #print ("pwm pin",pin, "set to", value)                    
                elif (self.pinUse[pin] == self.PUNUSED): # if pin is not allocated
                    self.pinUse[pin] = self.POUTPUT # switch it to output
                    GPIO.setup(pin,GPIO.OUT)
                    GPIO.output(pin,int(value)) # set output to 1 or 0
                    print 'pin' , pin , ' changed to digital out from unused' 
                    print ("pin",pin, "set to", value)
            #print pin,value,type,self.pinUse[pin]
        except ValueError:
            print "mistake made in trying to update an invalid pin"
            print pin,value,type
            pass
        
    def pinFreq(self, pin, freq):
        self.pinRef[pin].ChangeFrequency(freq)

    def pinSonar(self, pin):
        #print pin
        #print self.pinUse[pin]         
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
        
    def pinRCTime (self,pin):
        reading = 0
        #print "rc pin told to set to output"
        self.pinUpdate(pin,0)
        #print "rc changed to ouput"
        time.sleep(0.1)
        #print "sleep done"
        GPIO.setup(pin,GPIO.IN)
        #print "rc set to input"
        #time.sleep(3)
        #print "sleep 2 done"
        # This takes about 1 millisecond per loop cycle
        while (GPIO.input(pin) == GPIO.LOW) and (reading < 1000):
            reading += 1
        #print "change back to output"
        GPIO.setup(pin,GPIO.OUT)
        self.pinUpdate(pin,0)
        return reading
        
    def pinSonar2(self, trig,echo):
        #print pin
        #print self.pinUse[pin]         
        self.pinUse[trig] = self.PSONAR
        self.pinUse[echo] = self.PSONAR        
        GPIO.setup(trig,GPIO.OUT)
        GPIO.setup(echo,GPIO.OUT)        
        ti = time.time()
        # setup a list to hold 3 values and then do 3 distance calcs and store them
        #print 'sonar started'
        distlist = [0.0,0.0,0.0,0.0,0.0]
        ts=time.time()
        for k in range(5):
            #print "sonar pulse" , k
            GPIO.output(trig, 1)    # Send Pulse high
            time.sleep(0.00001)     #  wait
            GPIO.output(trig, 0)  #  bring it back low - pulse over.
            t0=time.time() # remember current time
            GPIO.setup(echo,GPIO.IN)
            #PIN_USE[i] = PINPUT don't bother telling system
            
            t1=t0
            # This while loop waits for input pin (7) to be low but with a 0.04sec timeout 
            while ((GPIO.input(echo)==0) and ((t1-t0) < 0.02)):
                #time.sleep(0.00001)
                t1=time.time()
            t1=time.time()
            #print 'low' , (t1-t0).microseconds
            t2=t1
            #  This while loops waits for input pin to go high to indicate pulse detection
            #  with 0.04 sec timeout
            while ((GPIO.input(echo)==1) and ((t2-t1) < 0.02)):
                #time.sleep(0.00001)
                t2=time.time()
            t2=time.time()
            #print 'high' , (t2-t1).microseconds
            t3=(t2-t1)  # t2 contains time taken for pulse to return
            #print "total time " , t3
            distance=t3*343/2*100  # calc distance in cm
            distlist[k]=distance
            #print distance
            GPIO.setup(echo,GPIO.OUT)
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
        except Exception,e: 
            print "Some error reading pin" ,pin
            print str(e)
            return 0
            
    def pinEvent(self, pin):
        #print "pin",pin ,"set to", self.pinUse[pin]
        #print pin ," being read"
        try:
            if self.pinEventEnabled == True:
                return GPIO.event_detected(pin)
            else:
                return False
        except Exception,e: 
            print "error reading pin event" ,pin
            print str(e)
            return False     
        
    def startServod(self, pins):
        print ("Starting servod")
        os.system("sudo pkill -f servod")
        for pin in pins:
            self.pinUse[pin] = self.PSERVOD
        os.system(SCRIPTPATH +'/sgh_servod --idle-timeout=20000 --p1pins="' + str(pins).strip('[]') + '"')
        print (SCRIPTPATH +'/sgh_servod --idle-timeout=20000 --p1pins="' + str(pins).strip('[]') + '"')
        
        self.servodPins = pins

    def pinServod(self, pin, value):
        #print ("echo " + str(self.servodPins.index(pin)) + "=" + str(value) + " > /dev/servoblaster")
        os.system("echo " + str(self.servodPins.index(pin)) + "=" + str(value) + " > /dev/servoblaster")
        
    def stopServod(self):
        os.system("sudo pkill -f servod")
        
    def findDS180(self):
        print ("Starting DS180")
        try:
            os.system('sudo modprobe w1-gpio')
            os.system('sudo modprobe  w1-therm')
            possSensors  = os.listdir('/sys/bus/w1/devices')
            #print possSensors
            for loop in possSensors:
                if loop[:2] == "28":
                     self.dsSensorId = loop
                     return True
        except:
            pass
        return False

    def getDS180Temp(self):
        temperature = -300.0
        try:
            tfile = open("/sys/bus/w1/devices/"+ self.dsSensorId  +"/w1_slave")
            # Read all of the text in the file.
            text = tfile.read()
            # Close the file now that the text has been read.
            tfile.close()
            # Split the text with new lines (\n) and select the second line.
            secondline = text.split("\n")[1]
            # Split the line into words, referring to the spaces, and select the 10th word (counting from 0).
            temperaturedata = secondline.split(" ")[9]
            # The first two characters are "t=", so get rid of those and convert the temperature from a string to a number.
            temperature = float(temperaturedata[2:]) / 1000.0
        except:
            pass
        print temperature
        return temperature




#### End of main program

