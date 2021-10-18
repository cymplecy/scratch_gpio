#!/usr/bin/env python
# sgh_GPIOController - control Raspberry Pi GPIO ports using RPi.GPIO by Ben Crosten
#                      and servod by Richard Hirst
#                       Tidied up gpioBoth callback code kindly supplied by Charlotte Godley 
#Copyright (C) 2013-2015 by Simon Walters 

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
#V8 16DEc19 Bug fix - initilise pigpio value


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
        return GPIO.RPI_INFO['P1_REVISION']
    
    def cleanup(self):
        GPIO.cleanup()

    def __init__(self, debug=False):
        GPIO.setmode(GPIO.BOARD)
        GPIO.setwarnings(False)
        GPIO.cleanup()
        self.piRevision = self.getPiRevision()
        self.i2cbus = 1
        if self.piRevision == 1:
            self.i2cbus = 0
        print "RPi.GPIO Version" , GPIO.VERSION
        print "Board Revision" , self.piRevision
        

        #Set some constants and initialise lists
        self.numOfPins = 27 #there are actually 26 but python can't count properly :)
        if self.piRevision > 2:
            self.numOfPins = 41
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
        self.PPWMMOTOR = 2048
        self.PPWMLED = 4096

        #self.INVERT = False
        self.ledDim = 100

        self.PWMMOTORFREQ = 10        
        
        self.dsSensorId  = ""
        self.senderLoopDelay = 0.2
        self.mFreq = 10
        self.ultraFreq = 0.5
        self.ultraSamples = 7      
        self.pFreq = 200
       
       
       

        self.pinUse = [self.PUNUSED] * self.numOfPins
        self.servodPins = None
        
        self.pinRef = [None] * self.numOfPins
        self.pinCount = [0] * self.numOfPins
        self.countDirection = [1] * self.numOfPins
        self.pinEncoderDiff = [0] * self.numOfPins
        self.encoderStopCounting = [0] * self.numOfPins
        self.pinLastState = [0] * self.numOfPins
        self.encoderTime = [0] * self.numOfPins
        self.encoderTimeDiff = [0.0] * self.numOfPins
        self.gpioLookup = [0] * self.numOfPins
        self.callbackInUse = [False] * self.numOfPins
        self.pinValue = [0] * self.numOfPins
        self.pinInvert = [False] * self.numOfPins
        #print "pinValue" , self.pinValue
        #print "pin Value 3 = ", self.pinValue[3]
        self.pinUltraRef = [None] * self.numOfPins
        self.pinTrigger = [0] * self.numOfPins
        self.pinTriggerName = ["x"] * self.numOfPins
        self.anyTrigger = 0
        self.pinServoValue = [None] * self.numOfPins
        self.gpioMyPinEventDetected = [False] * self.numOfPins
        self.pinTriggerLastState = [0] * self.numOfPins        
        self.encoderCallback = 0
        self.piAndBash = [self.PUNUSED] * 16
        
        self.pinEventEnabled = True
        self.encoderInUse = 0
        
        self.nunchuckLevel = 1

        self.capTouch = None
        self.capTouchHelper = None
        self.ADS1015 = None
        self.lightDirection = 0
        self.lightValue = 0
        self.lightInfo = False
        self.mqttBroker = None
        self.mqttListener = None
        self.mqttClient = None
        self.mqttTopic = None
        self.mqttRetainFlag = True

        
        self.pinMapName = [None] * self.numOfPins  
        self.stepperAPos = 0
        self.stepperBPos = 0
        self.stepperDPos = 0
        self.totalLoopTime = 0
        self.scrolldelay = 0.1
        self.flag = "off"
        self.stepperPins = [[11, 12, 13, 15], [16, 18, 22, 7], [33, 32, 31, 29], [ 38, 37, 36, 35]]
        
        self.pigpio = None

        self.validPins =      [ 3,         5,       7, 8,   10,11,12,13,   15,16,   18,19,   21,22,23,24,   26]
        
        
        if self.piRevision == 1:
        #                       0  1  2  3  4  5  6  7  8  9 10 11 12 13 14 15 16 17 18 19 20 21 22 23 24 25 26 27 28 29 30 31 32 33 34 35 36 37 38 39 40
        #                     [ 3, 5,99,99, 7,99,99,26,24,21,19,23,99,99, 8,10,99,11,12,99,99,13,15,16,18,22,99]
            self.gpioLookup = [99,99,99, 0,99, 1,99, 4,14,99,15,17,18,21,99,22,23,99,24,10,99, 9,25,11, 8,99, 7]
        if self.piRevision == 2:
            self.gpioLookup = [99,99,99, 2,99, 3,99, 4,14,99,15,17,18,27,99,22,23,99,24,10,99, 9,25,11, 8,99, 7]
            
        if self.piRevision > 2:
            self.validPins =  [3,           5,       7, 8,   10,11,12,13,   15,16,   18,19,   21,22,23,24,   26,      29,   31,32,33,   35,36,37,38,   40]
            self.gpioLookup = [99,99,99, 2,99, 3,99, 4,14,99,15,17,18,27,99,22,23,99,24,10,99, 9,25,11, 8,99, 7,99,99, 5,99, 6,12,13,99,19,16,26,20,99,21]

        self.revgpioLookup = [99] *41    
        for index, item in enumerate(self.gpioLookup):
            if item != 99:
                self.revgpioLookup[item] = index
        #print self.revgpioLookup        
        #self.ULTRA_IN_USE = [False] * self.PINS
        #self.ultraTotalInUse = 0
        #self.ultraSleep = 1.0
        self.addon_from_file = None
        try:
            with open('/boot/addon.txt') as afile:
                self.addon_from_file = afile.read().lower().strip().replace('\n','').replace('\r','')
                print "AddOn from file:",self.addon_from_file
        except:
            pass
        print ("self.addon_from_file:" + str(self.addon_from_file))
        self.debug = debug
        if self.debug:
            print "sghGC Debug enabled"
        # End init
        
    # def my_callbackA(self,pin):
        # #return
        # name = "A"
        # if self.debug:
            # print name, "Callback"
        # if self.debug:
            # print name, "event ", dt.datetime.now(), pin
        # #if pin != 12:
            # #return
        
        # val = self.pinRead(pin)
        # if self.debug:
            # print name, "callback pin ", dt.datetime.now(), pin , val
        
        # time.sleep(0.010)
        # if self.debug:
            # print name, "callback pin after delay",  dt.datetime.now(),pin , val
        # val2 = self.pinRead(pin)
        # if val == val2:
            # if val != self.pinLastState[pin]:
                # self.pinCount[pin] += (self.countDirection[pin] * 1) # inc or dec count based on direction required
                # self.pinLastState[pin] = val2
                # if self.debug:
                    # print  name, "1 count ",dt.datetime.now(),self.pinCount[pin]
            # else:
                # if val == 1:
                    # self.pinCount[pin] += (self.countDirection[pin] * 2) # inc or dec count based on direction required
                    # self.pinLastState[pin] = val2
                    # if self.debug:
                        # print  name, "2count on 1 ",dt.datetime.now(),self.pinCount[pin]     
                # else:
                    # self.pinLastState[pin] = val2
                    # self.pinCount[pin] += (self.countDirection[pin] * 0)
                    # if self.debug:
                        # print  name ,"0count on 0 ",dt.datetime.now(),self.pinCount[pin]  
                    
                    
    # def my_callbackB(self,pin):
        # name = "B"
        # if self.debug:
            # print name, "Callback"
        # if self.debug:
            # print name, "event ", dt.datetime.now(), pin
        # #if pin != 12:
            # #return
        
        # val = self.pinRead(pin)
        # if self.debug:
            # print name, "callback pin ", dt.datetime.now(), pin , val
        
        # time.sleep(0.010)
        # if self.debug:
            # print name, "callback pin after delay",  dt.datetime.now(),pin , val
        # val2 = self.pinRead(pin)
        # if val == val2:
            # if val != self.pinLastState[pin]:
                # self.pinCount[pin] += (self.countDirection[pin] * 1) # inc or dec count based on direction required
                # self.pinLastState[pin] = val2
                # if self.debug:
                    # print  name, "1 count ",dt.datetime.now(),self.pinCount[pin]
            # else:
                # if val == 1:
                    # self.pinCount[pin] += (self.countDirection[pin] * 2) # inc or dec count based on direction required
                    # self.pinLastState[pin] = val2
                    # if self.debug:
                        # print  name, "2count on 1 ",dt.datetime.now(),self.pinCount[pin]     
                # else:
                    # self.pinLastState[pin] = val2
                    # self.pinCount[pin] += (self.countDirection[pin] * 0)
                    # if self.debug:
                        # print  name ,"0count on 0 ",dt.datetime.now(),self.pinCount[pin]               

    ### Callback input edge detect routine that ignores transients - Readability inspired by @Charwarz
    def oldgpioBoth(self,pin,delay = 0.030):
        #print dt.datetime.now() , "pin",pin,"laststate", self.pinTriggerLastState[pin]
        
        ### Average out the pin state over the delay period
        t0=time.time()
        avg = 0.0
        count = 1 # should be zero but this prevents a possible div by zero error later
        while (time.time() - t0) < delay:
            avg = avg + self.pinRead(pin)
            count = count + 1
        avg = float(avg) / float(count)
        ###
        
        #print dt.datetime.now(), "pin",pin,"average value over delay",avg
        pinStateOverDelayPeriod = 1 if avg > 0.5 else 0

        if self.pinTriggerLastState[pin] != pinStateOverDelayPeriod: 
            # update the state if confirmed change has occured
            #print dt.datetime.now(),"pin",pin,"changed to", pinStateOverDelayPeriod
            self.gpioMyPinEventDetected[pin] = True
        #else:
            #print dt.datetime.now(),"Transient Detected on pin",pin

        self.pinTriggerLastState[pin] = pinStateOverDelayPeriod
        
    def gpioBoth(self,pin,delay = 0.030):
        #print dt.datetime.now() , "pin",pin,"laststate", self.pinTriggerLastState[pin]
        
        ### Average out the pin state over the delay period
        avg = 0
        for loop in range(0,10):
            avg = avg + self.pinRead(pin)
        avg = float(avg) / 10.0
        ###
        
        #print dt.datetime.now(), "pin",pin,"average value over delay",avg
        pinStateOverDelayPeriod = 1 if avg > 0.5 else 0

        if self.pinTriggerLastState[pin] != pinStateOverDelayPeriod: 
            # update the state if confirmed change has occured
            #print dt.datetime.now(),"pin",pin,"changed to", pinStateOverDelayPeriod
            self.gpioMyPinEventDetected[pin] = True
        #else:
            #print dt.datetime.now(),"Transient Detected on pin",pin

        self.pinTriggerLastState[pin] = pinStateOverDelayPeriod        


                    
    def my_callbackA(self,pin):
        #return
        name = "A"
        if self.debug:
            print name, "Callback"
        if self.debug:
            print name, "event ", dt.datetime.now(), pin
        #if pin != 12:
            #return
        
        val = self.pinRead(pin)
        if self.debug:
            print name, "callback pin ", dt.datetime.now(), pin , val
        
        time.sleep(0.010)
        if self.debug:
            print name, "callback pin after delay",  dt.datetime.now(),pin , val
        val2 = self.pinRead(pin)
        if val == val2:
            if val == 1:
                self.pinCount[pin] += (self.countDirection[pin] * 1) # inc or dec count based on direction required
                self.pinLastState[pin] = val2
                if self.debug:
                    print  name, "1count on 1 ",dt.datetime.now(),self.pinCount[pin]   
                    
                    
    def my_callbackB(self,pin):
        name = "B"
        if self.debug:
            print name, "Callback"
        if self.debug:
            print name, "event ", dt.datetime.now(), pin
        #if pin != 12:
            #return
        
        val = self.pinRead(pin)
        if self.debug:
            print name, "callback pin ", dt.datetime.now(), pin , val
        
        time.sleep(0.010)
        if self.debug:
            print name, "callback pin after delay",  dt.datetime.now(),pin , val
        val2 = self.pinRead(pin)
        if val == val2:
            if val == 1:
                self.pinCount[pin] += (self.countDirection[pin] * 1) # inc or dec count based on direction required
                self.pinLastState[pin] = val2
                if self.debug:
                    print  name, "1count on 1 ",dt.datetime.now(),self.pinCount[pin]     
                   
        
    #reset pinmode
    def resetPinMode(self):
        print "resetting pin mode" 
        self.stopServod()
        print "servod stopped"
        for pin in self.validPins:
            try:
                self.pinRef[pin].stop() # stop PWM from running
                self.pinRef[pin] = None
            except:
                pass
            self.pinRef[pin] = None #reset pwm flag
                
            try:
                GPIO.remove_event_detect(pin) #Stop Any event detection for input and counting
            except:
                pass
                
            try:
                self.callbackInUse[pin] = False  #reset event callback flags
            except:
                pass
                
            if (self.pinUse[pin] == self.POUTPUT):
                GPIO.setup(pin,GPIO.IN)   
            elif (self.pinUse[pin] == self.PINPUT):
                GPIO.setup(pin,GPIO.IN)   
            elif (self.pinUse[pin] == self.PINPUTDOWN):
                GPIO.setup(pin,GPIO.IN)  
            elif (self.pinUse[pin] == self.PINPUTNONE):
                GPIO.setup(pin,GPIO.IN)
            elif (self.pinUse[pin] == self.PCOUNT):
                GPIO.setup(pin,GPIO.IN)
            self.pinUse[pin] = self.PUNUSED
            self.pinServoValue[pin] = None
            
            print "reset pin", pin
            self.pinValue[pin] = 0
            self.pinInvert[pin] = False
            

    #Procedure to set pin mode for each pin
    def setPinMode(self):
        for pin in self.validPins:
            #print pin
            try:
                GPIO.remove_event_detect(pin)
            except:
                pass
            try:
                self.callbackInUse[pin] = False
            except:
                pass
            if (self.pinUse[pin] == self.POUTPUT):
                #print 'setting pin' , pin , ' to out' 
                                    
                GPIO.setup(pin,GPIO.OUT)
                if (self.pinInvert[pin] == True):
                    GPIO.output(pin,1)
                else:
                    GPIO.output(pin,0)
                self.pinValue[pin] = 0
            elif (self.pinUse[pin] == self.PINPUT):
                #print 'setting pin' , pin , ' to in with pull up' 
                GPIO.setup(pin,GPIO.IN,pull_up_down=GPIO.PUD_UP)
                try:
                    GPIO.add_event_detect(pin, GPIO.BOTH, callback=self.gpioBoth,bouncetime=50)  # add rising edge detection on a channel
                except:
                    pass
            elif (self.pinUse[pin] == self.PINPUTDOWN):
                #print 'setting pin' , pin , ' to in with pull down' 
                GPIO.setup(pin,GPIO.IN,pull_up_down=GPIO.PUD_DOWN)
                try:
                    GPIO.add_event_detect(pin, GPIO.BOTH, callback=self.gpioBoth,bouncetime=50)  # add rising edge detection on a channel
                except:
                    pass             
            elif (self.pinUse[pin] == self.PINPUTNONE):
                #print 'setting pin' , pin , ' to in with pull down' 
                GPIO.setup(pin,GPIO.IN)
                try:
                    GPIO.add_event_detect(pin, GPIO.BOTH, callback=self.gpioBoth,bouncetime=50)  # add rising edge detection on a channel
                except:
                    pass             
            elif (self.pinUse[pin] == self.PCOUNT):
                if self.callbackInUse[pin] == False:
                    #print 'setting pin' , pin , ' as counting pin' 
                    GPIO.setup(pin,GPIO.IN)#,pull_up_down=GPIO.PUD_DOWN)#,pull_up_down=GPIO.PUD_DOWN)
                    try: # add event callback but use try block just in case its already set
                        if self.encoderCallback == 1:
                            #GPIO.add_event_detect(pin, GPIO.RISING, callback=self.my_callbackB)#,bouncetime=10)  # add rising edge detection on a channel
                            self.callbackInUse[pin] = True
                            self.encoderCallback = 2
                            if self.debug:
                                print "callback B set for pin ", pin
                            
                        if self.encoderCallback == 0:
                            #GPIO.add_event_detect(pin, GPIO.RISING, callback=self.my_callbackA)#,bouncetime=10)  # add rising edge detection on a channel
                            self.callbackInUse[pin] = True
                            self.encoderCallback = 1
                            if self.debug:
                                print "callback A set for pin ", pin
                            
                    except Exception,e: 
                        print "Error on event detection setup on pin" ,pin
                        print str(e)
                else:
                    print ("Callback already in use")
                    
        print ("SetPinMode:",self.pinUse)
                
    def pinUpdate(self, pin, value,type = 'plain',stepDelay = 0.003):
        #print "pinUpdate p,v,t,pwmref: ",pin,value,type,self.pinRef[pin]
        #print "pin",pin
        #print "pvalue",self.pinValue
        self.pinValue[pin] = value
        self.mFreq = max(5,abs(value/2))
        if (self.ledDim < 100) and (type == 'plain'):
            type = "pwm"
            value = value * self.ledDim
        try:
            #print pin,value,type,self.pinUse[pin]
            if type[0:3] == "pwm": # 
                #print "processing pwm"
                #return
                if (self.pinInvert[pin] == True): # Invert data value (needed for active low devices)
                    value = 100 - abs(value)
                    
                #print "motor freq calc", self.mFreq
                try: 
                    #print "try jsut updating pwm"
                    self.pinRef[pin].ChangeDutyCycle(max(0,min(100,abs(value)))) # just update PWM value
                    if type == "pwmmotor":
                        #print "motor freq used a", self.mFreq
                        self.pinRef[pin].ChangeFrequency(self.mFreq) # change freq to motor freq
                    elif type != "pwmbeep":
                        #print "motor freq used a", self.mFreq
                        self.pinRef[pin].ChangeFrequency(self.pFreq) # change freq to motor freq                        
             
                    #print "updating pwm suceceed"
                except:
                    #print "pwm not set so now setting up"
                    try:
                        GPIO.remove_event_detect(pin)
                        self.callbackInUse[pin] = False
                    except:
                        pass
                       
                    GPIO.setup(pin,GPIO.OUT) # Setup
                    if type == "pwmmotor":
                        #print "motor freq used b", self.mFreq
                        self.pinRef[pin] = GPIO.PWM(pin,self.mFreq) # create new PWM instance
                    elif type != "pwmbeep":
                        #print "motor freq used a", self.mFreq
                        self.pinRef[pin] = GPIO.PWM(pin,self.pFreq) # create new PWM instance
                    self.pinRef[pin].start(max(0,min(100,abs(value)))) # update PWM value
                    #print "pwm setup on pin",pin, "now has ref",self.pinRef[pin]
                    self.pinUse[pin] = self.PPWM # set pin use as PWM
         
            elif type == "plain":
                #print "Plain processing- Pin " , pin , " commanded to be " , value
                #print "pinUpdate p,v,t,pwmref: ",pin,value,type,self.pinRef[pin]
                if (self.pinInvert[pin] == True): # Invert data value (useful for 7 segment common anode displays)
                    value = 1 - abs(value)
                if (self.pinUse[pin] == self.POUTPUT): # if already an output
                    GPIO.output(pin, int(value)) # set output to 1 ot 0
                    #print 'pin' , pin , ' was already an output.  Now set to' , value
                    
                elif (self.pinUse[pin] in [self.PINPUT,self.PINPUTNONE,self.PINPUTDOWN]): # if pin is an input
                    try:
                        GPIO.remove_event_detect(pin)
                        self.callbackInUse[pin] = False
                    except:
                        pass  
                    self.pinUse[pin] = self.POUTPUT # switch it to output
                    GPIO.setup(pin,GPIO.OUT)
                    GPIO.output(pin, int(value)) # set output to 1 to 0
                    #print 'pin' , pin , ' was an input - change to output value' , value                    
               
                  
                elif (self.pinUse[pin] == self.PUNUSED): # if pin is not allocated
                    self.pinUse[pin] = self.POUTPUT # switch it to output
                    GPIO.setup(pin,GPIO.OUT)
                    GPIO.output(pin,int(value)) # set output to 1 or 0
                    #print 'pin' , pin , ' was ununsed - now out value ' , value            

                elif (self.pinUse[pin] == self.PPWM): # pin was set as pwm
                    #print "pinUpdate p,v,t,pwmref: ",pin,value,type,self.pinRef[pin]
                    try:
                        #print "pinUpdate p,v,t,pwmref: ",pin,value,type,self.pinRef[pin]
                        #print ("Stopping previous instance on pin",pin)
                        #print "pinref on pin" ,pin , "is" ,self.pinRef[pin]
                        self.pinRef[pin].stop()
                        #print ("previous instance on pin",pin ,"stopped")
                        self.pinRef[pin] = None
                    except:
                        pass
                    self.pinUse[pin] = self.POUTPUT # switch it to output

                    GPIO.setup(pin,GPIO.OUT)
                    #print "switched to output"
                    GPIO.output(pin, int(value)) # set output to 1 to 0
                    
                    #print 'pin' , pin , ' was PWM now set to ' , value                       
        except ValueError:
            print "mistake made in trying to update an invalid pin"
            print pin,value,type
            pass
            
    def motorUpdate(self, Pin1, Pin2, value):
        #print "motUpdate called" , Pin1, Pin2,value 
        #self.mFreq = max(11,abs(value/2))
        #print "mFreq= " , self.mFreq
        PinPWM = Pin1
        PinC = Pin2
        
        if value == 0:
            self.pinUpdate(PinPWM,0)
            self.pinUpdate(PinC,0)
            return
        # print
        # print "motorupdate:",value,Pin1,self.pinValue[Pin1],Pin2,self.pinValue[Pin2]
        # print
        
        #Add in delay when switch from forward to back to minimise current spike
        oldValue = self.pinValue[Pin1] + self.pinValue[Pin2]
        if ((value > 0) and (oldValue < 0)) or ((value < 0) and (oldValue > 0)):
            self.pinUpdate(PinPWM,0)
            self.pinUpdate(PinC,0)
            time.sleep(0.2)
            
        if value < 0:
            temp = Pin1 #swap which pin is pwm'd
            PinPWM = PinC
            PinC = temp
                
        self.pinUpdate(PinPWM,value,type="pwmmotor")
        self.pinUpdate(PinC,0)
            
        #self.pinValue[PinPWM] = value
        #self.pinValue[PinC] = 0

        
        
    def pinFreq(self, pin, freq):
        self.pinRef[pin].ChangeFrequency(freq)
        
    def changePWMFreq(self):
        for pin in self.validPins:
            try:
                #print "changefreq" , pin
                self.pinFreq(pin,self.pFreq)
            except:
                pass
        return

    def pinSonar(self, trig,echo):
        #modified 16Oct21 to handle being called with two pin umbers (which will be equal anyway)
        #print pin
        #print self.pinUse[pin]         
        self.pinUse[trig] = self.PSONAR
        GPIO.setup(trig,GPIO.OUT)
        ti = time.time()
        # setup a list to hold 3 values and then do 3 distance calcs and store them
        #print 'sonar started' 
        distlist = [0] * self.ultraSamples
        distance = 0
        ts=time.time()

        try:
            for k in range(self.ultraSamples):
                #print "sonar pulse" , k
                GPIO.output(trig, 0)
                time.sleep(0.06)#set pin low for 60ms as per spec sheet to allow for old pulses not finished
                GPIO.output(trig, 1)    # Send Pulse high
                time.sleep(0.00002)     #  wait
                GPIO.output(trig, 0)  #  bring it back low - pulse over.
                t0=time.time() # remember current time
                GPIO.setup(echo,GPIO.IN)
                #PIN_USE[i] = PINPUT don't bother telling system

                t1=t0
                # This while loop waits for input pin (7) to be low but with a timeout
                while ((GPIO.input(echo)==0) and ((t1-t0) < 0.02)):
                    #time.sleep(0.00001)
                    t1=time.time()
                t1=time.time()
                #print 'low' , (t1-t0) * 1000
                t2=t1
                #  This while loops waits for input pin to go high to indicate pulse detection
                #  with  timeout
                #tcount = 0
                while ((GPIO.input(echo)==1) and ((t2-t1) < 0.02)):
                    #time.sleep(0.000005)
                    t2=time.time()
                    #tcount += 1
                t2=time.time()
                #print "tcount",tcount
                #print 'high' , (t2-t1).microseconds
                t3=(t2-t1)  # t2 contains time taken for pulse to return
                #print "time of pulse flight " , t3 * 1000
                # 20cm (40 in total ~~ 1.2 milliseconds)
                #distance = t3 * 17150  # calc distance in cm t3 * 343 / 2 * 100
                distlist[k]=int(t3 * 17150)
                #print distance
                GPIO.setup(trig,GPIO.OUT)
            #tf = time.time() - ts
            #print ("Proctime:",tf)
            #print ("Dist:",sorted(distlist))
            #print "mid: ", self.ultraSamples / 2
            distance = sorted(distlist)[int(self.ultraSamples / 2)] # sort the list and pick middle value as best distance

        except:
            print ("ultra fail")
            pass
        
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
        #distance = "x:" +str(int(distlist[0])) + "*" + str(int(distlist[1])) +"*" + str(int(distlist[2])) +"*" + str(int(distlist[3])) +"*" + str(int(distlist[4]))
        #distance = "fred"
        return int(distance)
        
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
        ti = time.time()
        # setup a list to hold 3 values and then do 3 distance calcs and store them
        #print 'sonar started' 
        distlist = [0] * self.ultraSamples
        distance = 0
        ts=time.time()

        try:
            for k in range(self.ultraSamples):
                #print "sonar pulse" , k
                GPIO.output(trig, 0)
                time.sleep(0.06)#set pin low for 60ms as per spec sheet to allow for old pulses not finished
                GPIO.output(trig, 1)    # Send Pulse high
                time.sleep(0.00002)     #  wait
                GPIO.output(trig, 0)  #  bring it back low - pulse over.
                t0=time.time() # remember current time
                GPIO.setup(echo,GPIO.IN)
                #PIN_USE[i] = PINPUT don't bother telling system

                t1=t0
                # This while loop waits for input pin (7) to be low but with a timeout
                while ((GPIO.input(echo)==0) and ((t1-t0) < 0.02)):
                    #time.sleep(0.00001)
                    t1=time.time()
                t1=time.time()
                #print 'low' , (t1-t0) * 1000
                t2=t1
                #  This while loops waits for input pin to go high to indicate pulse detection
                #  with  timeout
                #tcount = 0
                while ((GPIO.input(echo)==1) and ((t2-t1) < 0.02)):
                    #time.sleep(0.000005)
                    t2=time.time()
                    #tcount += 1
                t2=time.time()
                #print "tcount",tcount
                #print 'high' , (t2-t1).microseconds
                t3=(t2-t1)  # t2 contains time taken for pulse to return
                #print "time of pulse flight " , t3 * 1000
                # 20cm (40 in total ~~ 1.2 milliseconds)
                #distance = t3 * 17150  # calc distance in cm t3 * 343 / 2 * 100
                distlist[k]=int(t3 * 17150)
                #print distance
                GPIO.setup(trig,GPIO.OUT)
            #tf = time.time() - ts
            #print ("Proctime:",tf)
            #print ("Dist:",sorted(distlist))
            #print "mid: ", self.ultraSamples / 2
            distance = sorted(distlist)[int(self.ultraSamples / 2)] # sort the list and pick middle value as best distance

        except:
            print ("ultra fail")
            pass
        
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
        #distance = "x:" +str(int(distlist[0])) + "*" + str(int(distlist[1])) +"*" + str(int(distlist[2])) +"*" + str(int(distlist[3])) +"*" + str(int(distlist[4]))
        #distance = "fred"
        return int(distance)
        
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
                #return GPIO.event_detected(pin)
                if self.gpioMyPinEventDetected[pin] == True:
                    self.gpioMyPinEventDetected[pin] = False
                    return True
                else:
                    return False
            else:
                return False
        except Exception,e: 
            print "error reading pin event" ,pin
            print str(e)
            return False     
        
    def startServod(self, pins):
        print (" ")
        print ("Starting servod")
        os.system("sudo pkill -f servod")
        time.sleep(0.1)
        print "any running servod killed"
        for pin in pins:
            self.pinUse[pin] = self.PSERVOD
            
        pins = []
        for pin in self.validPins:
            if self.pinUse[pin] == self.PSERVOD:
                pins.append(pin)
        print pins
        
        os.system(SCRIPTPATH +'/sgh_servod --idle-timeout=20000 --p1pins="' + str(pins).strip('[]') + '"')
        #print (SCRIPTPATH +'/sgh_servod --idle-timeout=20000 --p1pins="' + str(pins).strip('[]') + '"')
        print "servod started"
        
        self.servodPins = pins

    def pinServod(self, pin, value):
        print pin , "=" , value
        if self.pinUse[pin] != self.PSERVOD:
            self.startServod([pin])
        self.pinServoValue[pin] = value
        for pin in self.servodPins:
            os.system("echo " + str(self.servodPins.index(pin)) + "=" + str(self.pinServoValue[pin]) + " > /dev/servoblaster")
            print ("echo " + str(self.servodPins.index(pin)) + "=" + str(self.pinServoValue[pin]) + " > /dev/servoblaster")

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
                     print "DS18B found"
                     return True
        except:
            pass
        print "ds18b not found"
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
            #print "2nd line",secondline
            # Split the line into words, referring to the spaces, and select the 10th word (counting from 0).
            temperaturedata = secondline.split(" ")[9]
            #print "temp data", temperaturedata
            # The first two characters are "t=", so get rid of those and convert the temperature from a string to a number.
            temperature = float(temperaturedata[2:]) / 1000.0
        except:
            pass
        print temperature
        return temperature

    def setPinInvert(self, pin, state = False):
        self.pinInvert[pin] = state
        
    def setAllInvert(self, state = False):
        for pin in self.validPins:
            self.pinInvert[pin] = state  


#### End of main program

