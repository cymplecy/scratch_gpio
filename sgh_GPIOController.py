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
        print "RPi.GPIO Version" , GPIO.VERSION
        print "Board Revision" , self.piRevision
        

        #Set some constants and initialise lists
        self.numOfPins = 41 #there are actually 40 but python can't count properly :)
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

        self.PWMFREQ = 200
        self.PWMMOTORFREQ = 10        
        
        self.dsSensorId  = ""
        self.senderLoopDelay = 0.2
        self.mFreq = 10
        self.ultraFreq = 1
       
       
       

        self.pinUse = [self.PUNUSED] * self.numOfPins
        self.servodPins = None
        
        self.pinRef = [None] * self.numOfPins
        self.pinCount = [0] * self.numOfPins
        self.countDirection = [1] * self.numOfPins
        self.pinEncoderDiff = [0] * self.numOfPins
        self.pinLastState = [0] * self.numOfPins
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
        self.encoderCallback = 0
        
        self.pinEventEnabled = True
        
        self.nunchuckLevel = 1
		
        if self.piRevision == 1:
        #                       0  1  2  3  4  5  6  7  8  9 10 11 12 13 14 15 16 17 18 19 20 21 22 23 24 25 26 27 28 29 30 31 32 33 34 35 36 37 38 39 40
        #                     [ 3, 5,99,99, 7,99,99,26,24,21,19,23,99,99, 8,10,99,11,12,99,99,13,15,16,18,22,99]
            self.gpioLookup = [99,99,99, 0,99, 1,99, 4,14,99,15,17,18,21,99,22,23,99,24,10,99, 9,25,11, 8,99, 7,99,99,99,99,99,99,99,99,99,99,99,99,99,99]
        else:                #[99,99, 2,99, 7, 3,99,26,24,21,19,23,99,99, 8,10,99,11,12,99,99,13,15,16,18,22,99]
            self.gpioLookup = [99,99,99, 2,99, 3,99, 4,14,99,15,17,18,27,99,22,23,99,24,10,99, 9,25,11, 8,99, 7,99,99, 5,99, 6,12,13,99,19,16,26,20,99,21]
            
        self.validPins = [3,5,7,8,10,11,12,13,15,16,18,19,21,22,23,24,26,29,31,32,33,35,36,37,38,40]
        
        
        #self.ULTRA_IN_USE = [False] * self.PINS
        #self.ultraTotalInUse = 0
        #self.ultraSleep = 1.0
        self.debug = debug
        if self.debug:
            print "sghGC Debug enabled"
        # End init
    
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
            if val != self.pinLastState[pin]:
                self.pinCount[pin] += (self.countDirection[pin] * 1) # inc or dec count based on direction required
                self.pinLastState[pin] = val2
                if self.debug:
                    print  name, "1 count ",dt.datetime.now(),self.pinCount[pin]
            else:
                if val == 1:
                    self.pinCount[pin] += (self.countDirection[pin] * 2) # inc or dec count based on direction required
                    self.pinLastState[pin] = val2
                    if self.debug:
                        print  name, "2count on 1 ",dt.datetime.now(),self.pinCount[pin]     
                else:
                    self.pinLastState[pin] = val2
                    self.pinCount[pin] += (self.countDirection[pin] * 0)
                    if self.debug:
                        print  name ,"0count on 0 ",dt.datetime.now(),self.pinCount[pin]  
                    
                    
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
            if val != self.pinLastState[pin]:
                self.pinCount[pin] += (self.countDirection[pin] * 1) # inc or dec count based on direction required
                self.pinLastState[pin] = val2
                if self.debug:
                    print  name, "1 count ",dt.datetime.now(),self.pinCount[pin]
            else:
                if val == 1:
                    self.pinCount[pin] += (self.countDirection[pin] * 2) # inc or dec count based on direction required
                    self.pinLastState[pin] = val2
                    if self.debug:
                        print  name, "2count on 1 ",dt.datetime.now(),self.pinCount[pin]     
                else:
                    self.pinLastState[pin] = val2
                    self.pinCount[pin] += (self.countDirection[pin] * 0)
                    if self.debug:
                        print  name ,"0count on 0 ",dt.datetime.now(),self.pinCount[pin]                  
        
    #reset pinmode
    def resetPinMode(self):
        print "resetting pin mode" 
        self.stopServod()
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
                if (self.pinInvert[pin] == True):
                    GPIO.output(pin,1)
                else:
                    GPIO.output(pin,0)
                self.pinValue[pin] = 0
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
                    GPIO.setup(pin,GPIO.IN,pull_up_down=GPIO.PUD_DOWN)#,pull_up_down=GPIO.PUD_DOWN)
                    try: # add event callback but use try block just in case its already set
                        if self.encoderCallback == 1:
                            GPIO.add_event_detect(pin, GPIO.BOTH, callback=self.my_callbackB,bouncetime=10)  # add rising edge detection on a channel
                            self.callbackInUse[pin] = True
                            self.encoderCallback = 2
                            if self.debug:
                                print "callback B set for pin ", pin
                            
                        if self.encoderCallback == 0:
                            GPIO.add_event_detect(pin, GPIO.BOTH, callback=self.my_callbackA,bouncetime=10)  # add rising edge detection on a channel
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
        #print "pinUpdate p,v,t: ",pin,value,type
        #print "pin",pin
        #print "pvalue",self.pinValue
        self.pinValue[pin] = value
        self.mFreq = max(11,abs(value/2))
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
                    #if type == "pwmmotor":
                    #   print "motor freq used a", self.mFreq
                    #    self.pinRef[pin] = GPIO.PWM(pin,self.mFreq) # create new PWM instance
                    #else:
                    #    self.pinRef[pin] = GPIO.PWM(pin,self.PWMFREQ) # create new PWM instance
                    self.pinRef[pin].ChangeDutyCycle(max(0,min(100,abs(value)))) # just update PWM value
                    if type == "pwmmotor":
                        #print "motor freq used a", self.mFreq
                        self.pinRef[pin].ChangeFrequency(self.mFreq) # change freq to motor freq
                    else:
                        self.pinRef[pin].ChangeFrequency(self.PWMFREQ) # change freq to power/led freq                 
                    #print "updating pwm suceceed"
                except:
                    #print "pwm not set so now setting up"
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
                    if type == "pwmmotor":
                        print "motor freq used b", self.mFreq
                        self.pinRef[pin] = GPIO.PWM(pin,self.mFreq) # create new PWM instance
                    else:
                        self.pinRef[pin] = GPIO.PWM(pin,self.PWMFREQ) # create new PWM instance
                    self.pinRef[pin].start(max(0,min(100,abs(value)))) # update PWM value
                    self.pinUse[pin] = self.PPWM # set pin use as PWM
         
            elif type == "plain":
                #print "Plain processing- Pin " , pin , " commanded to be " , value
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

                elif (self.pinUse[pin] in [self.PPWM,self.PPWMMOTOR]): # if pin is not allocated
                    self.pinRef[pin].ChangeDutyCycle(max(0,min(100,abs(value*100)))) # just update PWM value
                    #print 'pin' , pin , ' was PWM so just updated to ' , value * 100                          
        except ValueError:
            print "mistake made in trying to update an invalid pin"
            print pin,value,type
            pass
            
    def motorUpdate(self, Pin1, Pin2, Pin3, value):
        self.mFreq = max(11,abs(value/2))
       # print "mFreq= " , self.mFreq
        PinPWM = Pin1
        PinC = Pin2
        
        cPinValue = 0
        if value >= 0:
            cPinValue = 0
        else:
            cPinValue = 0
            temp = Pin1 #swap which pin is pwm'd
            PinPWM = PinC
            PinC = temp
                
       
        try:
            try: 
                self.pinRef[PinPWM].ChangeDutyCycle(max(0,min(100,abs(value)))) # just update PWM value
                self.pinRef[PinC].ChangeDutyCycle(max(0,min(100,abs(cPinValue)))) # just update PWM value
                #print "PWM Pin ", PinPWM , "updated to value",abs(value)

            except:
                try:
                    #print ("Stopping previous instance")
                    sghGC.pinRef[PinPWM].stop()
                except:
                    pass
                try:
                    #print ("Stopping previous instance")
                    sghGC.pinRef[PinC].stop()
                except:
                    pass                    
                        
                if (self.pinUse[PinPWM] in [self.PINPUT,self.PINPUTNONE,self.PINPUTDOWN]): # if pin was an input then we need to clean up
                    try:
                        GPIO.remove_event_detect(PinPWM)
                        self.callbackInUse[PinPWM] = False
                    except:
                        pass

                #sort out cPin
                if (self.pinUse[PinC] in [self.PINPUT,self.PINPUTNONE,self.PINPUTDOWN]): # if pin is an input
                    try:
                        GPIO.remove_event_detect(PinC)
                        self.callbackInUse[PinC] = False
                    except:
                        pass  
                GPIO.setup(PinC,GPIO.OUT) # Setup
                self.pinRef[PinC] = GPIO.PWM(PinC,self.mFreq) # create new PWM instance
                self.pinRef[PinC].start(max(0,min(100,abs(cPinValue)))) # update PWM value                 
                 
                GPIO.setup(PinPWM,GPIO.OUT) # Setup
                self.pinRef[PinPWM] = GPIO.PWM(PinPWM,self.mFreq) # create new PWM instance
                self.pinRef[PinPWM].start(max(0,min(100,abs(value)))) # update PWM value  
                
                #print "PWM Pin ", PinPWM , "was not PWM - not set to value",abs(value)                
                self.pinUse[PinPWM] = self.PPWMMOTOR # set pin use as PWM
                self.pinUse[PinC] = self.PPWMMOTOR # set pin use as PWM

            self.pinValue[PinPWM] = value
            self.pinValue[PinC] = cPinValue

        
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
            GPIO.output(pin, 0)
            time.sleep(0.05)#set pin low for 50ms as per spec
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
            
        pins = []
        for pin in self.validPins:
            if self.pinUse[pin] == self.PSERVOD:
                pins.append(pin)
        print pins
        
        os.system(SCRIPTPATH +'/sgh_servod --idle-timeout=20000 --p1pins="' + str(pins).strip('[]') + '"')
        print (SCRIPTPATH +'/sgh_servod --idle-timeout=20000 --p1pins="' + str(pins).strip('[]') + '"')
        
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

