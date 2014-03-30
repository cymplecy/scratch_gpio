#!/usr/bin/env python
# ScratchGPIO - control Raspberry Pi GPIO ports using Scratch.
#Copyright (C) 2013 by Simon Walters based on original code for PiFace by Thomas Preston

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

# This code now hosted on Github thanks to Ben Nuttall
Version =  'v5.0.91' # 30Mar14 Powerxx,yy broadcast now accepted



import threading
import socket
import time
import sys
import struct
import datetime as dt
import shlex
import os
import math
import re
import sgh_GPIOController
import sgh_PiGlow
import sgh_PiMatrix
import sgh_Stepper
import logging
import subprocess
try:
	from Adafruit_PWM_Servo_Driver import PWM
	from sgh_PCF8591P import sgh_PCF8591P
	from sgh_Adafruit_8x8 import sgh_EightByEight
except:
	pass
#try and inport smbus but don't worry if not installed
#try:
#    from smbus import SMBus
#except:
#    pass

#import RPi.GPIO as GPIO


class Compass:

    __scales = {
        0.88: [0, 0.73],
        1.30: [1, 0.92],
        1.90: [2, 1.22],
        2.50: [3, 1.52],
        4.00: [4, 2.27],
        4.70: [5, 2.56],
        5.60: [6, 3.03],
        8.10: [7, 4.35],
    }

    def __init__(self, port=0, address=0x1E, gauss=1.3, declination=(0,0)):
        self.bus = SMBus(port)
        self.address = address

        (degrees, minutes) = declination
        self.__declDegrees = degrees
        self.__declMinutes = minutes
        self.__declination = (degrees + minutes / 60) * math.pi / 180

        (reg, self.__scale) = self.__scales[gauss]
        self.bus.write_byte_data(self.address, 0x00, 0x70) # 8 Average, 15 Hz, normal measurement
        self.bus.write_byte_data(self.address, 0x01, reg << 5) # Scale
        self.bus.write_byte_data(self.address, 0x02, 0x00) # Continuous measurement

    def declination(self):
        return (self.__declDegrees, self.__declMinutes)

    def twos_complement(self, val, len):
        # Convert twos compliment to integer
        if (val & (1 << len - 1)):
            val = val - (1<<len)
        return val

    def __convert(self, data, offset):
        val = self.twos_complement(data[offset] << 8 | data[offset+1], 16)
        if val == -4096: return None
        return round(val * self.__scale, 4)

    def axes(self):
        data = self.bus.read_i2c_block_data(self.address, 0x00)
        #print map(hex, data)
        x = self.__convert(data, 3)
        y = self.__convert(data, 7)
        z = self.__convert(data, 5)
        return (x,y,z)

    def heading(self):
        (x, y, z) = self.axes()
        headingRad = float(math.atan2(y, x))
        headingRad += self.__declination

        # Correct for reversed heading
        if headingRad < 0:
            headingRad += 2 * math.pi

        # Check for wrap and compensate
        elif headingRad > 2 * math.pi:
            headingRad -= 2 * math.pi

        # Convert to degrees from radians
        headingDeg = headingRad * 180 / math.pi
        degrees = math.floor(headingDeg)
        minutes = round((headingDeg - degrees) * 60)
        return headingDeg

    def degrees(self, (degrees, minutes)):
        return str(degrees) + "*" + str(minutes) + "'"
    
    def degreesdecimal(self, (degrees, minutes)):
        return str(degrees + (minutes /60.0) ) if (degrees >=0) else str(degrees - (minutes /60.0) )

    def __str__(self):
        (x, y, z) = self.axes()
        return "Axis X: " + str(x) + "\n" \
               "Axis Y: " + str(y) + "\n" \
               "Axis Z: " + str(z) + "\n" \
               "dec deg: " + str(self.__declDegrees) + "\n" \
               "dec min: " + str(self.__declMinutes) + "\n" \
               "Declination: " + self.degreesdecimal(self.declination()) + "\n" \
               "Heading: " + str(self.heading()) + "\n"
               
### End Compasss ###################################################################################################

def isNumeric(s):
    try:
        float(s)
        return True
    except ValueError:
        return False
        
def removeNonAscii(s): return "".join(i for i in s if ord(i)<128)

def xgetValue(searchString, dataString):
    outputall_pos = dataString.find((searchString + ' '))
    sensor_value = dataString[(outputall_pos+1+len(searchString)):].split()
    return sensor_value[0]
    
def sign(number):return cmp(number,0)

def parse_data(dataraw, search_string):
    outputall_pos = dataraw.find(search_string)
    return dataraw[(outputall_pos + 1 + search_string.length):].split()
    

class MyError(Exception):
    def __init__(self, value):
        self.value = value

    def __str__(self):
        return repr(self.value)

class ScratchSender(threading.Thread):

    def __init__(self, socket):
        threading.Thread.__init__(self)
        self.scratch_socket = socket
        self._stop = threading.Event()
        self.time_last_ping = 0.0
        self.time_last_compass = 0.0
        self.distlist = [0.0,0.0,0.0]
        
        


    def stop(self):
        self._stop.set()

    def stopped(self):
        return self._stop.isSet()
        
    def broadcast_changed_pins(self, changed_pin_map, pin_value_map):
        for pin in sghGC.validPins:
            #print pin
            # if we care about this pin's value
            if (changed_pin_map >> pin) & 0b1:
                #print "changed"
                pin_value = (pin_value_map >> pin) & 0b1
                if (sghGC.pinUse[pin] in [sghGC.PINPUT,sghGC.PINPUTNONE,sghGC.PINPUTDOWN]):
                    #print pin , pin_value
                    #print "broadcast changed pin"
                    self.broadcast_pin_update(pin, pin_value)
                    

                    
                                     
    def broadcast_pin_update(self, pin, value):
        #print ADDON 
        #sensor_name = "gpio" + str(GPIO_NUM[pin_index])
        #bcast_str = 'sensor-update "%s" %d' % (sensor_name, value)
        #print 'sending: %s' % bcast_str
        #self.send_scratch_command(bcast_str)   
        
        #Normal action is to just send updates to pin values but this can be modified if known addon in use
        if "ladder" in ADDON:
            #do ladderboard stuff
            sensor_name = "switch" + str([0,21,19,24,26].index(pin))
        elif "motorpitx" in ADDON:
            #do MotorPiTx stuff
            if pin == 13:
                sensor_name = "input1"
            if pin == 7:
                sensor_name = "input2"
        elif "berry" in ADDON:
            #do berryclip stuff
            if pin == 26:
                sensor_name = "switch"
            if pin == 22:
                sensor_name = "switch2"
        elif "piringo" in ADDON:
            #do PiRingo stuff
            sensor_name = "switch" + str(1 + [19,21].index(pin))
        elif "pibrella" in ADDON:
            #print pin
            #sensor_name = "in" + str([0,19,21,24,26,23].index(pin))
            try:
                sensor_name = "Input" + ["NA","A","B","C","D","E"][([0,21,26,24,19,23].index(pin))]
                if sensor_name == "InputE":
                    sensor_name = "switch"
            except:
                print "pibrella input out of range"
                sensor_name = "pin" + str(pin)
                pass
                
        elif "pidie" in ADDON:
            #print pin
            #sensor_name = "in" + str([0,19,21,24,26,23].index(pin))
            try:
                sensor_name = ["green","red","blue","yellow"][([19,21,26,24].index(pin))]
            except:
                print "pidie input out of range"
                sensor_name = "pin" + str(pin)
                pass
        elif "pi2go" in ADDON:
            #print pin
            try:
                sensor_name = ["left","front","right","lineleft","lineright","switch1","switch2","switch3"][([7,15,11,12,13,16,18,22].index(pin))]
            except:
                print "pi2go input out of range"
                sensor_name = "pin" + str(pin)
                pass                
        else:
            sensor_name = "pin" + str(pin)
        #  
            # 
            # 
            #  
        bcast_str = 'sensor-update "%s" %d' % (sensor_name, value)
        if ("piringo" in ADDON) or ("pidie" in ADDON):
            bcast_str = 'sensor-update "%s" %s' %  (sensor_name,("on","off")[value == 1])
        if ("pibrella" in ADDON):
            bcast_str = 'sensor-update "%s" %s' %  (sensor_name,("off","on")[value == 1])            
        if ("fishdish" in ADDON):
            bcast_str = 'sensor-update "switch" %s' %  (("on","off")[value == 1])
        #print 'sending: %s' % bcast_str
        self.send_scratch_command(bcast_str)
        if "motorpitx" in ADDON:
            bcast_str = 'broadcast "%s%s"' % (sensor_name,("off","on")[value == 1])
            #print 'sending: %s' % bcast_str
            self.send_scratch_command(bcast_str)
        

        
    def send_scratch_command(self, cmd):
        n = len(cmd)
        b = (chr((n >> 24) & 0xFF)) + (chr((n >> 16) & 0xFF)) + (chr((n >>  8) & 0xFF)) + (chr(n & 0xFF))
        self.scratch_socket.send(b + cmd)

    def run(self):
        global firstRun,ADDON,compass
        # while firstRun:
            # print "first run running"
        #time.sleep(5)
        last_bit_pattern=0L
        
        #self.send_scratch_command('broadcast "SetPins"')
        #print sghGC.pinUse
        with lock:
            for pin in sghGC.validPins:
                if (sghGC.pinUse[pin] in [sghGC.PINPUT,sghGC.PINPUTNONE,sghGC.PINPUTDOWN]):
                    #self.broadcast_pin_update(pin, sghGC.pinRead(pin))
                    last_bit_pattern += sghGC.pinRead(pin) << pin
                else:
                    last_bit_pattern += 1 << pin
                #print 'lbp %s' % bin(last_bit_pattern)

        last_bit_pattern = last_bit_pattern ^ -1
        lastPinUpdateTime = 0
        while not self.stopped():
            time.sleep(0.01) # be kind to cpu  :)
            #print "sender running"
            pin_bit_pattern = 0
            with lock:
                #print "lOCKED"
                for pin in sghGC.validPins:
                    #print pin
                    if (sghGC.pinUse[pin]  in [sghGC.PINPUT,sghGC.PINPUTNONE,sghGC.PINPUTDOWN]):
                        #print 'trying to read pin' , pin 
                        pin_bit_pattern += sghGC.pinRead(pin) << pin
                    else:
                        pin_bit_pattern += 1 << pin
            
            #print "unlocked"
                #print bin(pin_bit_pattern) , pin_bit_pattern
            #print bin(pin_bit_pattern) , pin_bit_pattern
            # if there is a change in the input pins
            changed_pins = pin_bit_pattern ^ last_bit_pattern
            #print "changed pins" , bin(changed_pins)
            if changed_pins:
                #print 'pin bit pattern' , bin(pin_bit_pattern)

                try:
                    self.broadcast_changed_pins(changed_pins, pin_bit_pattern)

                except Exception as e:
                    print e
                    break

            last_bit_pattern = pin_bit_pattern
            
            if (time.time() - lastPinUpdateTime)  > 2:
                #print int(time.time())
                lastPinUpdateTime = time.time()
                for pin in sghGC.validPins:
                    if (sghGC.pinUse[pin]  in [sghGC.PINPUT,sghGC.PINPUTNONE,sghGC.PINPUTDOWN]):
                        self.broadcast_pin_update(pin, sghGC.pinRead(pin))

            if (time.time() - self.time_last_ping) > 1: # Check if time to do another ultra ping
                for pin in sghGC.validPins:
                    if sghGC.pinUse[pin] == sghGC.PULTRA:
                        distance = sghGC.pinSonar(pin) # do a ping
                        sghGC.pinUse[pin] = sghGC.PULTRA # reset pin use back from sonar to ultra
                        sensor_name = 'ultra' + str(pin)
                        if "motorpitx" in ADDON:
                            if pin == 13:
                                sensor_name = "ultra1"
                            if pin == 7:
                                sensor_name = "ultra2"
                                    
                        bcast_str = 'sensor-update "%s" %d' % (sensor_name, distance)
                        #print 'sending: %s' % bcast_str
                        self.send_scratch_command(bcast_str)
                        self.time_last_ping = time.time()
    
            if (time.time() - self.time_last_compass) > 0.25:
                #print "time up"
                #print compass
                #If Compass board truely present
                if (compass != None):
                    #print "compass code"
                    heading = compass.heading()
                    sensor_name = 'heading'
                    bcast_str = 'sensor-update "%s" %d' % (sensor_name, heading)
                    #print 'sending: %s' % bcast_str
                    self.send_scratch_command(bcast_str)
                self.time_last_compass = time.time()

            #time.sleep(1)

            
class ScratchListener(threading.Thread):
    def __init__(self, socket):
        threading.Thread.__init__(self)
        self.scratch_socket = socket
        self._stop = threading.Event()
        self.dataraw = ''
        self.value = None
        self.valueNumeric = None
        self.valueIsNumeric = None
        self.OnOrOff = None
        self.encoderDiff = 0
        self.turnSpeed = 100

        
    def send_scratch_command(self, cmd):
        n = len(cmd)
        b = (chr((n >> 24) & 0xFF)) + (chr((n >> 16) & 0xFF)) + (chr((n >>  8) & 0xFF)) + (chr(n & 0xFF))
        self.scratch_socket.send(b + cmd)
                       
    def getValue(self,searchString):
        outputall_pos = self.dataraw.find((searchString + ' '))
        sensor_value = self.dataraw[(outputall_pos+1+len(searchString)):].split()
        return sensor_value[0]
        
    def bFind(self,searchStr):
        return (' '+searchStr in self.dataraw)
        
    def bFindOn(self,searchStr):
        return (self.bFind(searchStr + 'on ') or self.bFind(searchStr + 'high ') or self.bFind(searchStr + '1 '))
        
    def bFindOff(self,searchStr):
        return (self.bFind(searchStr + 'off ') or self.bFind(searchStr + 'low ') or self.bFind(searchStr + '0 '))
        
    def bFindOnOff(self,searchStr):
        self.OnOrOff = None
        if (self.bFind(searchStr + 'on ') or self.bFind(searchStr + 'high ') or self.bFind(searchStr + '1 ')):
            self.OnOrOff = 1
            return True
        elif (self.bFind(searchStr + 'off ') or self.bFind(searchStr + 'low ') or self.bFind(searchStr + '0 ')):
            self.OnOrOff = 0
            return True
        else:
            return False
            

    def bCheckAll(self):
        if self.bFindOnOff('all'):
            for pin in sghGC.validPins:
                #print pin
                if sghGC.pinUse[pin] in [sghGC.POUTPUT,sghGC.PPWM]:
                    #print pin
                    sghGC.pinUpdate(pin,self.OnOrOff)

    def bPinCheck(self):
        logging.debug("bPinCheck:%s",searchStr )    
        for pin in sghGC.validPins:
            logging.debug("bPinCheck:%s",pin )    
            if self.bFindOnOff('pin' + str(pin)):
                sghGC.pinUpdate(pin,self.OnOrOff)
            if self.bFindOnOff('gpio' + str(sghGC.gpioLookup[pin])):
                sghGC.pinUpdate(pin,self.OnOrOff)
            if self.bFindValue('power' + str(pin)):
                print pin,self.value
                if self.valueIsNumeric:
                    sghGC.pinUpdate(pin,self.valueNumeric,type="pwm")
                else:
                    sghGC.pinUpdate(pin,0,type="pwm")                  


    def bLEDCheck(self,ledList):
        for led in range(1,(1+ len(ledList))): # loop thru led numbers
            if self.bFindOnOff('led' + str(led)):
                sghGC.pinUpdate(ledList[led - 1],self.OnOrOff)
                
    def bListCheck(self,pinList,nameList):
        for loop in range(0,len(pinList)): # loop thru list
            if self.bFindOnOff(str(nameList[loop])):
                sghGC.pinUpdate(pinList[loop],self.OnOrOff)
                    
            if self.bFindValue('power' + str(nameList[loop])):
                if self.valueIsNumeric:
                    sghGC.pinUpdate(pinList[loop],self.valueNumeric,type="pwm")
                else:
                    sghGC.pinUpdate(pinList[loop],0,type="pwm")                    
                
    def bFindValue(self,searchStr):
        #logging.debug("Searching for:%s",searchStr )
        self.value = None
        self.valueNumeric = None
        self.valueIsNumeric = False
        if self.bFind(searchStr):
            self.findPos = self.dataraw.find((searchStr))
            #logging.debug("1st chars of value:%s",(self.dataraw[self.findPos + len(searchStr):]) )
            try:
                if (self.dataraw[self.findPos + len(searchStr):][0]) == " ":
                    logging.debug("space found in bfindvalue")
                    self.value = ""
                    return True
            except IndexError:
                self.value = ""
                return True
            sensor_value = self.dataraw[(self.dataraw.find((searchStr)) + 0 + len(searchStr)):].split()
            try:
                self.value = sensor_value[0]
            except IndexError:
                self.value = ""
                pass
            #print self.value
            if isNumeric(self.value):
                self.valueNumeric = float(self.value)
                self.valueIsNumeric = True
                #print "numeric" , self.valueNumeric
            return True
        else:
            return False                
            
                    
    def bLEDPowerCheck(self,ledList):
        for led in range(1,(1+ len(ledList))): # loop thru led numbers
            #print "power" +str(led) + ","
            if self.bFindValue('power' + str(led) +","):
                if self.valueIsNumeric:
                    sghGC.pinUpdate(ledList[led - 1],self.valueNumeric,type="pwm")
                else:
                    sghGC.pinUpdate(ledList[led - 1],0,type="pwm")            
        
    def vFind(self,searchStr):
        return ((' '+searchStr + ' ') in self.dataraw)
        
    def vFindOn(self,searchStr):
        return (self.vFind(searchStr + 'on') or self.vFind(searchStr + 'high')or self.vFind(searchStr + '1'))
        
    def vFindOff(self,searchStr):
        return (self.vFind(searchStr + 'off') or self.vFind(searchStr + 'low') or self.vFind(searchStr + '0'))
        
    def vFindOnOff(self,searchStr):
        self.value = None
        self.valueNumeric = None
        self.valueIsNumeric = False
        self.OnOrOff = None
        if self.vFind(searchStr):
            self.value = self.getValue(searchStr)
            if str(self.value) in ["high","on","1"]:
                self.valueNumeric = 1
                self.OnOrOff = 1
            else:
                self.valueNumeric = 0
                self.OnOrOff = 0
            return True
        else:
            return False

    def vFindValue(self,searchStr):
        #print "searching for ", searchStr 
        self.value = None
        self.valueNumeric = None
        self.valueIsNumeric = False
        if self.vFind(searchStr):
            #print "found"
            self.value = self.getValue(searchStr)
            #print self.value
            if isNumeric(self.value):
                self.valueNumeric = float(self.value)
                self.valueIsNumeric = True
                #print "numeric" , self.valueNumeric
            return True
        else:
            return False
            
    def vAllCheck(self,searchStr):
        if self.vFindOnOff(searchStr):
            for pin in sghGC.validPins:
                if sghGC.pinUse[pin] in [sghGC.POUTPUT,sghGC.PPWM]:
                    sghGC.pinUpdate(pin,self.valueNumeric)

    def vPinCheck(self):
        for pin in sghGC.validPins:
            #print "checking pin" ,pin
            if self.vFindValue('pin' + str(pin)):
                if self.valueIsNumeric:
                    sghGC.pinUpdate(pin,self.valueNumeric)
                else:
                    sghGC.pinUpdate(pin,0)
                    
            if self.vFindValue('power' + str(pin)):
                #print pin , "found"
                if self.valueIsNumeric:
                    sghGC.pinUpdate(pin,self.valueNumeric,type="pwm")
                else:
                    sghGC.pinUpdate(pin,0,type="pwm")
                    
            if self.vFindValue('motor' + str(pin)):
                if self.valueIsNumeric:
                    sghGC.pinUpdate(pin,self.valueNumeric,type="pwm")
                else:
                    sghGC.pinUpdate(pin,0,type="pwm")
                    
            if self.vFindValue('gpio' + str(sghGC.gpioLookup[pin])):
                logging.debug("gpio lookup %s",str(sghGC.gpioLookup[pin])) 
                if self.valueIsNumeric:
                    sghGC.pinUpdate(pin,self.valueNumeric)
                else:
                    sghGC.pinUpdate(pin,0)
                #time.sleep(1)
                
            if self.vFindValue('powergpio' + str(sghGC.gpioLookup[pin])):
                logging.debug("pin %s",pin )
                logging.debug("gpiopower lookup %s",str(sghGC.gpioLookup[pin])) 
                if self.valueIsNumeric:
                    sghGC.pinUpdate(pin,self.valueNumeric,type="pwm")
                else:
                    sghGC.pinUpdate(pin,0,type="pwm")
                    
    def vLEDCheck(self,ledList):
        for led in range(1,(1+ len(ledList))): # loop thru led numbers
            if self.vFindOnOff('led' + str(led)):
                sghGC.pinUpdate(ledList[led - 1],self.OnOrOff)
                    
            if self.vFindValue('power' + str(led)):
                if self.valueIsNumeric:
                    sghGC.pinUpdate(ledList[led - 1],self.valueNumeric,type="pwm")
                else:
                    sghGC.pinUpdate(ledList[led - 1],0,type="pwm")
                    
                    
    def vListCheck(self,pinList,nameList):
        for loop in range(0,len(pinList)): # loop thru led numbers
            if self.vFindOnOff(str(nameList[loop])):
                sghGC.pinUpdate(pinList[loop],self.OnOrOff)
                    
            if self.vFindValue('power' + str(nameList[loop])):
                if self.valueIsNumeric:
                    sghGC.pinUpdate(pinList[loop],self.valueNumeric,type="pwm")
                else:
                    sghGC.pinUpdate(pinList[loop],0,type="pwm")                    
                    
    def stop(self):
        self._stop.set()

    def stopped(self):
        return self._stop.isSet()

    def stepperUpdate(self, pins, value,steps=2123456789,stepDelay = 0.003):
        #print "pin" , pins , "value" , value
        #print "Stepper type", sgh_Stepper.sghStepper, "this one", type(sghGC.pinRef[pins[0]])
        try:
            sghGC.pinRef[pins[0]].changeSpeed(max(-100,min(100,value)),steps) # just update Stepper value
            #print "stepper updated"
           # print ("pin",pins, "set to", value)
        except:
            try:
                print ("Stopping PWM")
                sghGC.pinRef[pins[0]].stop()
            except:
                pass
            sghGC.pinRef[pins[0]] = None
            #time.sleep(5)
            #print ("New Stepper instance started", pins)
            sghGC.pinRef[pins[0]] = sgh_Stepper.sghStepper(sghGC,pins,stepDelay) # create new Stepper instance 
            sghGC.pinRef[pins[0]].changeSpeed(max(-100,min(100,value)),steps) # update Stepper value
            sghGC.pinRef[pins[0]].start() # update Stepper value                
           # print 'pin' , pins , ' changed to Stepper' 
            #print ("pins",pins, "set to", value)  
        sghGC.pinUse[pins[0]] = sghGC.POUTPUT
        
        
    def stopTurning(self,motorList,count,startCount):
        self.send_scratch_command('sensor-update "encoder" "turning"') #set turning sensor to turning
        countingPin = motorList[0][3] # use 1st motor counting pin only  
        print "EncoderDiffMove:",self.encoderDiff
        countwanted = startCount + count + self.encoderDiff # modifiy count based on previous result
        countattempted = startCount + count + int(1 * self.encoderDiff) # allow for modified behaviour
        countingPin = motorList[0][3] # use 1st motor counting pin only

        print countwanted,countattempted
        turningStartTime = time.time() # used to timeout if necessary 
        if count >= 0:
            while (sghGC.pinCount[countingPin]  < int(countattempted * 1) and ((time.time()-turningStartTime) < 2)):
                #Just block until count acheived or timeout occurs
                time.sleep(0.01)
        else:
            while (sghGC.pinCount[countingPin]  > int(countattempted * 1) and ((time.time()-turningStartTime) < 2)):
                #Just block until count acheived or timeout occurs
                time.sleep(0.01)
        for listLoop in range(0,2): # switch off both motors
            sghGC.pinUpdate(motorList[listLoop][1],0)
            sghGC.pinUpdate(motorList[listLoop][2],0)
        time.sleep(0.5) #wait until motors have actually stopped
        print ("Move Stopped",countingPin,(sghGC.pinCount[countingPin] - startCount))
        if self.encoderDiff == 0:
            self.encoderDiff = 1 * (countwanted - (sghGC.pinCount[countingPin])) #work out new error in position
        else:
            self.encoderDiff = 1 * (countwanted - (sghGC.pinCount[countingPin])) #work out new error in position
        print "Diff:" , self.encoderDiff
        self.send_scratch_command('sensor-update "encoder" "stopped"') # inform Scratch that turning is finished

    def beep(self,pin,freq,duration):
        logging.debug("Freq:%s", freq) 
        if sghGC.pinUse != sghGC.PPWM: # Checks use of pin if not PWM mode then
            sghGC.pinUpdate(pin,0,"pwm")  #Set pin to PWM mode
        startCount = time.time() #Get current time
        sghGC.pinFreq(pin,freq) # Set freq used for PWM cycle
        sghGC.pinUpdate(pin,50,"pwm")  # Set duty cycle to 50% to produce square wave
        while (time.time() - startCount) < (duration * 1.0): # Wait until duration has passed
            time.sleep(0.01)
        sghGC.pinUpdate(pin,0,"pwm") #Turn pin off
        

        
    # def beep(self,pin,freq,duration):
        # print freq 
        # if sghGC.pinUse != sghGC.PPWM: # Checks use of pin if not PWM mode then
            # sghGC.pinUpdate(pin,0,"pwm")  #Set pin to PWM mode
        # startCount = time.time() #Get current time
        # sghGC.pinFreq(pin,2000) # Set freq used for PWM cycle

        # while (time.time() - startCount) < (duration * 1.0): # Wait until duration has passed
            # sghGC.pinUpdate(pin,50,"pwm")  # Set duty cycle to 50% to produce square wave
            # time.sleep(0.2)#1.0 / freq)
            # sghGC.pinUpdate(pin,0,"pwm")  # Set duty cycle to 50% to produce square wave
            # time.sleep(0.2)#1.0 / freq)
        # sghGC.pinUpdate(pin,0,"pwm") #Turn pin off
        # print ("Beep Stopped")            
        


    def run(self):
        global firstRun,cycle_trace,step_delay,stepType,INVERT, \
               Ultra,ultraTotalInUse,piglow,PiGlow_Brightness,compass,ADDON
        


        #firstRun = True #Used for testing in overcoming Scratch "bug/feature"
        firstRunData = ''
        anyAddOns = False
        ADDON = ""

        #semi global variables used for servos in PiRoCon
        panoffset = 0
        tiltoffset = 0
        pan = 0
        tilt = 0
        steppersInUse = None
        beepDuration = 0.5
        beepNote = 60
        
                      
        if GPIOPlus == False:
            with lock:
                print "set pins standard"
                for pin in sghGC.validPins:
                    sghGC.pinUse[pin] = sghGC.PINPUT
                sghGC.pinUse[11] = sghGC.POUTPUT
                sghGC.pinUse[12] = sghGC.POUTPUT
                sghGC.pinUse[13] = sghGC.POUTPUT
                sghGC.pinUse[15] = sghGC.POUTPUT
                sghGC.pinUse[16] = sghGC.POUTPUT
                sghGC.pinUse[18] = sghGC.POUTPUT
                sghGC.setPinMode()
   
                               
        #This is main listening routine
        lcount = 0
        dataPrevious = ""
        debugLogging = False
        
        
        #This is the main loop that listens for messages from Scratch and sends appropriate commands off to various routines
        while not self.stopped():
            #lcount += 1
            #print lcount
            try:
                #print "try reading socket"
                BUFFER_SIZE = 512 # This size will accomdate normal Scratch Control 'droid app sensor updates
                data = dataPrevious + self.scratch_socket.recv(BUFFER_SIZE) # get the data from the socket plus any data not yet processed
                logging.debug("datalen: %s",len(data)) 
                logging.debug("RAW: %s", data)
                
                if "send-vars" in data:
                    #Reset if New project detected from Scratch
                    #tell outer loop that Scratch has disconnected
                    if cycle_trace == 'running':
                        cycle_trace = 'disconnected'
                        print "cycle_trace has changed to" ,cycle_trace
                        break
                
                if len(data) > 0: # Connection still valid so process the data received
                
                    dataIn = data 
                    #dataOut = ""
                    dataList = [] # used to hold series of broadcasts or sensor updates
                    dataPrefix = "" # data to be re-added onto front of incoming data
                    while len(dataIn) > 0: # loop thru data 
                        if len(dataIn) < 4:  #If whole length not received then break out of loop
                            #print "<4 chrs received"
                            dataPrevious = dataIn # store data and tag it onto next data read
                            break
                        sizeInfo = dataIn[0:4]
                        size = struct.unpack(">L", sizeInfo)[0] # get size of Scratch msg
                        #print "size:", size
                        if size > 0:
                            #print dataIn[4:size + 4]
                            #dataOut = dataOut + dataIn[4:size + 4].lower() + " "
                            dataMsg = dataIn[4:size + 4].lower() # turn msg into lower case
                            #print "msg:",dataMsg
                            if len(dataMsg) < size: # if msg recieved is too small
                                #print "half msg found"
                                #print size, len(dataMsg)
                                dataPrevious = dataIn # store data and tag it onto next data read
                                break
                            if len(dataMsg) == size: # if msg recieved is correct
                                if "alloff" in dataMsg:
                                    allSplit =  dataMsg.find("alloff")
                                    
                                    logging.debug("Whole message:%s", dataIn)
                                    #dataPrevious = dataIn # store data and tag it onto next data read
                                    #break
                                #print "half msg found"
                                #print size, len(dataMsg)
                                dataPrevious = dataIn # store data and tag it onto next data read
                                #break

                            dataPrevious = "" # no data needs tagging to next read
                            if ("alloff" in dataMsg) or ("allon" in dataMsg):
                                dataList.append(dataMsg)
                            else:
                                if dataMsg[0:2] == "br": # removed redundant "broadcast" and "sensor-update" txt
                                    if dataPrefix == "br":
                                        dataList[-1] = dataList[-1] + " "+ dataMsg[10:]
                                    else:
                                        dataList.append(dataMsg)
                                        dataPrefix = "br"
                                else:
                                    if dataPrefix == "se":
                                        dataList[-1] = dataList[-1] + dataMsg[10:]
                                    else:
                                        dataList.append(dataMsg)
                                        dataPrefix = "se"
                              
                                                                                            
                            dataIn = dataIn[size+4:] # cut data down that's been processed
                            
                    #print "previous:", dataPrevious
                   

                
                #print 'Cycle trace' , cycle_trace
                if len(data) == 0:
                    #This is due to client disconnecting or user loading new Scratch program so temp disconnect
                    #I'd like the program to retry connecting to the client
                    #tell outer loop that Scratch has disconnected
                    if cycle_trace == 'running':
                        cycle_trace = 'disconnected'
                        print "cycle_trace has changed to" ,cycle_trace
                        break

            except (KeyboardInterrupt, SystemExit):
                print "reraise error"
                raise
            except socket.timeout:
                #print "No data received: socket timeout"
                continue
            except:
                print "Unknown error occured with receiving data"
                #raise
                continue

            #At this point dataList[] contains a series of strings either broadcast or sensor-updates
            #print "data being processed:" , dataraw
            #This section is only enabled if flag set - I am in 2 minds as to whether to use it or not!
            #if (firstRun == True) or (anyAddOns == False):
            #print 
            logging.debug("dataList: %s",dataList)
            #print "GPIOPLus" , GPIOPlus
            for dataItem in dataList:
                dataraw = ' '.join([item.replace(' ','') for item in shlex.split(dataItem)]) 
                dataraw = " "+dataraw + " "
                self.dataraw = dataraw
                #print "Loop processing"
                #print self.dataraw
                #print
                if 'sensor-update' in self.dataraw:
                    #print "this data ignored" , dataraw
                    firstRunData = self.dataraw
                    #dataraw = ''
                    #firstRun = False
                    if self.vFindValue("autostart"):
                        if self.value == "true":
                            self.send_scratch_command("broadcast Scratch-StartClicked")
                            
                    if self.vFindValue("sghdebug"):
                        if (self.value == "1") and (debugLogging == False):
                            logging.getLogger().setLevel(logging.DEBUG)
                            debugLogging = True
                        if (self.value == "0") and (debugLogging == True):
                            logging.getLogger().setLevel(logging.INFO)
                            debugLogging = False                            
                            
                    if (debugLogging == False):
                         logging.getLogger().setLevel(logging.INFO)
                        
                            
                    if self.vFindValue("bright"):
                        sghGC.ledDim = int(self.valueNumeric) if self.valueIsNumeric else 100
                        PiGlow_Brightness = sghGC.ledDim
                        print sghGC.ledDim
                        
                    pinsoraddon = None
                    if self.vFindValue("setpins"):
                        setupValue = self.value
                        pinsoraddon = "pins"
                    if self.vFindValue("addon"):
                        setupValue = self.value                    
                        pinsoraddon = "addon"
                        
                                                
                        
                    if pinsoraddon != None:
                        ADDON = setupValue
                        print (ADDON, " declared")
                        
                        if "low" in ADDON:
                            with lock:
                                print "set pins to input with pulldown low"
                                for pin in sghGC.validPins:
                                    sghGC.pinUse[pin] = sghGC.PINPUTDOWN
                                sghGC.pinUse[3] = sghGC.PUNUSED
                                sghGC.pinUse[5] = sghGC.PUNUSED
                                sghGC.setPinMode()
                                anyAddOns = True
                        if "high" in ADDON:
                            with lock:
                                print "set pins to input"
                                for pin in sghGC.validPins:
                                    sghGC.pinUse[pin] = sghGC.PINPUT
                                sghGC.pinUse[3] = sghGC.PUNUSED
                                sghGC.pinUse[5] = sghGC.PUNUSED
                                sghGC.setPinMode()
                                anyAddOns = True        
                        if  "none" in ADDON:
                            with lock:
                                print "set pins to input"
                                for pin in sghGC.validPins:
                                    sghGC.pinUse[pin] = sghGC.PINPUTNONE
                                sghGC.pinUse[3] = sghGC.PUNUSED
                                sghGC.pinUse[5] = sghGC.PUNUSED
                                sghGC.setPinMode()
                                anyAddOns = True     
                        if  "normal" in ADDON:
                            with lock:
                                sghGC.pinUse[11] = sghGC.POUTPUT 
                                sghGC.pinUse[12] = sghGC.POUTPUT 
                                sghGC.pinUse[13] = sghGC.POUTPUT 
                                sghGC.pinUse[15] = sghGC.POUTPUT 
                                sghGC.pinUse[16] = sghGC.POUTPUT 
                                sghGC.pinUse[18] = sghGC.POUTPUT 
                                sghGC.pinUse[22] = sghGC.PINPUT 
                                sghGC.pinUse[7] = sghGC.PINPUT 
             
                                sghGC.setPinMode()
                                anyAddOns = True 
                                
                        if "ladder" in ADDON:
                            with lock:
                                sghGC.resetPinMode()
                                ladderOutputs = [11,12,13,15,16,18,22, 7, 5, 3]
                                for pin in ladderOutputs:
                                    sghGC.pinUse[pin] = sghGC.POUTPUT
                                for pin in [24,26,19,21]:
                                    sghGC.pinUse[pin] = sghGC.PINPUT
                                sghGC.setPinMode()
                                anyAddOns = True
                                
                        if "motorpitx" in ADDON:
                            with lock:
                                sghGC.resetPinMode()
                                sghGC.pinUse[11] = sghGC.POUTPUT #Out2 
                                sghGC.pinUse[15] = sghGC.POUTPUT #Out1
                                sghGC.pinUse[16] = sghGC.POUTPUT #Motor2 B
                                sghGC.pinUse[18] = sghGC.POUTPUT #Motor2 A
                                sghGC.pinUse[19] = sghGC.POUTPUT #Motor1
                                sghGC.pinUse[21] = sghGC.POUTPUT #Motor1
                                sghGC.pinUse[22] = sghGC.POUTPUT #Motr 2 Enable
                                sghGC.pinUse[23] = sghGC.POUTPUT #Motor1 Enable
                                
                                sghGC.pinUse[13] = sghGC.PINPUT #Motor1 Enable
                                sghGC.pinUse[7]  = sghGC.PINPUT #Motor1 Enable

                                sghGC.setPinMode()
                                sghGC.startServod([12,10]) # servos
                                print "MotorPiTx setup"
                                anyAddOns = True

                        if "piglow" in ADDON:        
                            with lock:
                                sghGC.resetPinMode()
                                PiGlow_Values = [0] * 18
                                PiGlow_Lookup = [0,1,2,3,14,12,17,16,15,13,11,10,6,7,8,5,4,9]
                                PiGlow_Brightness = 255  
                                anyAddOns = True

                        if "gpio" in ADDON:

                            with lock:
                                print sghGC.pinUse 
                                sghGC.resetPinMode()
                                print sghGC.pinUse 
                                sghGC.pinUse[11] = sghGC.POUTPUT
                                sghGC.pinUse[12] = sghGC.POUTPUT
                                sghGC.pinUse[13] = sghGC.POUTPUT
                                sghGC.pinUse[15] = sghGC.POUTPUT                                
                                sghGC.pinUse[16] = sghGC.POUTPUT
                                sghGC.pinUse[18] = sghGC.POUTPUT
                                sghGC.pinUse[7]  = sghGC.PINPUT
                                sghGC.pinUse[8]  = sghGC.PINPUT
                                sghGC.pinUse[10] = sghGC.PINPUT
                                sghGC.pinUse[22] = sghGC.PINPUT                                 
                                sghGC.setPinMode()
                                print  "gPiO setup"
                                print sghGC.pinUse 
                                anyAddOns = True
                                                           
                        if "berry" in ADDON:
                            with lock:
                                sghGC.resetPinMode()
                                berryOutputs = [7,11,15,19,21,23,24]
                                for pin in berryOutputs:
                                    sghGC.pinUse[pin] = sghGC.POUTPUT
                                sghGC.pinUse[26] = sghGC.PINPUT
                                sghGC.pinUse[22] = sghGC.PINPUT

                                sghGC.setPinMode()
                                anyAddOns = True
                            
                        if "pirocon" in ADDON:
                            with lock:
                                sghGC.resetPinMode()
                                sghGC.pinUse[19] = sghGC.POUTPUT #MotorA 
                                sghGC.pinUse[21] = sghGC.POUTPUT #MotorB (MotorA in v1.2)
                                sghGC.pinUse[26] = sghGC.POUTPUT #MotorA (MotorB in V1.2)
                                sghGC.pinUse[24] = sghGC.POUTPUT #MotorB
                                sghGC.pinUse[7]  = sghGC.PINPUT #ObsLeft
                                sghGC.pinUse[11] = sghGC.PINPUT #ObsRight
                                sghGC.pinUse[12] = sghGC.PINPUT #LFLeft
                                sghGC.pinUse[13] = sghGC.PINPUT #LFRight
                                
                                if "encoders" in ADDON:
                                    logging.debug("Encoders Found:%s", ADDON)
                                    sghGC.pinUse[7]  = sghGC.PCOUNT 
                                    sghGC.pinUse[11] = sghGC.PCOUNT 
                                    self.send_scratch_command('sensor-update "encoder" "stopped"') 
                                    self.send_scratch_command('sensor-update "count7" "0"') 
                                sghGC.setPinMode()
                                sghGC.startServod([18,22]) # servos orig
                                #sghGC.startServod([12,10]) # servos testing motorpitx

                                print "pirocon setup"
                                anyAddOns = True
                            
                        if "piringo" in ADDON:
                            with lock:
                                sghGC.resetPinMode()
                                print "piringo detected"
                                sghGC.INVERT = True # GPIO pull down each led so need to invert 0 to 1 and vice versa
                                piringoOutputs = [7,11,12,13,15,16,18,22, 24, 26, 8,10] # these are pins used for LEDS for PiRingo
                                piringoInputs = [19,21] # These are the pins connected to the switches
                                for pin in piringoOutputs:
                                    sghGC.pinUse[pin] = sghGC.POUTPUT # set leds as outputs
                                for pin in piringoInputs:
                                    sghGC.pinUse[pin] = sghGC.PINPUT # set switches as inputs
                                sghGC.setPinMode() # execute pin assignment
                                anyAddOns = True # add on declared
                            
                        if "pibrella" in ADDON:
                            with lock:
                                sghGC.resetPinMode()
                                pibrellaOutputs = [7,11,13,15,16,18,22,12]
                                for pin in pibrellaOutputs:
                                    sghGC.pinUse[pin] = sghGC.POUTPUT
                                pibrellaInputs = [21,26,24,19,23]
                                for pin in pibrellaInputs:
                                    sghGC.pinUse[pin] = sghGC.PINPUTDOWN

                                sghGC.setPinMode()
                                anyAddOns = True      

                        if "rgbled" in ADDON:
                            with lock:
                                sghGC.resetPinMode()
                                rgbOutputs = [12,16,18,22,7,11,13,15]
                                for pin in rgbOutputs:
                                    sghGC.pinUse[pin] = sghGC.POUTPUT

                                sghGC.setPinMode()
                                anyAddOns = True
                        
                        if "rtkmotorcon" in ADDON:
                            with lock:
                                sghGC.resetPinMode()
                                sghGC.pinUse[11] = sghGC.POUTPUT #Motor1 
                                sghGC.pinUse[12] = sghGC.POUTPUT #Motor1
                                sghGC.pinUse[15] = sghGC.POUTPUT #Motor2 
                                sghGC.pinUse[16] = sghGC.POUTPUT #Motor2

                                sghGC.setPinMode()
                                print "rtkmotorcon setup"
                                anyAddOns = True
                                
                        if "pidie" in ADDON:
                            print "pidie detected"
                            sghGC.INVERT = True # GPIO pull down each led so need to invert 0 to 1 and vice versa
                            with lock:
                                sghGC.resetPinMode()
                                pidieOutputs = [7,11,12,13,15,16,18,22,8]
                                for pin in pidieOutputs:
                                    sghGC.pinUse[pin] = sghGC.POUTPUT
                                pidieInputs = [21,19,24,26]
                                for pin in pidieInputs:
                                    sghGC.pinUse[pin] = sghGC.PINPUT

                                sghGC.setPinMode()
                                anyAddOns = True 
                            
                                
                        if "fishdish" in ADDON:
                            with lock:
                                sghGC.resetPinMode()
                                fishOutputs = [7,15,21,24]
                                for pin in fishOutputs:
                                    sghGC.pinUse[pin] = sghGC.POUTPUT
                                sghGC.pinUse[26] = sghGC.PINPUT

                                sghGC.setPinMode()
                                anyAddOns = True
                                
                        if "pi2go" in ADDON:
                            with lock:
                                sghGC.resetPinMode()
                                sghGC.pinUse[19] = sghGC.POUTPUT #MotorA 
                                sghGC.pinUse[21] = sghGC.POUTPUT #MotorA
                                sghGC.pinUse[26] = sghGC.POUTPUT #MotorB
                                sghGC.pinUse[24] = sghGC.POUTPUT #MotorB
                                sghGC.pinUse[7]  = sghGC.PINPUT #ObjLeft
                                sghGC.pinUse[11] = sghGC.PINPUT #ObjRight
                                sghGC.pinUse[15] = sghGC.PINPUT #ObjMid                                
                                sghGC.pinUse[12] = sghGC.PINPUT #LFLeft
                                sghGC.pinUse[13] = sghGC.PINPUT #LFRight                                
                                sghGC.pinUse[16]  = sghGC.PINPUT 
                                sghGC.pinUse[18]  = sghGC.PINPUT        
                                sghGC.pinUse[22]  = sghGC.PINPUT 
                                
                                sghGC.setPinMode()

                                #sghGC.startServod([12,10]) # servos testing motorpitx

                                print "pi2go setup"
                                anyAddOns = True
                        if "happi" in ADDON:
                            with lock:
                                sghGC.resetPinMode()
                                sghGC.pinUse[11] = sghGC.POUTPUT #Motor1 
                                sghGC.pinUse[12] = sghGC.POUTPUT #Motor1
                                sghGC.pinUse[15] = sghGC.POUTPUT #Motor2
                                sghGC.pinUse[16] = sghGC.POUTPUT #Motor2
                                sghGC.pinUse[3]  = sghGC.PINPUT 
                                sghGC.pinUse[5] = sghGC.PINPUT 
                                sghGC.pinUse[7] = sghGC.PINPUT                               
                                sghGC.pinUse[8] = sghGC.PINPUT 
                                sghGC.pinUse[10] = sghGC.PINPUT                          
                                sghGC.pinUse[13]  = sghGC.PINPUT 
                                sghGC.pinUse[18]  = sghGC.PINPUT        
                                sghGC.pinUse[19]  = sghGC.PINPUT 
                                sghGC.pinUse[21]  = sghGC.PINPUT 
                                sghGC.pinUse[22]  = sghGC.PINPUT 
                                sghGC.pinUse[23]  = sghGC.PINPUT 
                                sghGC.pinUse[24]  = sghGC.PINPUT 
                                sghGC.pinUse[26]  = sghGC.PINPUT 
                                
                                sghGC.setPinMode()

                                #sghGC.startServod([12,10]) # servos testing motorpitx

                                print "HapPi setup"
                                anyAddOns = True                                
                                

                # if (firstRun == True) and (anyAddOns == False): # if no addon found in firstrun then assume default configuration
                    # with lock:
                        # print "no AddOns Declared"
                        # sghGC.pinUse[11] = sghGC.POUTPUT
                        # sghGC.pinUse[12] = sghGC.POUTPUT
                        # sghGC.pinUse[13] = sghGC.POUTPUT
                        # sghGC.pinUse[15] = sghGC.POUTPUT
                        # sghGC.pinUse[16] = sghGC.POUTPUT
                        # sghGC.pinUse[18] = sghGC.POUTPUT
                        # sghGC.pinUse[7]  = sghGC.PINPUT
                        # sghGC.pinUse[22] = sghGC.PINPUT
                        # sghGC.setPinMode()
                                
                        # firstRun = False


                #If outputs need globally inverting (7 segment common anode needs it - PiRingo etc)
                if ('invert' in self.dataraw):
                    sghGC.INVERT = True
                    
                #Change pins from input to output if more needed
                if self.bFind('config'):
                    with lock:
                        for pin in sghGC.validPins:
                            #print "checking pin" ,pin
                            if self.bFindValue('config' + str(pin)):
                                if self.value == "in":
                                    sghGC.pinUse[pin] = sghGC.PINPUT                            
                                # if self.value == "inpulldown":
                                    # sghGC.pinUse[pin] = sghGC.PINPUTDOWN                            
                                # if self.value == "inpullnone":
                                    # sghGC.pinUse[pin] = sghGC.PINPUTNONE
                                # else:
                                    # sghGC.pinUse[pin] = sghGC.POUTPUT
                    
                        sghGC.setPinMode()           
    ### Check for AddOn boards being declared
                    
                #Listen for Variable changes
                if 'sensor-update' in self.dataraw:

                    #print "sensor-update rcvd" , dataraw
                               
                  
                    if "ladder" in ADDON:
                        #do ladderboard stuff

                        self.vAllCheck("leds") # check All LEDS On/Off/High/Low/1/0

                        self.vLEDCheck(ladderOutputs)
                                        
                    elif "motorpitx" in ADDON:
                        #do MotorPiTx stuff
                        #check for motor variable commands
                        
                        moveServos = False

                        if self.vFindValue('tiltoffset'):
                            tiltoffset = int(self.valueNumeric) if self.valueIsNumeric else 0
                            moveServos = True

                        if self.vFindValue('panoffset'):
                            panoffset = int(self.valueNumeric) if self.valueIsNumeric else 0
                            moveServos = True
                            
                        if self.vFindValue('tilt'):
                            #print "tilt command rcvd"
                            if self.valueIsNumeric:
                                tilt = int(self.valueNumeric) 
                                moveServos = True
                                #print "tilt=", tilt
                            elif self.value == "off":
                                os.system("echo " + "0" + "=0 > /dev/servoblaster")
                        else:
                            if self.vFindValue('servo1'):
                                #print "tilt command rcvd"
                                if self.valueIsNumeric:
                                    tilt = int(self.valueNumeric) 
                                    moveServos = True
                                    #print "tilt=", tilt
                                elif self.value == "off":
                                    sghGC.pinServod(12,"off")
                                    
                        if self.vFindValue('pan'):
                            #print "pan command rcvd"
                            if self.valueIsNumeric:
                                pan = int(self.valueNumeric) 
                                moveServos = True
                                #print "pan=", pan
                            elif self.value == "off":
                                os.system("echo " + "1" + "=0 > /dev/servoblaster")
                        else:
                            if self.vFindValue('servo2'):
                                #print "servob command rcvd"
                                if self.valueIsNumeric:
                                    pan = int(self.valueNumeric) 
                                    moveServos = True
                                    #print "servob=", pan
                                elif self.value == "off":
                                    sghGC.pinServod(10,"off")
                       
                        if moveServos == True:
                            #print "move servos == True"
                            degrees = int(tilt + tiltoffset)
                            degrees = min(90,max(degrees,-90))
                            servodvalue = 50+ ((90 - degrees) * 200 / 180)
                            sghGC.pinServod(12,servodvalue)
                            degrees = int(pan + panoffset)
                            degrees = min(90,max(degrees,-90))
                            servodvalue = 50+ ((90 - degrees) * 200 / 180)
                            #print "Value being sent to pin 10:",servodvalue
                            sghGC.pinServod(10,servodvalue)


                        #check for motor variable commands
                        motorList = [['motor1',19,21,23],['motor2',18,16,22]]
                        for listLoop in range(0,2):
                            if self.vFindValue(motorList[listLoop][0]):
                                svalue = int(self.valueNumeric) if self.valueIsNumeric else 0
                                # This technique can be used if enabel is held high by hardware
                                if svalue > 0:
                                    sghGC.pinUpdate(motorList[listLoop][1],(svalue),"pwm")
                                    sghGC.pinUpdate(motorList[listLoop][2],0)
                                    sghGC.pinUpdate(motorList[listLoop][3],1)# set enable to 1
                                elif svalue < 0:
                                    sghGC.pinUpdate(motorList[listLoop][1],0)            
                                    sghGC.pinUpdate(motorList[listLoop][2],(svalue),"pwm")   
                                    sghGC.pinUpdate(motorList[listLoop][3],1) # set enable to 1
                                else:
                                    sghGC.pinUpdate(motorList[listLoop][3],0)                      
                                    sghGC.pinUpdate(motorList[listLoop][1],0)
                                    sghGC.pinUpdate(motorList[listLoop][2],0)
                                    
                            
                    elif (("piglow" in ADDON) and (piglow != None)):
                        #do PiGlow stuff but make sure PiGlow physically detected             
                     
                        #check LEDS
                        for i in range(1,19):
                            if self.vFindValue('led' + str(i)):
                                svalue = int(self.valueNumeric) if self.valueIsNumeric else 0
                                svalue= min(255,max(svalue,0))
                                PiGlow_Values[PiGlow_Lookup[i-1]] = svalue
                                piglow.update_pwm_values(PiGlow_Values)
                                
                        for i in range(1,4):
                            if self.vFindValue('leg' + str(i)):
                                svalue = int(self.valueNumeric) if self.valueIsNumeric else 0
                                svalue= min(255,max(svalue,0))
                                PiGlow_Values[PiGlow_Lookup[((i-1)*6) + 0]] = svalue
                                PiGlow_Values[PiGlow_Lookup[((i-1)*6) + 1]] = svalue
                                PiGlow_Values[PiGlow_Lookup[((i-1)*6) + 2]] = svalue
                                PiGlow_Values[PiGlow_Lookup[((i-1)*6) + 3]] = svalue
                                PiGlow_Values[PiGlow_Lookup[((i-1)*6) + 4]] = svalue
                                PiGlow_Values[PiGlow_Lookup[((i-1)*6) + 5]] = svalue
                                piglow.update_pwm_values(PiGlow_Values)
                                
                            if self.vFindValue('arm' + str(i)):
                                svalue = int(self.valueNumeric) if self.valueIsNumeric else 0
                                svalue= min(255,max(svalue,0))
                                PiGlow_Values[PiGlow_Lookup[((i-1)*6) + 0]] = svalue
                                PiGlow_Values[PiGlow_Lookup[((i-1)*6) + 1]] = svalue
                                PiGlow_Values[PiGlow_Lookup[((i-1)*6) + 2]] = svalue
                                PiGlow_Values[PiGlow_Lookup[((i-1)*6) + 3]] = svalue
                                PiGlow_Values[PiGlow_Lookup[((i-1)*6) + 4]] = svalue
                                PiGlow_Values[PiGlow_Lookup[((i-1)*6) + 5]] = svalue
                                piglow.update_pwm_values(PiGlow_Values)
                                
                        pcolours = ['red','orange','yellow','green','blue','white']
                        for i in range(len(pcolours)):
                            if self.vFindValue(pcolours[i]):
                                svalue = int(self.valueNumeric) if self.valueIsNumeric else 0
                                svalue= min(255,max(svalue,0))
                                PiGlow_Values[PiGlow_Lookup[i+0]] = svalue
                                PiGlow_Values[PiGlow_Lookup[i+6]] = svalue
                                PiGlow_Values[PiGlow_Lookup[i+12]] = svalue
                                piglow.update_pwm_values(PiGlow_Values)
                            
                                
                        #Use bit pattern to control leds
                        if self.vFindValue('ledpattern'):
                            #print 'Found ledpattern'
                            num_of_bits = 18
                            bit_pattern = ('00000000000000000000000000' + self.value)[-num_of_bits:]
                            #print 'led_pattern %s' % bit_pattern
                            j = 0
                            for i in range(18):
                            #bit_state = ((2**i) & sensor_value) >> i
                            #print 'dummy pin %d state %d' % (i, bit_state)
                                if bit_pattern[-(j+1)] == '0':
                                    PiGlow_Values[PiGlow_Lookup[i]] = 0
                                else:
                                    PiGlow_Values[PiGlow_Lookup[i]] = 1
                                j = j + 1
                            
                            piglow.update_pwm_values(PiGlow_Values)
                        
                        #Replaced by global bright variable code
                        #if self.vFindValue('bright'):
                        #    svalue = int(self.valueNumeric) if self.valueIsNumeric else 0
                        #    svalue= min(255,max(svalue,0))
                        #    PiGlow_Brightness = svalue
                            
                    elif "gpio" in ADDON:
                        #do gPiO stuff
                        
                        self.vAllCheck("allpins") # check Allpins On/Off/High/Low/1/0
     
                        self.vPinCheck() # check for any pin On/Off/High/Low/1/0 any PWM settings using power or motor
                                
                        #check for motor variable commands
                        motorList = [['motora',11,12],['motorb',13,15]]
                        #motorList = [['motora',21,26],['motorb',19,24]]
                        for listLoop in range(0,2):
                            #print motorList[listLoop]
                            checkStr = motorList[listLoop][0]
                            if self.vFind(checkStr):
                                tempValue = getValue(checkStr, dataraw)
                                svalue = int(float(tempValue)) if isNumeric(tempValue) else 0
                                #print "svalue", svalue
                                if svalue > 0:
                                    #print motorList[listLoop]
                                    #print "motor set forward" , svalue
                                    self.pinUpdate(motorList[listLoop][2],1)
                                    self.pinUpdate(motorList[listLoop][1],(100-svalue),"pwm")
                                elif svalue < 0:
                                    #print motorList[listLoop]
                                    #print "motor set backward", svalue
                                    self.pinUpdate(motorList[listLoop][2],0)
                                    self.pinUpdate(motorList[listLoop][1],(svalue),"pwm")
                                else:
                                    #print svalue, "zero"
                                    self.pinUpdate(motorList[listLoop][1],0)
                                    self.pinUpdate(motorList[listLoop][2],0)

                        ######### End of gPiO Variable handling
                       
                    elif "berry" in ADDON:
                        #do BerryClip stuff
                        self.vAllCheck("leds") # check All LEDS On/Off/High/Low/1/0

                        self.vLEDCheck(berryOutputs) # check All LEDS On/Off/High/Low/1/0
                                    
                        if self.vFindOnOff('buzzer'):
                            self.index_pin_update(24,self.valueNumeric)

                        ######### End of BerryClip Variable handling
                        
                    elif "pirocon" in ADDON:
                        #do PiRoCon stuff
                        #logging.debug("Processing variables for PiRoCon")
                        #print "panoffset" , panoffset, "tilt",tiltoffset
                        moveServos = False

                        if self.vFindValue('tiltoffset'):
                            tiltoffset = int(self.valueNumeric) if self.valueIsNumeric else 0
                            moveServos = True

                        if self.vFindValue('panoffset'):
                            panoffset = int(self.valueNumeric) if self.valueIsNumeric else 0
                            moveServos = True
                            
                        if self.vFindValue('tilt'):
                            #print "tilt command rcvd"
                            if self.valueIsNumeric:
                                tilt = int(self.valueNumeric) 
                                moveServos = True
                                #print "tilt=", tilt
                            elif self.value == "off":
                                os.system("echo " + "0" + "=0 > /dev/servoblaster")
                        else:
                            if self.vFindValue('servoa'):
                                #print "tilt command rcvd"
                                if self.valueIsNumeric:
                                    tilt = int(self.valueNumeric) 
                                    moveServos = True
                                    #print "tilt=", tilt
                                elif self.value == "off":
                                    os.system("echo " + "0" + "=0 > /dev/servoblaster")
                                    
                        if self.vFindValue('pan'):
                            #print "pan command rcvd"
                            if self.valueIsNumeric:
                                pan = int(self.valueNumeric) 
                                moveServos = True
                                #print "pan=", pan
                            elif self.value == "off":
                                os.system("echo " + "1" + "=0 > /dev/servoblaster")
                        else:
                            if self.vFindValue('servob'):
                                #print "pan command rcvd"
                                if self.valueIsNumeric:
                                    pan = int(self.valueNumeric) 
                                    moveServos = True
                                    #print "pan=", pan
                                elif self.value == "off":
                                    os.system("echo " + "1" + "=0 > /dev/servoblaster")
                       
                        if moveServos == True:
                            degrees = int(tilt + tiltoffset)
                            degrees = min(80,max(degrees,-60))
                            servodvalue = 50+ ((90 - degrees) * 200 / 180)
                            #print "sending", servodvalue, "to servod"
                            #os.system("echo " + "0" + "=" + str(servodvalue-1) + " > /dev/servoblaster")
                            sghGC.pinServod(18,servodvalue) # orig =18
                            #os.system("echo " + "0" + "=" + str(servodvalue) + " > /dev/servoblaster")
                            degrees = int(pan + panoffset)
                            degrees = min(90,max(degrees,-90))
                            servodvalue = 50+ ((90 - degrees) * 200 / 180)
                            sghGC.pinServod(22,servodvalue) #orig =22
                            #os.system("echo " + "1" + "=" + str(servodvalue) + " > /dev/servoblaster")


                        #check for motor variable commands
                        motorList = [['motora',21,26],['motorb',19,24]]
                        if "piroconb" in ADDON:
                            logging.debug("PiRoConB Found:%s", ADDON)
                            motorList = [['motora',21,19],['motorb',26,24]]
                        #logging.debug("ADDON:%s", ADDON)
                        for listLoop in range(0,2):
                            if self.vFindValue(motorList[listLoop][0]):
                                svalue = int(self.valueNumeric) if self.valueIsNumeric else 0
                                if svalue > 0:
                                    sghGC.pinUpdate(motorList[listLoop][2],1)
                                    sghGC.pinUpdate(motorList[listLoop][1],(100-svalue),"pwm")
                                elif svalue < 0:
                                    sghGC.pinUpdate(motorList[listLoop][2],0)
                                    sghGC.pinUpdate(motorList[listLoop][1],(svalue),"pwm")
                                else:
                                    sghGC.pinUpdate(motorList[listLoop][1],0)
                                    sghGC.pinUpdate(motorList[listLoop][2],0)
                        

                        if (pcaPWM != None):
                            for i in range(0, 16): # go thru servos on PCA Board
                                if self.vFindValue('servo' + str(i + 1)):
                                    svalue = int(self.valueNumeric) if self.valueIsNumeric else 180
                                    #print i, svalue
                                    pcaPWM.setPWM(i, 0, svalue)
                                    
                            for i in range(0, 16): # go thru PowerPWM on PCA Board
                                if self.vFindValue('power' + str(i + 1)):
                                    svalue = int(self.valueNueric) if self.valueIsNumeric else 0
                                    svalue = min(4095,max(((svalue * 4096) /100),0))
                                    pcaPWM.setPWM(i, 0, svalue)
                                    
                        ######### End of PiRoCon Variable handling
                    elif "piringo" in ADDON:
                        #do piringo stuff

                        self.vAllCheck("leds") # check All LEDS On/Off/High/Low/1/0

                        self.vLEDCheck(piringoOutputs)
                        
                        
                    elif "pibrella" in ADDON: # PiBrella
               
                        self.vAllCheck("allpins") # check All On/Off/High/Low/1/0

                        self.vListCheck([13,11,7,15,16,18,22],["led1","led2","led3","led4","led5","led6","led7"])
                        self.vListCheck([13,11,11,11,7,15,16,18,22],["red","amber","yellow","orange","green","outpute","outputf","outputg","outputh"])
                        
                        if self.vFindValue('stepper'):
                            if self.valueIsNumeric:
                                self.stepperUpdate([15,16,18,22],self.valueNumeric)
                            else:
                                self.stepperUpdate([15,16,18,22],0)                        
                                    
                        if self.vFindValue("beep"):
                            try:
                                bn,bd = self.value.split(",")
                            except:
                                bn = "60"
                                bd = "1"
                            beepNote = int(float(bn))
                            beepDuration = (float(bd))
                            svalue = int(self.valueNumeric) if self.valueIsNumeric else 60
                            beepThread = threading.Thread(target=self.beep, args=[12,440* 2**((beepNote - 69)/12.0),beepDuration])
                            beepThread.start()
                            
                        # if self.vFindValue("beepnote"):
                            # beepNote = max(12,int(self.valueNumeric)) if self.valueIsNumeric else 60
                   
                        # if self.vFindValue("beepduration"):
                            # beepDuration = max(0.125,int(self.valueNumeric)) if self.valueIsNumeric else 0.5
                           
                            
                    elif "rgbled" in ADDON: # RGB-LED by Meltwater/rsstab/tim cox
               
                         #print ("rgb-led variable processing")            
                        if self.vFindOnOff("all"):
                            for loop in range(0,5):
                                sghGC.pinUpdate(rgbOutputs[loop],1 - self.valueNumeric)
                            for loop in range(5,8):
                                sghGC.pinUpdate(rgbOutputs[loop],self.valueNumeric)

                        rgbList = rgbOutputs[0:5]
                        for listLoop in rgbList:
                            if self.vFindOnOff("led"+str(1+rgbList.index(listLoop))):
                                sghGC.pinUpdate(rgbOutputs[rgbList.index(listLoop)],1-self.valueNumeric)
                            if self.vFindValue("power"+str(1+rgbList.index(listLoop))):
                                if self.valueIsNumeric:
                                    sghGC.pinUpdate(rgbOutputs[rgbList.index(listLoop)],100-self.valueNumeric,"pwm")
                                else:
                                    sghGC.pinUpdate(rgbOutputs[rgbList.index(listLoop)],1)
                        
                        
                        rgbList = ["red","green","blue"]
                        for listLoop in rgbList:
                            if self.vFindOnOff(listLoop):
                                print listLoop , "found",
                                sghGC.pinUpdate(rgbOutputs[5+rgbList.index(listLoop)],self.valueNumeric)
                                                            
                    elif "rtkmotorcon" in ADDON:  
                        #check for motor variable commands
                        motorList = [['motor1',11,12],['motor2',15,16]]
                        for listLoop in range(0,2):
                            if self.vFindValue(motorList[listLoop][0]):
                                svalue = int(self.valueNumeric) if self.valueIsNumeric else 0
                                if svalue > 0:
                                    sghGC.pinUpdate(motorList[listLoop][2],1)
                                    sghGC.pinUpdate(motorList[listLoop][1],(100-svalue),"pwm")
                                elif svalue < 0:
                                    sghGC.pinUpdate(motorList[listLoop][2],0)
                                    sghGC.pinUpdate(motorList[listLoop][1],(svalue),"pwm")
                                else:
                                    sghGC.pinUpdate(motorList[listLoop][1],0)
                                    sghGC.pinUpdate(motorList[listLoop][2],0)
                                    
                    elif "pidie" in ADDON:
                        self.vAllCheck("leds") # check All LEDS On/Off/High/Low/1/0
                        self.vListCheck([7,11,12,13,15,16,18,22,8],["led1","led2","led3","led4","led5","led6","led7","led8","led9"])
                        self.vListCheck([7,11,12,13,15,16,18,22,8],["1","2","3","4","5","6","7","8","9"])     
                        
                    elif "fishdish" in ADDON:
                        #do fishdish stuff
                        self.vAllCheck("leds") # check All LEDS On/Off/High/Low/1/0

                        self.vLEDCheck(fishOutputs) # check All LEDS On/Off/High/Low/1/0
                                    
                        if self.vFindOnOff('buzzer'):
                            self.index_pin_update(24,self.valueNumeric)
                            
                    elif "pi2go" in ADDON:
                        #do PiRoCon stuff
                        logging.debug("Processing variables for Pi2Go")

                        #check for motor variable commands
                        motorList = [['motorb',21,19],['motora',26,24]]
                        logging.debug("ADDON:%s", ADDON)
                        for listLoop in range(0,2):
                            if self.vFindValue(motorList[listLoop][0]):
                                svalue = int(self.valueNumeric) if self.valueIsNumeric else 0
                                logging.debug("svalue %s %s", motorList[listLoop][0],svalue)
                                if svalue > 0:
                                    sghGC.pinUpdate(motorList[listLoop][2],1)
                                    sghGC.pinUpdate(motorList[listLoop][1],(100-svalue),"pwm")
                                elif svalue < 0:
                                    sghGC.pinUpdate(motorList[listLoop][2],0)
                                    sghGC.pinUpdate(motorList[listLoop][1],(svalue),"pwm")
                                else:
                                    sghGC.pinUpdate(motorList[listLoop][1],0)
                                    sghGC.pinUpdate(motorList[listLoop][2],0)
                                    
                    elif "happi" in ADDON:
                        #do happi stuff
                        logging.debug("Processing variables for HapPi")

                        #check for motor variable commands
                        motorList = [['motor1',11,12],['motor2',15,16]]
                        logging.debug("ADDON:%s", ADDON)
                        for listLoop in range(0,2):
                            if self.vFindValue(motorList[listLoop][0]):
                                svalue = int(self.valueNumeric) if self.valueIsNumeric else 0
                                logging.debug("svalue %s %s", motorList[listLoop][0],svalue)
                                if svalue > 0:
                                    sghGC.pinUpdate(motorList[listLoop][2],1)
                                    sghGC.pinUpdate(motorList[listLoop][1],(100-svalue),"pwm")
                                elif svalue < 0:
                                    sghGC.pinUpdate(motorList[listLoop][2],0)
                                    sghGC.pinUpdate(motorList[listLoop][1],(svalue),"pwm")
                                else:
                                    sghGC.pinUpdate(motorList[listLoop][1],0)
                                    sghGC.pinUpdate(motorList[listLoop][2],0)                                    
                                    
                    else:   #normal variable processing with no add on board
                        
                        self.vAllCheck("allpins") # check All On/Off/High/Low/1/0
     
                        self.vPinCheck() # check for any pin On/Off/High/Low/1/0 any PWM settings using power or motor
                        #logging.debug("Steppers in use")
                        if steppersInUse == True:
                            #logging.debug("Steppers in use")
                            stepperList = [['motora',[11,12,13,15]],['motorb',[16,18,22,7]]]
                            for listLoop in range(0,2):
                                if self.vFindValue(stepperList[listLoop][0]):
                                    logging.debug("Stepper found %s",stepperList[listLoop][0])
                                    if self.valueIsNumeric:
                                        self.stepperUpdate(stepperList[listLoop][1],self.valueNumeric)
                                    else:
                                        self.stepperUpdate(stepperList[listLoop][1],0)
                                     
                            stepperList = [['positiona',[11,12,13,15]],['positionb',[16,18,22,7]]]
                            for listLoop in range(0,2):
                                #print ("look for steppers") 
                                if self.vFindValue(stepperList[listLoop][0]):
                                    print ("Found stepper",stepperList[listLoop][0])
                                    if self.valueIsNumeric:
                                        print ("value =",self.value)
                                        print stepperList[listLoop][1][0]
                                        try:
                                            print ("Trying to see if turn prev set")
                                            direction = int(100 * sign(int(self.valueNumeric) - turn[stepperList[listLoop][1][0]]))
                                            steps = abs(int(self.valueNumeric) - turn[stepperList[listLoop][1][0]])
                                        except:
                                            direction = int(100 * sign(int(self.valueNumeric)))
                                            steps = abs(int(self.valueNumeric))
                                            turn = [None] * sghGC.numOfPins
                                            pass
                                        print ("direction and steps",direction,steps)
                                        self.stepperUpdate(stepperList[listLoop][1],direction,steps)
                                        turn[stepperList[listLoop][1][0]] = self.valueNumeric
                                        print ("position set to :",turn[stepperList[listLoop][1][0]])
                                    else:
                                        self.stepperUpdate(stepperList[listLoop][1],0)
                                        try:
                                            turn[stepperList[listLoop][1][0]] = 0
                                        except:
                                            turn = [None] * sghGC.numOfPins
                                            turn[stepperList[listLoop][1][0]] = 0
                                            pass
                        else:       
                            motorList = [['motora',11],['motorb',12]]
                            for listLoop in range(0,2):
                                if self.vFindValue(motorList[listLoop][0]):
                                    if self.valueIsNumeric:
                                        sghGC.pinUpdate(motorList[listLoop][1],self.valueNumeric,type="pwm")
                                    else:
                                        sghGC.pinUpdate(motorList[listLoop][1],0,type="pwm")
                                                
                    #Use bit pattern to control ports
                    if self.vFindValue('pinpattern'):
                        svalue = self.value 
                        bit_pattern = ('00000000000000000000000000'+svalue)[-sghGC.numOfPins:]
                        j = 0
                        onSense = '1' if sghGC.INVERT else '0' # change to look for 0 if invert on
                        onSense = '0'
                        for pin in sghGC.validPins:
                            if (sghGC.pinUse[pin] == sghGC.POUTPUT):
                                #print "pin" , bit_pattern[-(j+1)]
                                if bit_pattern[-(j+1)] == onSense:
                                    sghGC.pinUpdate(pin,0)
                                else:
                                    sghGC.pinUpdate(pin,1)
                                j = j + 1                   

                    checkStr = 'stepdelay'
                    if  (checkStr + ' ') in dataraw:
                        #print "MotorA Received"
                        #print "stepper status" , stepperInUse[STEPPERA]
                        tempValue = getValue(checkStr, dataraw)
                        if isNumeric(tempValue):
                            step_delay = int(float(tempValue))
                            print 'step delay changed to', step_delay
                            
                    
                    if pcfSensor != None: #if PCF ADC found
                        if self.vFindValue('dac'):
                            svalue = int(self.valueNumeric) if self.valueIsNumeric else 0
                            pcfSensor.writeDAC(svalue)

    ### Check for Broadcast type messages being received

                if 'broadcast' in self.dataraw:
                    #print 'broadcast:' , self.dataraw

                    if self.bFindValue("setpins"):
                        logging.debug("SetPins broadcast found")
                        logging.debug("SetPins value len %d",len(self.value))
                        logging.debug("SetPins value %s",self.value)
                        
                        if len(self.value) == 0:
                            with lock:
                                
                                for pin in sghGC.validPins:
                                    sghGC.pinUse[pin] = sghGC.PINPUTDOWN
                                sghGC.pinUse[11] = sghGC.POUTPUT
                                sghGC.pinUse[12] = sghGC.POUTPUT
                                sghGC.pinUse[13] = sghGC.POUTPUT
                                sghGC.pinUse[15] = sghGC.POUTPUT
                                sghGC.pinUse[16] = sghGC.POUTPUT
                                sghGC.pinUse[18] = sghGC.POUTPUT
                                #sghGC.pinUse[22] = sghGC.PINPUT
                                #sghGC.pinUse[7] = sghGC.PINPUT
                                sghGC.setPinMode()
                                anyAddOns = True
                        
                        elif self.value == "low":
                            with lock:
                                print "set pins to input with pulldown low"
                                for pin in sghGC.validPins:
                                    sghGC.pinUse[pin] = sghGC.PINPUTDOWN
                                #sghGC.pinUse[3] = sghGC.PUNUSED
                                #sghGC.pinUse[5] = sghGC.PUNUSED
                                sghGC.setPinMode()
                                anyAddOns = True
                                
                        elif self.value == "high":
                            with lock:
                                print "set pins to input with pull ups"
                                for pin in sghGC.validPins:
                                    sghGC.pinUse[pin] = sghGC.PINPUT
                                #sghGC.pinUse[3] = sghGC.PUNUSED
                                #sghGC.pinUse[5] = sghGC.PUNUSED
                                sghGC.setPinMode()
                                anyAddOns = True     
                                
                        elif self.value == "none":
                            with lock:
                                print "set pins to input with no pullups"
                                for pin in sghGC.validPins:
                                    sghGC.pinUse[pin] = sghGC.PINPUTNONE
                                #sghGC.pinUse[3] = sghGC.PUNUSED
                                #sghGC.pinUse[5] = sghGC.PUNUSED
                                sghGC.setPinMode()
                                anyAddOns = True                        
                                
                    if self.bFindOnOff("sghdebug"):
                        if (self.OnOrOff == True) and (debugLogging == False):
                            logging.getLogger().setLevel(logging.DEBUG)
                            debugLogging = True
                        if (self.OnOrOff == False) and (debugLogging == True):
                            logging.getLogger().setLevel(logging.INFO)
                            debugLogging = False                            
                            
                    if (debugLogging == False):
                         logging.getLogger().setLevel(logging.INFO)                                
                    
                    if self.bFindValue("bright"):
                        sghGC.ledDim = int(self.valueNumeric) if self.valueIsNumeric else 100
                        PiGlow_Brightness = sghGC.ledDim
                        print sghGC.ledDim
                        
                    #self.send_scratch_command("broadcast Begin")
                    if self.bFind("stepper"):
                        print ("Stepper declared")
                        steppersInUse = True
                        sghGC.pinUse[11] = sghGC.POUTPUT
                        sghGC.pinUse[12] = sghGC.POUTPUT
                        sghGC.pinUse[13] = sghGC.POUTPUT
                        sghGC.pinUse[15] = sghGC.POUTPUT
                        sghGC.pinUse[16] = sghGC.POUTPUT
                        sghGC.pinUse[18] = sghGC.POUTPUT
                        sghGC.pinUse[22] = sghGC.POUTPUT
                        sghGC.pinUse[7]  = sghGC.POUTPUT
                        sghGC.setPinMode()

                    if "ladder" in ADDON: # Gordon's Ladder Board
                        #do ladderboard stuff
                        #print ("Ladder broadcast processing")                    
                        self.bCheckAll() # Check for all off/on type broadcasrs
                        self.bLEDCheck(ladderOutputs) # Check for LED off/on type broadcasts
                                
                    elif "motorpitx" in ADDON: # Boeeerb MotorPiTx

                        if ('sonar1') in dataraw:
                            distance = sghGC.pinSonar(13)
                            #print'Distance:',distance,'cm'
                            sensor_name = 'sonar' + str(13)
                            bcast_str = 'sensor-update "%s" %d' % (sensor_name, distance)
                            #print 'sending: %s' % bcast_str
                            self.send_scratch_command(bcast_str)
                            
                        if ('sonar2') in dataraw:
                            distance = sghGC.pinSonar(7)
                            #print'Distance:',distance,'cm'
                            sensor_name = 'sonar' + str(7)
                            bcast_str = 'sensor-update "%s" %d' % (sensor_name, distance)
                            #print 'sending: %s' % bcast_str
                            self.send_scratch_command(bcast_str)                        
                            
                        if self.bFind('ultra1'):
                            print 'start pinging on', str(13)
                            sghGC.pinUse[13] = sghGC.PULTRA
                            
                        if self.bFind('ultra2'):
                            print 'start pinging on', str(7)
                            sghGC.pinUse[7] = sghGC.PULTRA
                            
                    elif (("piglow" in ADDON) and (piglow != None)): # Pimoroni PiGlow
                        #print "processing piglow variables"
                    
                        if self.bFindOnOff('all'):
                            #print "found allon/off"
                            for i in range(1,19):
                                #print i
                                PiGlow_Values[i-1] = PiGlow_Brightness * self.OnOrOff
                                #print "Values", PiGlow_Values
                                piglow.update_pwm_values(PiGlow_Values)
                                 
                        #check LEDS
                        for i in range(1,19):
                            #check_broadcast = str(i) + 'on'
                            #print check_broadcast
                            if self.bFindOnOff('led'+str(i)):
                                #print dataraw
                                PiGlow_Values[PiGlow_Lookup[i-1]] = PiGlow_Brightness * self.OnOrOff
                                piglow.update_pwm_values(PiGlow_Values)

                            if self.bFindOnOff('light'+str(i)):
                                #print dataraw
                                PiGlow_Values[PiGlow_Lookup[i-1]] = PiGlow_Brightness * self.OnOrOff
                                piglow.update_pwm_values(PiGlow_Values)
                                
                        pcolours = ['red','orange','yellow','green','blue','white']
                        for i in range(len(pcolours)):
                            if self.bFindOnOff(pcolours[i]):
                                #print dataraw
                                PiGlow_Values[PiGlow_Lookup[i+0]] = PiGlow_Brightness * self.OnOrOff
                                PiGlow_Values[PiGlow_Lookup[i+6]] = PiGlow_Brightness * self.OnOrOff
                                PiGlow_Values[PiGlow_Lookup[i+12]] = PiGlow_Brightness * self.OnOrOff
                                piglow.update_pwm_values(PiGlow_Values)
                                                           
                        for i in range(1,4):
                            if self.bFindOnOff('leg'+str(i)):
                                #print dataraw
                                PiGlow_Values[PiGlow_Lookup[((i-1)*6) + 0]] = PiGlow_Brightness * self.OnOrOff
                                PiGlow_Values[PiGlow_Lookup[((i-1)*6) + 1]] = PiGlow_Brightness * self.OnOrOff
                                PiGlow_Values[PiGlow_Lookup[((i-1)*6) + 2]] = PiGlow_Brightness * self.OnOrOff
                                PiGlow_Values[PiGlow_Lookup[((i-1)*6) + 3]] = PiGlow_Brightness * self.OnOrOff
                                PiGlow_Values[PiGlow_Lookup[((i-1)*6) + 4]] = PiGlow_Brightness * self.OnOrOff
                                PiGlow_Values[PiGlow_Lookup[((i-1)*6) + 5]] = PiGlow_Brightness * self.OnOrOff
                                piglow.update_pwm_values(PiGlow_Values)
                                
                            if self.bFindOnOff('arm'+str(i)):
                                #print dataraw
                                PiGlow_Values[PiGlow_Lookup[((i-1)*6) + 0]] = PiGlow_Brightness * self.OnOrOff
                                PiGlow_Values[PiGlow_Lookup[((i-1)*6) + 1]] = PiGlow_Brightness * self.OnOrOff
                                PiGlow_Values[PiGlow_Lookup[((i-1)*6) + 2]] = PiGlow_Brightness * self.OnOrOff
                                PiGlow_Values[PiGlow_Lookup[((i-1)*6) + 3]] = PiGlow_Brightness * self.OnOrOff
                                PiGlow_Values[PiGlow_Lookup[((i-1)*6) + 4]] = PiGlow_Brightness * self.OnOrOff
                                PiGlow_Values[PiGlow_Lookup[((i-1)*6) + 5]] = PiGlow_Brightness * self.OnOrOff
                                piglow.update_pwm_values(PiGlow_Values)

                    elif "gpio" in ADDON: # gPiO
                        #print ("gPiO broadcast processing")
                        self.bCheckAll() # Check for all off/on type broadcasts
                        self.bPinCheck() # Check for pin off/on type broadcasts
                    
                    elif "berry" in ADDON: # BerryClip

                        #print ("Berry broadcast processing")                    
                        self.bCheckAll() # Check for all off/on type broadcasts
                        self.bLEDCheck(berryOutputs) # Check for LED off/on type broadcasts
                        if self.bFindOnOff('buzzer'):
                            sghGC.pinUpdate(24,self.OnOrOff)
                            
                    elif "pirocon" in ADDON: # pirocon         

                        self.bCheckAll() # Check for all off/on type broadcasrs
                        self.bPinCheck() # Check for pin off/on type broadcasts
                                    
                        #check pins
                        for pin in sghGC.validPins:
                            if self.bFindOnOff('pin' + str(pin)):
                                sghGC.pinUpdate(pin,self.OnOrOff)

                            if self.bFind('sonar' + str(pin)):
                                distance = sghGC.pinSonar(pin)
                                #print'Distance:',distance,'cm'
                                sensor_name = 'sonar' + str(pin)
                                bcast_str = 'sensor-update "%s" %d' % (sensor_name, distance)
                                #print 'sending: %s' % bcast_str
                                self.send_scratch_command(bcast_str)
                                
                            #Start using ultrasonic sensor on a pin    
                            if self.bFind('ultra' + str(pin)):
                                print 'start pinging on', str(pin)
                                sghGC.pinUse[pin] = sghGC.PULTRA

                    
                        motorList = [['turnr',21,26,7],['turnl',19,24,11]]
                        if "piroconb" in ADDON:
                            logging.debug("PiRoConB Found:%s", ADDON)
                            motorList = [['turnr',21,19,7],['turnl',26,24,11]]						
                                                       
                        if self.bFindValue("move"):
                            svalue = int(self.valueNumeric) if self.valueIsNumeric else 0
                            
                            sghGC.countDirection[motorList[0][3]] = -1 if svalue < 0 else 1
                            print "sghdir" , sghGC.countDirection[motorList[0][3]]
                            
                            turnDualThread = threading.Thread(target=self.stopTurning, args=[motorList,svalue,sghGC.pinCount[motorList[0][3]]])
                            turnDualThread.start()
                            for listLoop in range(0,2):
                                if svalue > 0:
                                    sghGC.pinUpdate(motorList[listLoop][2],1)
                                    sghGC.pinUpdate(motorList[listLoop][1],(100-self.turnSpeed),"pwm")
                                elif svalue < 0:
                                    sghGC.pinUpdate(motorList[listLoop][2],0)
                                    sghGC.pinUpdate(motorList[listLoop][1],(self.turnSpeed),"pwm")
                                
                        if self.bFindValue("turn"):
                            svalue = int(self.valueNumeric) if self.valueIsNumeric else 0
                            turnDualThread = threading.Thread(target=self.stopTurning, args=[motorList,svalue])
                            turnDualThread.start()                        
                            if svalue > 0:
                                sghGC.pinUpdate(motorList[0][2],1)
                                sghGC.pinUpdate(motorList[0][1],(100-self.turnSpeed),"pwm")
                                sghGC.pinUpdate(motorList[1][2],0)
                                sghGC.pinUpdate(motorList[1][1],(self.turnSpeed),"pwm")                               
                            elif svalue < 0:
                                sghGC.pinUpdate(motorList[0][2],0)
                                sghGC.pinUpdate(motorList[0][1],(self.turnSpeed),"pwm")
                                sghGC.pinUpdate(motorList[1][2],1)
                                sghGC.pinUpdate(motorList[1][1],(100-self.turnSpeed),"pwm")                             

                    
                    elif "piringo" in ADDON: # piringo
                        #do piringo stuff
                        self.bCheckAll() # Check for all off/on type broadcasrs
                        self.bLEDCheck(piringoOutputs) # Check for LED off/on type broadcasts
                        self.bLEDPowerCheck(piringoOutputs) # Vary LED Brightness
                        
                    elif "pibrella" in ADDON: # PiBrella
                        #print ("PiBrella broadcast processing")                    
                        self.bCheckAll() # Check for all off/on type broadcasts                    

                        self.bListCheck([13,11,7,15,16,18,22],["led1","led2","led3","led4","led5","led6","led7"])
                        self.bListCheck([13,11,11,11,7,15,16,18,22],["red","amber","yellow","orange","green","outpute","outputf","outputg","outputh"])
                                
                        if self.bFindValue("beep"):
                            try:
                                bn,bd = self.value.split(",")
                            except:
                                bd = "0.25"
                                try:
                                    bn = int(self.valueNumeric)
                                except:
                                    bn = "60"
                            beepNote = int(float(bn))
                            beepDuration = (float(bd))
                            svalue = int(self.valueNumeric) if self.valueIsNumeric else 60
                            beepThread = threading.Thread(target=self.beep, args=[12,440* 2**((beepNote - 69)/12.0),beepDuration])
                            beepThread.start()
                            
                    elif "rgbled" in ADDON: # rgb-led

                        #print ("rgb-led broadcast processing")            
                        if self.bFindOnOff("all"):
                            for loop in range(0,5):
                                sghGC.pinUpdate(rgbOutputs[loop],1 - self.OnOrOff)
                            for loop in range(5,8):
                                sghGC.pinUpdate(rgbOutputs[loop],self.OnOrOff)

                        rgbList = rgbOutputs[0:5]
                        for listLoop in rgbList:
                            if self.bFindOnOff("led"+str(1+rgbList.index(listLoop))):
                                sghGC.pinUpdate(rgbOutputs[rgbList.index(listLoop)],1-self.OnOrOff)
                        
                        
                        rgbList = ["red","green","blue"]
                        for listLoop in rgbList:
                            if self.bFindOnOff(listLoop):
                                print listLoop , "found",
                                sghGC.pinUpdate(rgbOutputs[5+rgbList.index(listLoop)],self.OnOrOff)
                                
                    elif "pidie" in ADDON: # pidie
                        #do piringo stuff
                        self.bCheckAll() # Check for all off/on type broadcasrs

                        self.bLEDPowerCheck(pidieOutputs) # Vary LED Brightness          

                        self.bListCheck([7,11,12,13,15,16,18,22,8],["led1","led2","led3","led4","led5","led6","led7","led8","led9"])
                        self.bListCheck([7,11,12,13,15,16,18,22,8],["1","2","3","4","5","6","7","8","9"])                             

                    elif "fishdish" in ADDON: # fishdish
                        #do piringo stuff
                        self.bCheckAll() # Check for all off/on type broadcasrs
                        self.bLEDCheck(fishOutputs) # Check for LED off/on type broadcasts
                        self.bLEDPowerCheck(fishOutputs) # Vary LED Brightness       

                        fishList = ["green","yellow","red"]
                        for listLoop in fishList:
                            if self.bFindOnOff(listLoop):
                                print listLoop , "found",
                                sghGC.pinUpdate(fishOutputs[fishList.index(listLoop)],self.OnOrOff)    
                                
                        if self.bFindOnOff('buzzer'):
                            sghGC.pinUpdate(24,self.OnOrOff)                         

       
                    else: # Plain GPIO Broadcast processing

                        self.bCheckAll() # Check for all off/on type broadcasrs
                        #self.bPinCheck() # Check for pin off/on type broadcasts
                                    
                        #check pins
                        for pin in sghGC.validPins:
                            if self.bFindOnOff('pin' + str(pin)):
                                sghGC.pinUpdate(pin,self.OnOrOff)
                            if self.bFindOnOff('gpio' + str(sghGC.gpioLookup[pin])):
                                sghGC.pinUpdate(pin,self.OnOrOff)
                            if self.bFindValue('power' + str(pin)+","):
                                #logging.debug("bPowerPin:%s",pin )    
                                if self.valueIsNumeric:
                                    sghGC.pinUpdate(pin,self.valueNumeric,type="pwm")
                                else:
                                    sghGC.pinUpdate(pin,0,type="pwm")                                   

                            if self.bFind('sonar' + str(pin)):
                                
                                distance = sghGC.pinSonar(pin)
                                #print'Distance:',distance,'cm'
                                sensor_name = 'sonar' + str(pin)
                                bcast_str = 'sensor-update "%s" %d' % (sensor_name, distance)
                                #print 'sending: %s' % bcast_str
                                self.send_scratch_command(bcast_str)
                                
                            #Start using ultrasonic sensor on a pin    
                            if self.bFind('ultra' + str(pin)):
                                print 'start pinging on', str(pin)
                                sghGC.pinUse[pin] = sghGC.PULTRA
                           
                                          
                        #end of normal pin checking

                                
                    stepperList = [['positiona',[11,12,13,15]],['positionb',[16,18,22,7]]]
                    for listLoop in range(0,2):
                        #print ("loop" , listLoop)
                        if self.bFindValue(stepperList[listLoop][0]):
                            if self.valueIsNumeric:
                                self.stepperUpdate(stepperList[listLoop][1],10,self.valueNumeric)
                            else:
                                self.stepperUpdate(stepperList[listLoop][1],0)                            

                    if self.bFind('pinpattern'):
                        #print 'Found pinpattern broadcast'
                        #print dataraw
                        #num_of_bits = PINS
                        outputall_pos = self.dataraw.find('pinpattern')
                        sensor_value = self.dataraw[(outputall_pos+10):].split()
                        #print sensor_value
                        #sensor_value[0] = sensor_value[0][:-1]                    
                        #print sensor_value[0]
                        bit_pattern = ('00000000000000000000000000'+sensor_value[0])[-sghGC.numOfPins:]
                        #print 'bit_pattern %s' % bit_pattern
                        j = 0
                        for pin in sghGC.validPins:
                            if (sghGC.pinUse[pin] == sghGC.POUTPUT):
                                #print "pin" , bit_pattern[-(j+1)]
                                if bit_pattern[-(j+1)] == '0':
                                    sghGC.pinUpdate(pin,0)
                                else:
                                    sghGC.pinUpdate(pin,1)
                                j = j + 1
                                 
                    if pcfSensor != None: #if PCF ADC found
                        for channel in range(1,5): #loop thru all 4 inputs
                            if self.bFind('adc'+str(channel)):
                                adc = pcfSensor.readADC(channel - 1) # get each value
                                #print'Distance:',distance,'cm'
                                sensor_name = 'adc'+str(channel)
                                bcast_str = 'sensor-update "%s" %d' % (sensor_name, adc)
                                #print 'sending: %s' % bcast_str
                                self.send_scratch_command(bcast_str)
                                
                                    
                    origdataraw = self.dataraw
                    if AdaMatrix != None: #Matrix connected
                        #print self.dataraw
                        #print
                        self.dataraw = self.dataraw[self.dataraw.find("broadcast") + 10:]
                        #print self.dataraw
                        #print
                        #print self.dataraw.split('broadcast')
                        broadcastList = self.dataraw.split(' ')
                        for broadcastListLoop in broadcastList:
                            self.dataraw = str(broadcastListLoop)
                            #print self.dataraw
                            if self.bFind("alloff"):
                                AdaMatrix.clear()
                            if self.bFind("sweep"):
                                for y in range(0, 8):
                                    for x in range(0, 8):
                                        AdaMatrix.setPixel((7-x),y)
                                        time.sleep(0.05)
                            
                            for ym in range(0,8):
                                for xm in range(0,8):
                                
                                    if self.bFindValue("matrixon"+str(xm)+"x"+str(ym)+"y"):
                                        AdaMatrix.setPixel((7 - xm),ym)
                                        
                                    if self.bFindValue("matrixoff"+str(xm)+"x"+str(ym)+"y"):
                                        AdaMatrix.clearPixel((7 - xm),ym)
                                
                                # if self.bFindValue("matrixon"):
                                    # #print self.value
                                    # xPos = int(self.value[0:1])
                                    # yPos = int(self.value[2:3])
                                    # AdaMatrix.setPixel((7 - xPos),yPos)
                                    
                                # if self.bFindValue("matrixoff"):
                                    # #print self.value
                                    # xPos = int(self.value[0:1])
                                    # yPos = int(self.value[2:3])
                                    # AdaMatrix.clearPixel((7 - xPos),yPos)        
                                    
                            if self.bFindValue("brightness"):
                                if self.valueIsNumeric:
                                    AdaMatrix.setBrightness(max(0,min(15,self.valueNumeric)))
                                else:
                                    AdaMatrix.setBrightness(15)
                                    
                            if self.bFindValue('matrixpattern'):
                                bit_pattern = (self.value+'00000000000000000000000000000000000000000000000000000000000000000')[0:64]
                                #print 'bit_pattern %s' % bit_pattern
                                for j in range(0,64):
                                    ym = j // 8
                                    xm = j - (8 * ym)
                                    if bit_pattern[j] == '0':
                                        AdaMatrix.clearPixel((7 - xm),ym)
                                    else:
                                        AdaMatrix.setPixel((7 - xm),ym)
                                    j = j + 1
                                    
                            rowList = ['a','b','c','d','e','f','g','h']
                            for i in range(0,8):
                                if self.bFindValue('row'+rowList[i]):
                                    bit_pattern = (self.value + "00000000")[0:8]
                                    #print 'bit_pattern %s' % bit_pattern
                                    for j in range(0,8):
                                        ym = i
                                        xm = j
                                        if bit_pattern[(j)] == '0':
                                            AdaMatrix.clearPixel((7 - xm),ym)
                                        else:
                                            AdaMatrix.setPixel((7 - xm),ym)
                                            
                            colList = ['a','b','c','d','e','f','g','h']
                            for i in range(0,8):
                                if self.bFindValue('col'+rowList[i]):
                                    #print self.value
                                    bit_pattern = (self.value + "00000000")[0:8]
                                    for j in range(0,8):
                                        ym = j
                                        xm = i
                                        if bit_pattern[(j)] == '0':
                                            AdaMatrix.clearPixel((7 - xm),ym)
                                        else:
                                            AdaMatrix.setPixel((7 - xm),ym)       

                            if self.bFindValue('scrollleft'):
                                AdaMatrix.scroll("left")
                            if self.bFindValue('scrollright'):
                                print "scrollr" 
                                AdaMatrix.scroll("right")    
                                
                    if PiMatrix != None: #Matrix connected
                        #print self.dataraw
                        #print
                        self.dataraw = self.dataraw[self.dataraw.find("broadcast") + 10:]
                        #print self.dataraw
                        #print
                        #print self.dataraw.split('broadcast')
                        broadcastList = self.dataraw.split(' ')
                        for broadcastListLoop in broadcastList:
                            self.dataraw = str(broadcastListLoop)
                            #print self.dataraw
                            
                            if self.bFindOnOff("all"):
                                PiMatrix.clear(self.OnOrOff)
                                
                            if self.bFindOnOff("sweep"):
                                for y in range(0, 8):
                                    for x in range(0, 8):
                                        PiMatrix.setPixel(x,y,self.OnOrOff)
                                        time.sleep(0.05)  
                            
                            if self.bFindValue("matrixo"):
                                for ym in range(0,8):
                                    for xm in range(0,8):
                                        if self.bFindValue("matrixonx"+str(xm)+"y"+str(ym)):
                                            PiMatrix.setPixel(xm,ym)
                                        if self.bFindValue("matrixoffx"+str(xm)+"y"+str(ym)):
                                            PiMatrix.clearPixel(xm,ym)
                                    
                            # if self.bFindValue("brightness"):
                                # if self.valueIsNumeric:
                                    # PiMatrix.setBrightness(max(0,min(15,self.valueNumeric)))
                                # else:
                                    # PiMatrix.setBrightness(15)
                                    
                            if self.bFindValue('matrixpattern'):
                                bit_pattern = (self.value+'00000000000000000000000000000000000000000000000000000000000000000')[0:64]
                                #print 'bit_pattern %s' % bit_pattern
                                for j in range(0,64):
                                    ym = j // 8
                                    xm = j - (8 * ym)
                                    PiMatrix.setPixel(xm,ym,int(float(bit_pattern[j])))
                                    j = j + 1
                                    
                            rowList = ['a','b','c','d','e','f','g','h']
                            for i in range(0,8):
                                if self.bFindValue('row'+rowList[i]):
                                    bit_pattern = (self.value + "00000000")[0:8]
                                    #print 'bit_pattern %s' % bit_pattern
                                    for j in range(0,8):
                                        ym = i
                                        xm = j
                                        PiMatrix.setPixel(xm,ym,int(float(bit_pattern[j])))
                                            
                            colList = ['a','b','c','d','e','f','g','h']
                            for i in range(0,8):
                                if self.bFindValue('col'+rowList[i]):
                                    #print self.value
                                    bit_pattern = (self.value + "00000000")[0:8]
                                    for j in range(0,8):
                                        ym = j
                                        xm = i
                                        PiMatrix.setPixel(xm,ym,int(float(bit_pattern[j])))   

                            if self.bFindValue('scrollleft'):
                                PiMatrix.scroll("left")
                            if self.bFindValue('scrollright'):
                                PiMatrix.scroll("right")    
                                
                    self.dataraw = origdataraw
                    
                    if self.bFind('gettime'):
                        now = dt.datetime.now()
                        #print now
                        fulldatetime = now.strftime('%Y%m%d%H%M%S')
                        sensor_name = 'fulldatetime'
                        bcast_str = 'sensor-update "%s" %s' % (sensor_name, fulldatetime)
                        #print 'sending: %s' % bcast_str
                        self.send_scratch_command(bcast_str)
                        hrs = now.strftime('%H')
                        sensor_name = 'hours'
                        bcast_str = 'sensor-update "%s" %s' % (sensor_name, hrs)
                        #print 'sending: %s' % bcast_str
                        self.send_scratch_command(bcast_str)
                        minutes = now.strftime('%M')
                        sensor_name = 'minutes'
                        bcast_str = 'sensor-update "%s" %s' % (sensor_name, minutes)
                        #print 'sending: %s' % bcast_str
                        self.send_scratch_command(bcast_str)
                        
                    if self.bFind("readcount"): #update pin count values
                        for pin in sghGC.validPins: #loop thru all pins
                            if sghGC.pinUse[pin] == sghGC.PCOUNT:
                                if self.bFind('readcount'+str(pin)):
                                    #print ('readcount'+str(pin))
                                    #print (sghGC.pinCount[pin])
                                    #print'Distance:',distance,'cm'
                                    sensor_name = 'count'+str(pin)
                                    bcast_str = 'sensor-update "%s" %d' % (sensor_name, sghGC.pinCount[pin])
                                    #print 'sending: %s' % bcast_str
                                    self.send_scratch_command(bcast_str)
                                    
                    if self.bFind("resetcount"): #update pin count values
                        for pin in sghGC.validPins: #loop thru all pins
                            if sghGC.pinUse[pin] == sghGC.PCOUNT:
                                if self.bFind('resetcount'+str(pin)):
                                    sghGC.pinCount[pin] = 0
                        print "diff reset"
                        self.encoderDiff = 0

                    if self.bFind("getip"): #find ip address
                        logging.debug("Finding IP")
                        arg = 'ip route list'
                        p=subprocess.Popen(arg,shell=True,stdout=subprocess.PIPE)
                        ipdata = p.communicate()
                        split_data = ipdata[0].split()
                        ipaddr = split_data[split_data.index('src')+1]
                        logging.debug("IP:%s", ipaddr)
                        sensor_name = 'ipaddress'
                        bcast_str = 'sensor-update "%s" %s' % (sensor_name, "ip"+ipaddr)
                        self.send_scratch_command(bcast_str)
                        
                    if self.bFind("gettemp"): #find temp address
                        if sghGC.dsSensorId == "":
                            sghGC.findDS180()
                            if sghGC.dsSensorId == "":
                                print "ds:", sghGC.dsSensorId
                                print "getting temp"
                                temperature = sghGC.getDS180Temp( sghGC.dsSensorId)
                                sensor_name = 'temperature'
                                bcast_str = 'sensor-update "%s" %s' % (sensor_name, str(temperature))
                                self.send_scratch_command(bcast_str)
                                             
                                        

                    if  '1coil' in dataraw:
                        print "1coil broadcast"
                        stepType = 0
                        print "step mode" ,stepMode[stepType]
                        step_delay = 0.0025

                    if  '2coil' in dataraw:
                        print "2coil broadcast"
                        stepType = 1
                        print "step mode" ,stepMode[stepType]
                        step_delay = 0.0025
                        
                    if  'halfstep' in dataraw:
                        print "halfstep broadcast"
                        stepType = 2
                        print "step mode" ,stepMode[stepType]
                        step_delay = 0.0013
                        
                    if "version" in dataraw:
                        bcast_str = 'sensor-update "%s" %s' % ("Version", Version)
                        #print 'sending: %s' % bcast_str
                        self.send_scratch_command(bcast_str)
                        
                 
                    #end of broadcast check


                if 'stop handler' in dataraw:
                    print "stop handler msg setn from Scratch"
                    cleanup_threads((listener, sender))
                    sys.exit()

                #else:
                    #print 'received something: %s' % dataraw
###  End of  ScratchListner Class

def create_socket(host, port):
    while True:
        try:
            print 'Trying'
            scratch_sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            scratch_sock.connect((host, port))
            break
        except socket.error:
            print "There was an error connecting to Scratch!"
            print "I couldn't find a Mesh session at host: %s, port: %s" % (host, port) 
            time.sleep(3)
            #sys.exit(1)

    return scratch_sock

def cleanup_threads(threads):
    print ("cleanup threads started")
    for thread in threads:
        thread.stop()
    print "Threads told to stop"
    
    for thread in threads:
        thread.join()
    print "Waiting for join on main threads to complete"
        
    for pin in sghGC.validPins:
        try:
            print "Stopping ", pin
            sghGC.pinRef[pin].stop()
            print "Stopped ", pin
        except:
            continue
            
    try:
        print "Stopping Matrix"
        PiMatrix.stop()
        print "Stopped "
    except:
        pass

    print ("cleanup threads finished")

        
######### Main Program Here


#Set some constants and initialise lists

sghGC = sgh_GPIOController.GPIOController(True)
print sghGC.getPiRevision()

ADDON = ""
logging.basicConfig(stream=sys.stderr, level=logging.DEBUG)# default DEBUG - quiwr = INFO

 
PORT = 42001
DEFAULT_HOST = '127.0.0.1'
BUFFER_SIZE = 8192 #used to be 100
SOCKET_TIMEOUT = 2
firstRun = True
lock = threading.Lock()

piglow = None
try:
    if sghGC.getPiRevision() == 1:
        print "Rev1 Board" 
        piglow = sgh_PiGlow.PiGlow(0)
    else:
        piglow = sgh_PiGlow.PiGlow(1)
        print ("PiGlow:",piglow)
        print ("Update PWM value on PiGLow attempted")
        piglow.update_pwm_values()#PiGlow_Values)
except:
    print "No PiGlow Detected"
    

##if sghGC.getPiRevision() == 1:
##    print "Rev1 Board" 
##    piglow = sgh_PiGlow.PiGlow(0)
##else:
##    piglow = sgh_PiGlow.PiGlow(1)
##print ("PiGlow:",piglow)
##print ("Update PWM value on PiGLow attempted")
##piglow.update_pwm_values()#PiGlow_Values)

    
#See if Compass connected
compass = None
try:
    if sghGC.getPiRevision == 1:
        compass = Compass(gauss = 4.7, declination = (-0,0))
    else:
        compass = Compass(gauss = 4.7, declination = (-0,0))
    print "compass detected"
except:
    print "No Compass Detected"
    
pcaPWM = None
try:
    pcaPWM = PWM(0x40, debug=False)
    print pcaPWM
    print pcaPWM.setPWMFreq(60)                        # Set frequency to 60 Hz
    print "AdaFruit PCA9685 detected"
except:
    print "No pcaPwm Detected"
    
pcfSensor = None
# try:
    # if sghGC.getPiRevision() == 1:
        # pcfSensor = sgh_PCF8591P(0) #i2c, 0x48)
    # else:
        # pcfSensor = sgh_PCF8591P(1) #i2c, 0x48)
    # print pcfSensor
    # print "PCF8591P Detected"
# except:
    # print "No PCF8591 Detected"
    
AdaMatrix = None
try:
    AdaMatrix = sgh_EightByEight(address=0x70)
    print AdaMatrix
    print "AdaMatrix Detected"
except:
    print "No AdaMatrix Detected"
    
PiMatrix = None
#PiMatrix = sgh_PiMatrix.sgh_PiMatrix(0x20,0)
try:
    if sghGC.getPiRevision() == 1:
        print "Rev1 Board" 
        PiMatrix = sgh_PiMatrix(0x20,0)
    else:
        PiMatrix = sgh_PiMatrix(0x20,1)
    print PiMatrix
    print "PiMatrix Detected"
    PiMatrix.start()
except:
    print "No PiMatrix Detected"
#PiMatrix.start()
    #time.sleep(5)
    


if __name__ == '__main__':
    SCRIPTPATH = os.path.split(os.path.realpath(__file__))[0]
    print "PATH:" ,SCRIPTPATH
    if len(sys.argv) > 1:
        host = sys.argv[1]
    else:
        host = DEFAULT_HOST
    host = host.replace("'", "")
    
    GPIOPlus = True
    if len(sys.argv) > 2:
        if sys.argv[2] == "standard":
            GPIOPlus = False
        


cycle_trace = 'start'


#sghGC.setPinMode()

while True:

    if (cycle_trace == 'disconnected'):
        print "Scratch disconnected"
        cleanup_threads((listener, sender))
        print "Thread cleanup done after disconnect"
        sghGC.stopServod()
        print "servod stopped afer disconnect"
        time.sleep(1)
        cycle_trace = 'start'

    if (cycle_trace == 'start'):
        ADDON = ""
        INVERT = False
        # open the socket
        print 'Starting to connect...' ,
        the_socket = create_socket(host, PORT)
        print 'Connected!'
        the_socket.settimeout(SOCKET_TIMEOUT) #removed 3dec13 to see what happens
        listener = ScratchListener(the_socket)
#        steppera = StepperControl(11,12,13,15,step_delay)
#        stepperb = StepperControl(16,18,22,7,step_delay)
#        stepperc = StepperControl(24,26,19,21,step_delay)


##        data = the_socket.recv(BUFFER_SIZE)
##        print "Discard 1st data buffer" , data[4:].lower()
        sender = ScratchSender(the_socket)
        cycle_trace = 'running'
        print "Running...."
        listener.start()
        sender.start()
##        stepperb.start()


    # wait for ctrl+c
    try:
#        val = values.pop(0)
#        values.append(val)
#        # update the piglow with current values
#        piglow.update_pwm_values(values)

        time.sleep(0.1)
    except KeyboardInterrupt:
        print ("Keyboard Interrupt")
        cleanup_threads((listener,sender))
        sghGC.stopServod()
        print ("servod stopped")
        sghGC.cleanup()
        print ("Pin Cleanup done")
        sys.exit()
        print "CleanUp complete"
        
#### End of main program

        

