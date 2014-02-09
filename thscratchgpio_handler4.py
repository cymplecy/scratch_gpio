#!/usr/bin/env python
# ScratchThymio - control Thymio Robot coupled with Raspberry Pi  using Scratch.
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
Version =  'v0.0.02' # 6Feb14



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
  
try:  
    import dbus
    import dbus.mainloop.glib
    import gobject
    from optparse import OptionParser
except:
    pass

proxSensorsVal=[0,0,0,0,0]
groundSensorsAmbient=[0,0]
groundSensorsReflected=[0,0]
groundSensorsDelta=[0,0]
tempSensorsVal=[0]
accSensorsVal = [0,0,0]


def eventLoop():
    #print "Brait", network
    #get the values of the sensors
    Thymio.GetVariable("thymio-II", "prox.horizontal",reply_handler=get_variables_reply,error_handler=get_variables_error)
    # Thymio.GetVariable("thymio-II", "prox.ground.delta",reply_handler=deltaget_variables_reply,error_handler=deltaget_variables_error)
    # Thymio.GetVariable("thymio-II", "temperature",reply_handler=tempget_variables_reply,error_handler=tempget_variables_error)

    # Thymio.GetVariable("thymio-II", "acc",reply_handler=accget_variables_reply,error_handler=accget_variables_error)     
    #print the proximity sensors value in the terminal
    #print proxSensorsVal[0],proxSensorsVal[1],proxSensorsVal[2],proxSensorsVal[3],proxSensorsVal[4]
    return True


def get_variables_reply(r):
    global proxSensorsVal
    #print "R:", r
    proxSensorsVal=r

def get_variables_error(e):
    print 'error:'
    print str(e)
    loop.quit()
    
def deltaget_variables_reply(r):
    global groundSensorsDelta
    #print "R:", r
    groundSensorsDelta=r

def deltaget_variables_error(e):
    print 'error:'
    print str(e)
    loop.quit()
    
def tempget_variables_reply(r):
    global tempSensorsVal
    #print "R:", r
    tempSensorsVal=r

def tempget_variables_error(e):
    print 'error:'
    print str(e)
    loop.quit()
    
def accget_variables_reply(r):
    global accSensorsVal
    #print "R:", r
    accSensorsVal=r

def accget_variables_error(e):
    print 'error:'
    print str(e)
    loop.quit()


    
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
                if (sghGC.pinUse[pin] == sghGC.PINPUT):
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
                sensor_name = "In" + ["NA","A","B","C","D","E"][([0,21,26,24,19,23].index(pin))]
                if sensor_name == "InE":
                    sensor_name = "switch"
            except:
                print "pibrella input out of range"
                sensor_name = "pin" + str(pin)
                pass
                
        elif "pidie" in ADDON:
            #print pin
            #sensor_name = "in" + str([0,19,21,24,26,23].index(pin))
            try:
                sensor_name = ["blue","red","white","yellow"][([19,21,26,24].index(pin))]
            except:
                print "pidie input out of range"
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
            bcast_str = 'sensor-update "%s" %s' %  (sensor_name,("Down","Up")[value == 1])
        if ("fishdish" in ADDON):
            bcast_str = 'sensor-update "switch" %s' %  (("Up","Down")[value == 1])
        #print 'sending: %s' % bcast_str
        self.send_scratch_command(bcast_str)
        if "motorpitx" in ADDON:
            bcast_str = 'broadcast "%s%s"' % (sensor_name,("Off","On")[value == 1])
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
        
        self.send_scratch_command('broadcast "SetPins"')
        #print sghGC.pinUse
        with lock:
            for pin in sghGC.validPins:
                if (sghGC.pinUse[pin] == sghGC.PINPUT):
                    #self.broadcast_pin_update(pin, sghGC.pinRead(pin))
                    last_bit_pattern += sghGC.pinRead(pin) << pin
                else:
                    last_bit_pattern += 1 << pin
                #print 'lbp %s' % bin(last_bit_pattern)

        last_bit_pattern = last_bit_pattern ^ -1
        lastPinUpdateTime = 0
        lastThymioSensorUpdateTime = 0
        while not self.stopped():
            #time.sleep(0.01) # be kind to cpu  :)
            #print "sender running"
            pin_bit_pattern = 0
            with lock:
                #print "lOCKED"
                for pin in sghGC.validPins:
                    #print pin
                    if (sghGC.pinUse[pin] == sghGC.PINPUT):
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
            
            if (time.time() - lastPinUpdateTime)  > 0.1:
                #print int(time.time())
                lastPinUpdateTime = time.time()
                for pin in sghGC.validPins:
                    if (sghGC.pinUse[pin] == sghGC.PINPUT):
                        self.broadcast_pin_update(pin, sghGC.pinRead(pin))

   
                
            # if (time.time() - lastThymioSensorUpdateTime) > 0.6:

                # if Thymio != None:
                    # #print "getsensors"
                    # for loop in range(0,5):
                        # #print proxSensorsVal[loop]
                        # bcast_str = 'sensor-update "%s" %s' % ("front"+str(loop + 1), str(float(proxSensorsVal[loop])))
                        # self.send_scratch_command(bcast_str)
                    # for loop in range(0,2):
                        # #print groundSensorsDelta[loop]
                        # bcast_str = 'sensor-update "%s" %s' % ("ground"+str(loop + 1), str(float(groundSensorsDelta[loop])))
                        # self.send_scratch_command(bcast_str)                      
                    # #print tempSensorsVal[0]
                    # bcast_str = 'sensor-update "%s" %s' % ("temp", str(float(tempSensorsVal[0])))
                    # self.send_scratch_command(bcast_str)              
                    # for loop in range(0,3):
                        # #print accSensorsVal[loop]
                        # bcast_str = 'sensor-update "%s" %s' % ("accelerometer"+str(loop + 0), str(float(accSensorsVal[loop])))
                        # self.send_scratch_command(bcast_str)                                  

                # lastThymioSensorUpdateTime = time.time()                
                
                


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
        return (searchStr in self.dataraw)
        
    def bFindOn(self,searchStr):
        return (self.bFind(searchStr + 'on') or self.bFind(searchStr + 'high'))
        
    def bFindOff(self,searchStr):
        return (self.bFind(searchStr + 'off') or self.bFind(searchStr + 'low'))
        
    def bFindOnOff(self,searchStr):
        self.OnOrOff = None
        if (self.bFind(searchStr + 'on') or self.bFind(searchStr + 'high')):
            self.OnOrOff = 1
            return True
        elif (self.bFind(searchStr + 'off') or self.bFind(searchStr + 'low')):
            self.OnOrOff = 0
            return True
        else:
            return False
            
    # def dRtnOnOff(self,searchStr):
        # if self.bFindOn(searchStr):
            # return 1
        # else:
            # return 0

    def bCheckAll(self):
        if self.bFindOnOff('all'):
            for pin in sghGC.validPins:
                #print pin
                if sghGC.pinUse[pin] in [sghGC.POUTPUT,sghGC.PPWM]:
                    #print pin
                    sghGC.pinUpdate(pin,self.OnOrOff)

    def bPinCheck(self):
        for pin in sghGC.validPins:
            if self.bFindOnOff('pin' + str(pin)):
                sghGC.pinUpdate(pin,self.OnOrOff)
            if self.bFindOnOff('gpio' + str(sghGC.gpioLookup[pin])):
                sghGC.pinUpdate(pin,self.OnOrOff)
                #print pin

    def bLEDCheck(self,ledList):
        for led in range(1,(1+ len(ledList))): # loop thru led numbers
            if self.bFindOnOff('led' + str(led)):
                sghGC.pinUpdate(ledList[led - 1],self.OnOrOff)
                
    def bFindValue(self,searchStr):
        #print "searching for ", searchStr 
        self.value = None
        self.valueNumeric = None
        self.valueIsNumeric = False
        if self.bFind(searchStr):
            self.findPos = self.dataraw.find((searchStr))
            logging.debug("1st chars of value:%s",(self.dataraw[self.findPos + len(searchStr):]) )
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
        return ((searchStr + ' ') in self.dataraw)
        
    def vFindOn(self,searchStr):
        return (self.vFind(searchStr + 'on') or self.vFind(searchStr + 'high')or self.vFind(searchStr + '1'))
        
    def vFindOff(self,searchStr):
        return (self.vFind(searchStr + 'off') or self.vFind(searchStr + 'low') or self.vFind(searchStr + '0'))
        
    def vFindOnOff(self,searchStr):
        self.value = None
        self.valueNumeric = None
        self.valueIsNumeric = False
        if self.vFind(searchStr):
            self.value = self.getValue(searchStr)
            if str(self.value) in ["high","on","1"]:
                self.valueNumeric = 1
            else:
                self.valueNumeric = 0
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
                if self.valueIsNumeric:
                    sghGC.pinUpdate(pin,self.valueNumeric)
                else:
                    sghGC.pinUpdate(pin,0)
                    
            if self.vFindValue('gpiopower' + str(sghGC.gpioLookup[pin])):
                #print pin , "found"
                if self.valueIsNumeric:
                    sghGC.pinUpdate(pin,self.valueNumeric,type="pwm")
                else:
                    sghGC.pinUpdate(pin,0,type="pwm")
                    
    def vLEDCheck(self,ledList):
        for led in range(1,(1+ len(ledList))): # loop thru led numbers
            if self.vFindValue('led' + str(led)):
                if self.valueIsNumeric:
                    sghGC.pinUpdate(ledList[led - 1],self.valueNumeric)
                else:
                    sghGC.pinUpdate(ledList[led - 1],0)
                    
            if self.vFindValue('power' + str(led)):
                if self.valueIsNumeric:
                    sghGC.pinUpdate(ledList[led - 1],self.valueNumeric,type="pwm")
                else:
                    sghGC.pinUpdate(ledList[led - 1],0,type="pwm")
                    
    def stop(self):
        self._stop.set()

    def stopped(self):
        return self._stop.isSet()

    def run(self):
        global firstRun,cycle_trace,step_delay,stepType,INVERT, \
               Ultra,ultraTotalInUse,piglow,PiGlow_Brightness,compass,ADDON,proxSensorsVal
        


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
                    sghGC.pinUse[pin] = sghGC.PINPUTDOWN
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
        
        
        #This is the main loop that listens for messages from Scratch and sends appropriate commands off to various routines
        while not self.stopped():
            #lcount += 1
            #print lcount
            try:
                #print "try reading socket"
                BUFFER_SIZE = 512 # This size will accomdate normal Scratch Control 'droid app sensor updates
                data = dataPrevious + self.scratch_socket.recv(BUFFER_SIZE) # get the data from the socket plus any data not yet processed
                logging.debug(len(data)) 
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
                dataraw = dataraw = ' '.join([item.replace(' ','') for item in shlex.split(dataItem)])
                self.dataraw = dataraw
                #print "Loop processing"
                #print self.dataraw
                #print
                if 'sensor-update' in self.dataraw:
                    #print "this data ignored" , dataraw
                    firstRunData = self.dataraw
                    #dataraw = ''
                    #firstRun = False

 
                #If outputs need globally inverting (7 segment common anode needs it - PiRingo etc)
                if ('invert' in self.dataraw):
                    sghGC.INVERT = True
                    
      
    ### Check for AddOn boards being declared
                    
                #Listen for Variable changes
                if 'sensor-update' in self.dataraw:

                    #print "sensor-update rcvd" , dataraw
                  
                    if "fishdish" in ADDON:
                        #do fishdish stuff
                        self.vAllCheck("leds") # check All LEDS On/Off/High/Low/1/0

                        self.vLEDCheck(fishOutputs) # check All LEDS On/Off/High/Low/1/0
                                    
                        if self.vFindOnOff('buzzer'):
                            self.index_pin_update(24,self.valueNumeric)
                                    
                    else:   #normal variable processing with no add on board
                    
                        self.vAllCheck("allpins") # check All On/Off/High/Low/1/0
                        
                        self.vPinCheck() # check for any pin On/Off/High/Low/1/0 any PWM settings using power or motor


                        if  Thymio != None:
                            if self.vFindValue("motora"):
                                Thymio.SetVariable("thymio-II", "motor.left.target", [self.valueNumeric])
                            if self.vFindValue("motorleft"):
                                Thymio.SetVariable("thymio-II", "motor.left.target", [self.valueNumeric])                                
                            if self.vFindValue("motorb"):
                                Thymio.SetVariable("thymio-II", "motor.right.target", [self.valueNumeric])
                            if self.vFindValue("motorright"):
                                Thymio.SetVariable("thymio-II", "motor.right.target", [self.valueNumeric])                                
                                
 
    ### Check for Broadcast type messages being received
 
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
logging.basicConfig(stream=sys.stderr, level=logging.INFO)# default DEBUG

 
PORT = 42001
DEFAULT_HOST = '127.0.0.1'
BUFFER_SIZE = 8192 #used to be 100
SOCKET_TIMEOUT = 2
firstRun = True
lock = threading.Lock()


    

    


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
    Thymio = None
    try:
        parser = OptionParser()
        parser.add_option("-s", "--system", action="store_true", dest="system", default=False,help="use the system bus instead of the session bus")

        (options, args) = parser.parse_args()

        dbus.mainloop.glib.DBusGMainLoop(set_as_default=True)

        if options.system:
            bus = dbus.SystemBus()
        else:
            bus = dbus.SessionBus()

        #Create Aseba network 
        network = dbus.Interface(bus.get_object('ch.epfl.mobots.Aseba', '/'), dbus_interface='ch.epfl.mobots.AsebaNetwork')

        #print in the terminal the name of each Aseba NOde
        print network.GetNodesList()
        Thymio = network
    except:
        print "No Thmymio found"
   


cycle_trace = 'start'


#sghGC.setPinMode()

while True:

    if (cycle_trace == 'disconnected'):
        print "Scratch disconnected"
        cleanup_threads((listener, sender))
        print "Thread cleanup done after disconnect"
        sghGC.stopServod()
        print "servod stopped afer disconnect"
        #os.system('sudo pkill -f asebamedulla')
        time.sleep(1)
        cycle_trace = 'start'

    if (cycle_trace == 'start'):
        #os.system('asebamedulla "ser:device=/dev/ttyACM0" &')
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
        gobject.threads_init()           
        loop = gobject.MainLoop()
        #call the callback of Braitenberg algorithm
        handle = gobject.timeout_add (100, eventLoop) #every 0.1 sec
        #print "handle", handle
        #try:
        print "start loop"
        loop.run()
        print "loop exited"
        #except: 
            #pass
            # #KeyboardInterrupt:
            # print "Unexpected error:", sys.exc_info()[0]
            # sys.exit()
            # print ("Keyboard Interrupt")
            # cleanup_threads((listener,sender))
            # sghGC.stopServod()
            # print ("servod stopped")
            # sghGC.cleanup()
            # print ("Pin Cleanup done")
            # sys.exit()
            # print "Thymio CleanUp complete"            
            # print "loop running"
##        stepperb.start()


    # wait for ctrl+c
    try:
#        val = values.pop(0)
#        values.append(val)
#        # update the piglow with current values
#        piglow.update_pwm_values(values)

        time.sleep(2)
        print "loop"
    except KeyboardInterrupt:
        print ("Keyboard Interrupt")
        cleanup_threads((listener,sender))
        if Thymio != None:
            loop.quit()
            print "loop quitted"
        sghGC.stopServod()
        print ("servod stopped")
        sghGC.cleanup()
        print ("Pin Cleanup done")
        sys.exit()
        print "CleanUp complete"
        
#### End of main program

        

