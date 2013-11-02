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
Version =  '3.0.2' # 31Oct13



from array import *
import threading
import socket
import time
import sys
import struct
import datetime as dt
import shlex
import os
import math
try:
    from Adafruit_PWM_Servo_Driver import PWM
except:
    pass
#try and inport smbus but don't worry if not installed
try:
    from smbus import SMBus
except:
    pass

import RPi.GPIO as GPIO
GPIO.setmode(GPIO.BOARD)
GPIO.setwarnings(False)
GPIO.cleanup()
print "Board Revision" , GPIO.RPI_REVISION

#Set some constants and initialise arrays
STEPPERA=0
STEPPERB=1
STEPPERC=2
stepperInUse = array('b',[False,False,False])
INVERT = False
BIG_NUM = 2123456789

ADDON = ['Normal','Ladder','MotorPiTx','PiGlow','Compass','gPiO','Berry','pirocon'] #define addons
NUMOF_ADDON = len(ADDON) # find number of addons
ADDON_PRESENT = [False] * NUMOF_ADDON # create an enabled/disabled array
for i in range(NUMOF_ADDON): # set all addons to diabled
    ADDON_PRESENT[i] = False
    ADDON[i] = ADDON[i].lower()
    
turnAStep = 0
turnBStep = 0
turnCStep = 0
stepMode = ['1Coil','2Coil','HalfStep']
stepModeDelay = [0.0025,0.0025,0.0013]
stepType = 2
if stepType == 2:
    step_delay = 0.0013 # use smaller dealy fro halfstep mode
else:
    step_delay = 0.003

PORT = 42001
DEFAULT_HOST = '127.0.0.1'
BUFFER_SIZE = 240 #used to be 100
SOCKET_TIMEOUT = 1

CMD_ENABLE_OUTPUT = 0x00
CMD_ENABLE_LEDS = 0x13
CMD_SET_PWM_VALUES = 0x01
CMD_UPDATE = 0x16
PiGlow_Values = [0] * 18
PiGlow_Lookup = [0,1,2,3,14,12,17,16,15,13,11,10,6,7,8,5,4,9]
PiGlow_Brightness = 255

class PiGlow:
    i2c_addr = 0x54 # fixed i2c address of SN3218 ic
    bus = None

    def __init__(self, i2c_bus=1):
        #print "init"
        #print i2c_bus
        #self.bus = smbus.SMBus(i2c_bus)
        self.bus = SMBus(i2c_bus)
        self.enable_output()
        self.enable_leds()

    def enable_output(self):
        self.write_i2c(CMD_ENABLE_OUTPUT, 0x01)

    def enable_leds(self):
        self.write_i2c(CMD_ENABLE_LEDS, [0xFF, 0xFF, 0xFF])

    def update_pwm_values(self, values):
        #print "update pwm"
        self.write_i2c(CMD_SET_PWM_VALUES, values)
        self.write_i2c(CMD_UPDATE, 0xFF)

    def write_i2c(self, reg_addr, value):
        if not isinstance(value, list):
            value = [value];
        self.bus.write_i2c_block_data(self.i2c_addr, reg_addr, value)

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
               
piglow = None
try:
    if GPIO.RPI_REVISION == 1:
        piglow = PiGlow(0)
    else:
        piglow = PiGlow(1)
    piglow.update_pwm_values(PiGlow_Values)
except:
    print "No PiGlow Detected"
    
#See if Compass connected
compass = None
try:
    if GPIO.RPI_REVISION == 1:
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
    
#If I2C then don't uses pins 3 and 5
if ((piglow != None) or (compass != None) or (pcaPWM != None)):
    print "I2C device detected"
    PIN_NUM = array('i',[11,12,13,15,16,18,22, 7, 24,26,19,21,23, 8,10])
    PIN_USE = array('i',[ 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0])
else:
    print "No I2C Device Detected"
    PIN_NUM = array('i',[11,12,13,15,16,18,22, 7, 3, 5,24,26,19,21,23, 8,10])
    PIN_USE = array('i',[ 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0])
    

#  GPIO_NUM = array('i',[17,18,21,22,23,24,25,4,14,15,8,7,10,9])
PINS = len(PIN_NUM)
PIN_NUM_LOOKUP=[int] * 27

for i in range(PINS):
    PIN_NUM_LOOKUP[PIN_NUM[i]] = i
    print i, PIN_NUM[i]


PWM_OUT = [None] * PINS
ULTRA_IN_USE = [False] * PINS
ultraTotalInUse = 0
ultraSleep = 1.0




def isNumeric(s):
    try:
        float(s)
        return True
    except ValueError:
        return False
    
def removeNonAscii(s): return "".join(i for i in s if ord(i)<128)
    
def getValue(searchString, dataString):
    outputall_pos = dataString.find((searchString + ' '))
    sensor_value = dataString[(outputall_pos+1+len(searchString)):].split()
    return sensor_value[0]
    

def sign(number):return cmp(number,0)


def parse_data(dataraw, search_string):
    outputall_pos = dataraw.find(search_string)
    return dataraw[(outputall_pos + 1 + search_string.length):].split()
    
#Procedure to set pin mode for each pin
def SetPinMode():
    for i in range(PINS):
        if (PIN_USE[i] == 1):
            print 'setting pin' , PIN_NUM[i] , ' to out'
            GPIO.setup(PIN_NUM[i],GPIO.OUT)
        elif (PIN_USE[i] == 0):
            print 'setting pin' , PIN_NUM[i] , ' to in'
            GPIO.setup(PIN_NUM[i],GPIO.IN,pull_up_down=GPIO.PUD_UP)

        PIN_NUM_LOOKUP[PIN_NUM[i]] = i
        
#----------------------------- STEPPER CONTROL --------------
class StepperControl(threading.Thread):
    def __init__(self,pinA,pinB,pinC,pinD,step_delay):
        #self.stepper_num = stepper_num # find which stepper a or b
        #self.step_delay = step_delay
        self.stepperSpeed = 0 #stepp speed dset to 0 when thread created
        self.steps = BIG_NUM # default to psuedo infinte number of turns
        self.terminated = False
        self.toTerminate = False
        threading.Thread.__init__(self)
        self._stop = threading.Event()
        self.pins = array("i",[PIN_NUM_LOOKUP[pinA],PIN_NUM_LOOKUP[pinB],PIN_NUM_LOOKUP[pinC],PIN_NUM_LOOKUP[pinD]])
        self.slow_start = self.steps
        self.steps_start = self.steps
        self.paused = False
        self.pause_start_time = dt.datetime.now()

    def start(self):
        self.thread = threading.Thread(None, self.run, None, (), {})
        self.thread.start()


    def stop(self):
        self.toTerminate = True
        while self.terminated == False:
        # Just wait
            time.sleep(0.01)


    def changeSpeed(self, stepperSpeed,steps):
        self.stepperSpeed = int(stepperSpeed)
        self.steps = int(steps)
        self.steps_start = self.steps
        self.slow_start = self.steps - int(min(64,max(1,int(float(self.steps)*0.8))))
        if self.steps > (BIG_NUM / 2):
            self.slow_start = self.steps - 64       
            


    def index_pin_update(self, pin_index, value):
        if (PIN_USE[pin_index] == 0):
            PIN_USE[pin_index] = 1
            GPIO.setup(PIN_NUM[pin_index],GPIO.OUT)
            print 'pin' , PIN_NUM[pin_index] , ' changed to digital out from input'
        if (PIN_USE[pin_index] == 2):
            PIN_USE[pin_index] = 1
            PWM_OUT[pin_index].stop()
            GPIO.setup(PIN_NUM[pin_index],GPIO.OUT)
            print 'pin' , PIN_NUM[pin_index] , ' changed to digital out from PWM'
        if (PIN_USE[pin_index] == 1):
            #print 'setting physical pin %d to %d' % (PIN_NUM[pin_index],value)
            GPIO.output(PIN_NUM[pin_index], value)


    def step_coarse(self,a,b,c,d,delay):
        global stepType
        lstepMode = stepMode[stepType]
        #print stepMode[stepType]
        if lstepMode == '1Coil':
            self.index_pin_update(d,0)
            self.index_pin_update(a,1)
            time.sleep(delay)

            
            self.index_pin_update(b,1)
            self.index_pin_update(a,0)
            time.sleep(delay)
            
            
            self.index_pin_update(c,1)
            self.index_pin_update(b,0)
            time.sleep(delay)
            
            
            self.index_pin_update(d,1)
            self.index_pin_update(c,0)
            time.sleep(delay)
            
        elif lstepMode == '2Coil':
            self.index_pin_update(d,0)
            self.index_pin_update(c,0)
            self.index_pin_update(a,1)
            self.index_pin_update(b,1)

            time.sleep(delay)

            self.index_pin_update(a,0)
            self.index_pin_update(c,1)
            time.sleep(delay)
            
            self.index_pin_update(b,0)
            self.index_pin_update(d,1)
            time.sleep(delay)
            
            self.index_pin_update(c,0)
            self.index_pin_update(a,1)
            time.sleep(delay)
            
        elif lstepMode == 'HalfStep':
            self.index_pin_update(d,0) 
            self.index_pin_update(a,1)
            time.sleep(delay)


            self.index_pin_update(b,1)
            time.sleep(delay)

            self.index_pin_update(a,0)
            time.sleep(delay)
            
            self.index_pin_update(c,1)
            time.sleep(delay)

            self.index_pin_update(b,0)
            time.sleep(delay)

            self.index_pin_update(d,1)
            time.sleep(delay)

            self.index_pin_update(c,0)
            time.sleep(delay)
            
            self.index_pin_update(a,1)
            time.sleep(delay)

    def pause(self):
        self.index_pin_update(self.pins[0],0)
        self.index_pin_update(self.pins[1],0)
        self.index_pin_update(self.pins[2],0)
        self.index_pin_update(self.pins[3],0)
        self.paused = True
        print PIN_NUM[self.pins[0]], "pause method run"




    def run(self):
        #time.sleep(2) # just wait till board likely to be up and running
        self.pause_start_time = dt.datetime.now()
        while self.toTerminate == False:
            #print self.pins[0],self.pins[1],self.pins[2],self.pins[3]

            if (self.steps > 0):
                self.steps = self.steps - 1
                self.local_stepper_value=self.stepperSpeed # get stepper value in case its changed during this thread
                if self.local_stepper_value != 0: #if stepper_value non-zero
                    self.currentStepDelay = step_delay * 100 / abs(self.local_stepper_value)
                    if self.steps < (self.steps_start - self.slow_start) :
                        self.currentStepDelay = self.currentStepDelay *  (3.0 - (((self.steps) / float(self.steps_start - self.slow_start)))*2.0)
                        #print 2.0 - ((self.steps) / float(self.steps_start - self.slow_start))
                    if (self.slow_start < self.steps):
                        #print 2.0 - ((self.steps_start - self.steps) / float(self.steps_start - self.slow_start))
                        self.currentStepDelay = self.currentStepDelay * (3.0 - (((self.steps_start - self.steps) / float(self.steps_start - self.slow_start)))*2.0)
                    #print self.steps, self.currentStepDelay
                    if self.local_stepper_value > 0: # if positive value
                        self.step_coarse(self.pins[0],self.pins[1],self.pins[2],self.pins[3],self.currentStepDelay) #step forward
                    else:
                        self.step_coarse(self.pins[3],self.pins[2],self.pins[1],self.pins[0],self.currentStepDelay) #step forward
##                    if abs(local_stepper_value) != 100: # Only introduce delay if motor not full speed
##                        time.sleep(10*self.step_delay*((100/abs(local_stepper_value))-1))
                    self.pause_start_time = dt.datetime.now()
                    self.paused = False
                    #print PIN_NUM[self.pins[0]],self.pause_start_time
                else:
                    if ((dt.datetime.now() - self.pause_start_time).seconds > 10) and (self.paused == False):
                        self.pause()
                        #print PIN_NUM[self.pins[0]], "paused inner"
                        #print PIN_NUM[self.pins[0]], self.paused
                    #else:
                        #if self.paused == False:
                            #print PIN_NUM[self.pins[0]], "inner" ,(dt.datetime.now() - self.pause_start_time).seconds
                    time.sleep(0.1) # sleep if stepper value is zero
            else:
                if ((dt.datetime.now() - self.pause_start_time).seconds > 10) and (self.paused == False):
                    self.pause()
                    #print PIN_NUM[self.pins[0]], "paused outer"
                    #print PIN_NUM[self.pins[0]], self.paused
                #else:
                    #if self.paused == False:
                        #print PIN_NUM[self.pins[0]], "outer" ,(dt.datetime.now() - self.pause_start_time).seconds
                time.sleep(0.1) # sleep if stepper value is zero

        self.terminated = True
    ####### end of Stepper Class
                    
class PiZyPwm(threading.Thread):

  def __init__(self, frequency, gpioPin, gpioScheme):
     """
Init the PiZyPwm instance. Expected parameters are :
- frequency : the frequency in Hz for the PWM pattern. A correct value may be 100.
- gpioPin : the pin number which will act as PWM ouput
- gpioScheme : the GPIO naming scheme (see RPi.GPIO documentation)
"""
     self.baseTime = 1.0 / frequency
     self.maxCycle = 100.0
     self.sliceTime = self.baseTime / self.maxCycle
     self.gpioPin = gpioPin
     self.terminated = False
     self.toTerminate = False
     GPIO.setmode(gpioScheme)


  def start(self, dutyCycle):
    """
Start PWM output. Expected parameter is :
- dutyCycle : percentage of a single pattern to set HIGH output on the GPIO pin
Example : with a frequency of 1 Hz, and a duty cycle set to 25, GPIO pin will
stay HIGH for 1*(25/100) seconds on HIGH output, and 1*(75/100) seconds on LOW output.
"""
    self.dutyCycle = dutyCycle
    GPIO.setup(self.gpioPin, GPIO.OUT)
    self.thread = threading.Thread(None, self.run, None, (), {})
    self.thread.start()


  def run(self):
    """
Run the PWM pattern into a background thread. This function should not be called outside of this class.
"""
    while self.toTerminate == False:
      if self.dutyCycle > 0:
        GPIO.output(self.gpioPin, GPIO.HIGH)
        time.sleep(self.dutyCycle * self.sliceTime)
      
      if self.dutyCycle < self.maxCycle:
        GPIO.output(self.gpioPin, GPIO.LOW)
        time.sleep((self.maxCycle - self.dutyCycle) * self.sliceTime)

    self.terminated = True


  def changeDutyCycle(self, dutyCycle):
    """
Change the duration of HIGH output of the pattern. Expected parameter is :
- dutyCycle : percentage of a single pattern to set HIGH output on the GPIO pin
Example : with a frequency of 1 Hz, and a duty cycle set to 25, GPIO pin will
stay HIGH for 1*(25/100) seconds on HIGH output, and 1*(75/100) seconds on LOW output.
"""
    self.dutyCycle = dutyCycle


  def changeFrequency(self, frequency):
    """
Change the frequency of the PWM pattern. Expected parameter is :
- frequency : the frequency in Hz for the PWM pattern. A correct value may be 100.
Example : with a frequency of 1 Hz, and a duty cycle set to 25, GPIO pin will
stay HIGH for 1*(25/100) seconds on HIGH output, and 1*(75/100) seconds on LOW output.
"""
    self.baseTime = 1.0 / frequency
    self.sliceTime = self.baseTime / self.maxCycle


  def stop(self):
    """
Stops PWM output.
"""
    self.toTerminate = True
    while self.terminated == False:
      # Just wait
      time.sleep(0.01)
  
    GPIO.output(self.gpioPin, GPIO.LOW)
    GPIO.setup(self.gpioPin, GPIO.IN)


'''
from Tkinter import Tk
from tkSimpleDialog import askstring
root = Tk()
root.withdraw()
'''



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
        self.distarray = array('f',[0.0,0.0,0.0])


    def stop(self):
        self._stop.set()

    def stopped(self):
        return self._stop.isSet()

    def run(self):
        last_bit_pattern=0L
        for i in range(PINS):
            #print 'i %d' % i
            #print 'GPIO PIN %d' % GPIO_PIN_INPUT[i]
            if (PIN_USE[i] == 0):
                last_bit_pattern += GPIO.input(PIN_NUM[i]) << i
            #else:
                #last_bit_pattern += 1 << i
            #print 'lbp %s' % bin(last_bit_pattern)

        last_bit_pattern = last_bit_pattern ^ -1

        
        while not self.stopped():
            time.sleep(0.01) # be kind to cpu - not certain why :)
            pin_bit_pattern = 0L
            for i in range(PINS):
                if (PIN_USE[i] == 0):
                    #print 'pin' , PIN_NUM[i] , GPIO.input(PIN_NUM[i]
                    pin_bit_pattern += GPIO.input(PIN_NUM[i]) << i
                #else:
                    #pin_bit_pattern += 1 << i
            #print bin(pin_bit_pattern)
            # if there is a change in the input pins
            changed_pins = pin_bit_pattern ^ last_bit_pattern
            #print "changed pins" , bin(changed_pins)
            if changed_pins:
                #print 'pin bit pattern %d' % pin_bit_pattern

                try:
                    self.broadcast_changed_pins(changed_pins, pin_bit_pattern)
                except Exception as e:
                    print e
                    break

            last_bit_pattern = pin_bit_pattern

            if (time.time() - self.time_last_ping) > 1:

                for i in range(PINS):
                    if ULTRA_IN_USE[i] == True:
                        physical_pin = PIN_NUM[i]
                        #print 'Pinging Pin', physical_pin
                        #print PIN_USE[i]

                        ti = time.time()
                        # setup a array to hold 3 values and then do 3 distance calcs and store them
                        #print 'sonar started'
                        ts=time.time()
                        #print
                        for k in range(3):
                            #print "sonar pulse" , k
                            #GPIO.setup(physical_pin,GPIO.OUT)
                            #print physical_pin , i
                            GPIO.output(physical_pin, 1)    # Send Pulse high
                            time.sleep(0.00001)     #  wait
                            GPIO.output(physical_pin, 0)  #  bring it back low - pulse over.
                            t0=time.time() # remember current time
                            GPIO.setup(physical_pin,GPIO.IN)
                            #PIN_USE[i] = 0 don't bother telling system
                            
                            t1=t0
                            # This while loop waits for input pin (7) to be low but with a 0.04sec timeout 
                            while ((GPIO.input(physical_pin)==0) and ((t1-t0) < 0.02)):
                                #time.sleep(0.00001)
                                t1=time.time()
                            t1=time.time()
                            #print 'low' , (t1-t0).microseconds
                            t2=t1
                            #  This while loops waits for input pin to go high to indicate pulse detection
                            #  with 0.04 sec timeout
                            while ((GPIO.input(physical_pin)==1) and ((t2-t1) < 0.02)):
                                #time.sleep(0.00001)
                                t2=time.time()
                            t2=time.time()
                            #print 'high' , (t2-t1).microseconds
                            t3=(t2-t1)  # t2 contains time taken for pulse to return
                            #print "total time " , t3
                            distance=t3*343/2*100  # calc distance in cm
                            self.distarray[k]=distance
                            #print distance
                            GPIO.setup(physical_pin,GPIO.OUT)
                        tf = time.time() - ts
                        distance = sorted(self.distarray)[1] # sort the array and pick middle value as best distance
                        
                        #print "total time " , tf
                        #for k in range(5):
                            #print distarray[k]
                        #print "pulse time" , distance*58
                        #print "total time in microsecs" , (tf-ti).microseconds                    
                        # only update Scratch values if distance is < 500cm
                        if (distance > 280):
                            distance = 299
                        if (distance < 2):
                            distance = 1

                        #print'Distance:',distance,'cm'
                        sensor_name = 'ultra' + str(physical_pin)
                        if ADDON_PRESENT[2] == True:
                            if physical_pin == 13:
                                sensor_name = "ultra1"
                            if physical_pin == 7:
                                sensor_name = "ultra2"
                                    
                        bcast_str = 'sensor-update "%s" %d' % (sensor_name, distance)
                        #print 'sending: %s' % bcast_str
                        self.send_scratch_command(bcast_str)
                        self.time_last_ping = time.time()
    
            if (time.time() - self.time_last_compass) > 0.25:
                #print "time up"
                #print ADDON_PRESENT[4]
                #print compass
                #If Compass board truely present
                if ((ADDON_PRESENT[4] == True) and (compass != None)):
                    #print "compass code"
                    heading = compass.heading()
                    sensor_name = 'heading'
                    bcast_str = 'sensor-update "%s" %d' % (sensor_name, heading)
                    #print 'sending: %s' % bcast_str
                    self.send_scratch_command(bcast_str)
                self.time_last_compass = time.time()
                
                
                
            #time.sleep(1)

##            sensor_name = "turninga"
##            value = 100.0
##            bcast_str = 'sensor-update "%s" %d' % (sensor_name, value)
##            print 'sending: %s' % bcast_str
##            self.send_scratch_command(bcast_str)
            

    def broadcast_changed_pins(self, changed_pin_map, pin_value_map):
        for i in range(PINS):
            # if we care about this pin's value
            if (changed_pin_map >> i) & 0b1:
                pin_value = (pin_value_map >> i) & 0b1
                if (PIN_USE[i] == 0):
                    #print PIN_NUM[i] , pin_value
                    self.broadcast_pin_update(i, pin_value)
                                     

    def broadcast_pin_update(self, pin_index, value):
        #sensor_name = "gpio" + str(GPIO_NUM[pin_index])
        #bcast_str = 'sensor-update "%s" %d' % (sensor_name, value)
        #print 'sending: %s' % bcast_str
        #self.send_scratch_command(bcast_str)   
        if ADDON_PRESENT[1] == True:
            #do ladderboard stuff
            switch_array = array('i',[3,4,2,1])
            #switch_lookup = array('i',[24,26,19,21])
            sensor_name = "switch" + str(switch_array[pin_index-10])
        elif ADDON_PRESENT[2] == True:
            #do MotorPiTx stuff
            if PIN_NUM[pin_index] == 13:
                sensor_name = "input1"
            if PIN_NUM[pin_index] == 7:
                sensor_name = "input2"
        elif ADDON_PRESENT[6] == True:
            #do berryclip stuff
            #print PIN_NUM[pin_index]
            if PIN_NUM[pin_index] == 26:
                sensor_name = "switch"
        else:
            sensor_name = "pin" + str(PIN_NUM[pin_index])
        #if ADDON_PRESENT[5] == True:
            #print PIN_NUM[pin_index] , PIN_NUM[pin_index] in [7,8,10,22]
            #if not(PIN_NUM[pin_index] in [7,8,10,22]):
            #    return
        bcast_str = 'sensor-update "%s" %d' % (sensor_name, value)
        #print 'sending: %s' % bcast_str
        self.send_scratch_command(bcast_str)
        if ADDON_PRESENT[2] == True:
            bcast_str = 'broadcast "%s%s"' % (sensor_name,("Off","On")[value == 1])
            #print 'sending: %s' % bcast_str
            self.send_scratch_command(bcast_str)
        


    def send_scratch_command(self, cmd):
        n = len(cmd)
        a = array('c')
        a.append(chr((n >> 24) & 0xFF))
        a.append(chr((n >> 16) & 0xFF))
        a.append(chr((n >>  8) & 0xFF))
        a.append(chr(n & 0xFF))
        self.scratch_socket.send(a.tostring() + cmd)


class ScratchListener(threading.Thread):
    def __init__(self, socket):
        threading.Thread.__init__(self)
        self.scratch_socket = socket
        self._stop = threading.Event()
        self.dataraw = ''
        
    def send_scratch_command(self, cmd):
        n = len(cmd)
        a = array('c')
        a.append(chr((n >> 24) & 0xFF))
        a.append(chr((n >> 16) & 0xFF))
        a.append(chr((n >>  8) & 0xFF))
        a.append(chr(n & 0xFF))
        self.scratch_socket.send(a.tostring() + cmd)
        
    def dFind(self,searchStr):
        return (searchStr in self.dataraw)
        
    def dFindOn(self,searchStr):
        return (self.dFind(searchStr + 'on') or self.dFind(searchStr + 'high'))
        
    def dFindOff(self,searchStr):
        return (self.dFind(searchStr + 'off') or self.dFind(searchStr + 'low'))
        
    def dFindOnOff(self,searchStr):
        return (self.dFind(searchStr + 'on') or self.dFind(searchStr + 'high') 
                or self.dFind(searchStr + 'off') or self.dFind(searchStr + 'low'))

    def dRtnOnOff(self,searchStr):
        if self.dFindOn(searchStr):
            return 1
        else:
            return 0
        
    def dVFind(self,searchStr):
        return ((searchStr + ' ') in self.dataraw)
        
    def dVFindOn(self,searchStr):
        return (self.dVFind(searchStr + 'on') or self.dVFind(searchStr + 'high')or self.dVFind(searchStr + '1'))
        
    def dVFindOff(self,searchStr):
        return (self.dVFind(searchStr + 'off') or self.dVFind(searchStr + 'low') or self.dVFind(searchStr + '0'))
        
    def dVFindOnOff(self,searchStr):
        return (self.dVFindOn(searchStr) or self.dVFindOff(searchStr))

    def dVRtnOnOff(self,searchStr):
        if self.dVFindOn(searchStr):
            return 1
        else:
            return 0


    def stop(self):
        self._stop.set()

    def stopped(self):
        return self._stop.isSet()

    def index_pin_update(self, pin_index, value):
        if INVERT == True:
            if PIN_USE[pin_index] == 1:
                value = abs(value - 1)
        if (PIN_USE[pin_index] == 0):
            PIN_USE[pin_index] = 1
            GPIO.setup(PIN_NUM[pin_index],GPIO.OUT)
            print 'pin' , PIN_NUM[pin_index] , ' changed to digital out from input'
        if (PIN_USE[pin_index] == 2):
            PIN_USE[pin_index] = 1
            PWM_OUT[pin_index].stop()
            GPIO.setup(PIN_NUM[pin_index],GPIO.OUT)
            print 'pin' , PIN_NUM[pin_index] , ' changed to digital out from PWM'
        if (PIN_USE[pin_index] == 1):
            #print 'setting gpio %d (physical pin %d) to %d' % (GPIO_NUM[pin_index],PIN_NUM[pin_index],value)
            GPIO.output(PIN_NUM[pin_index], value)
            
    def index_pwm_update(self, pin_index, value):
        print "pwm changed" , value
        if PIN_USE[pin_index] != 2:
            PIN_USE[pin_index] = 2
            PWM_OUT[pin_index] = PiZyPwm(100, PIN_NUM[pin_index], GPIO.BOARD)
            PWM_OUT[pin_index].start(max(0,min(100,abs(value))))
        else:
            PWM_OUT[pin_index].changeDutyCycle(max(0,min(100,abs(value))))
            

    def step_coarse(self,a,b,c,d,delay):
        self.index_pin_update(a,1)
        self.index_pin_update(d,0)
        time.sleep(delay)

        self.index_pin_update(b,1)
        self.index_pin_update(a,0)
        time.sleep(delay)
        
        self.index_pin_update(c,1)
        self.index_pin_update(b,0)
        time.sleep(delay)
        
        self.index_pin_update(d,1)
        self.index_pin_update(c,0)
        time.sleep(delay)

    def run(self):
        global cycle_trace,turnAStep,turnBStep,turnCStep,step_delay,stepType,INVERT, \
               steppera,stepperb,stepperc,Ultra,ultraTotalInUse,piglow,PiGlow_Brightness, \
                   compass

        firstRun = False #Used for testing in overcoming Scratch "bug/feature"
        firstRunData = ''
        #This is main listening routine
        lcount = 0
        while not self.stopped():
            #lcount += 1
            #print lcount
            try:
                #print "try reading socket"
                data = self.scratch_socket.recv(BUFFER_SIZE) # get the data from the socket
                dataraw = data[4:].lower() # convert all to lowercase
                #print 'Received from scratch-Length: %d, Data: %s' % (len(dataraw), dataraw)

                if len(dataraw) > 0:
                    dataraw = ' '.join([item.replace(' ','') for item in shlex.split(dataraw)])
                    self.dataraw = dataraw
                    #print dataraw

                #print 'Cycle trace' , cycle_trace
                if len(dataraw) == 0:
                    #This is probably due to client disconnecting
                    #I'd like the program to retry connecting to the client
                    #tell outer loop that Scratch has disconnected
                    if cycle_trace == 'running':
                        cycle_trace = 'disconnected'
                        break

            except (KeyboardInterrupt, SystemExit):
                #print "reraise error"
                raise
            except socket.timeout:
                #print "No data received: socket timeout"
                continue
            except:
                print "Unknown error occured with receiving data"
                continue
            
            #print "data being processed:" , dataraw
            #This section is only enabled if flag set - I am in 2 minds as to whether to use it or not!
            if firstRun == True:
                if 'sensor-update' in dataraw:
                    #print "this data ignored" , dataraw
                    firstRunData = dataraw
                    dataraw = ''
                    firstRun = False

            #If outputs need globally inverting (7 segment common anode needs it)
            if ('invert' in dataraw):
                INVERT = True
                
            #Change pins from input to output if more needed
            if ('config' in dataraw):
                for i in range(PINS):
                    #check_broadcast = str(i) + 'on'
                    #print check_broadcast
                    physical_pin = PIN_NUM[i]
                    if 'config' + str(physical_pin)+'out' in dataraw: # change pin to output from input
                        if PIN_USE[i] == 0:                           # check to see if it is an input at moment
                            GPIO.setup(PIN_NUM[i],GPIO.OUT)           # make it an output
                            print 'pin' , PIN_NUM[i] , ' out'
                            PIN_USE[i] = 1
                    if 'config' + str(physical_pin)+'in' in dataraw:                # change pin to input from output
                        if PIN_USE[i] != 0:                                         # check to see if it not an input already
                            GPIO.setup(PIN_NUM[i],GPIO.IN,pull_up_down=GPIO.PUD_UP) # make it an input
                            print 'pin' , PIN_NUM[i] , ' in'
                            PIN_USE[i] = 0
                            
### Check for AddOn boards being declared
            for i in range(NUMOF_ADDON):
                #print "checking for " , ("addon " + ADDON[i]) 
                if ("addon " + ADDON[i]) in dataraw:
                    print "addon " + ADDON[i] + " declared"
                    ADDON_PRESENT[i] = True
                    if ADDON[i] == "ladder":

                        for k in range(0,10):
                            PIN_USE[k] = 1
                        for k in range(10,14):
                            PIN_USE[k] = 0
                        SetPinMode()

                        for k in range(1,5):
                            sensor_name = 'switch' + str(k)
                            bcast_str = 'sensor-update "%s" %d' % (sensor_name, 1)
                            #print 'sending: %s' % bcast_str
                            self.send_scratch_command(bcast_str)
                            
                    if ADDON[i] == "motorpitx":
                        #print "addon " + ADDON[i] + " declared"
                        self.index_pin_update(PIN_NUM_LOOKUP[21], 0)
                        self.index_pin_update(PIN_NUM_LOOKUP[19], 0)
                        self.index_pin_update(PIN_NUM_LOOKUP[16], 0)
                        self.index_pin_update(PIN_NUM_LOOKUP[18], 0)
                        
                        pin=PIN_NUM_LOOKUP[13]
                        PIN_USE[pin] = 0
                        GPIO.setup(13,GPIO.IN,pull_up_down=GPIO.PUD_UP)
                        
                        #setup servo2 for pin 10
                        pin=PIN_NUM_LOOKUP[10]
                        PIN_USE[pin] = 1
                        GPIO.setup(10,GPIO.OUT)
                        os.system("sudo pkill -f servodpirocon")

                        os.system('ps -ef | grep -v grep | grep "./servodmotorpitx" || ./servodmotorpitx')
                        
                    if ADDON[i] == "berry":

                        PIN_USE[PIN_NUM_LOOKUP[7]] = 1
                        PIN_USE[PIN_NUM_LOOKUP[11]] = 1
                        PIN_USE[PIN_NUM_LOOKUP[15]] = 1
                        PIN_USE[PIN_NUM_LOOKUP[19]] = 1
                        PIN_USE[PIN_NUM_LOOKUP[21]] = 1
                        PIN_USE[PIN_NUM_LOOKUP[23]] = 1
                        PIN_USE[PIN_NUM_LOOKUP[24]] = 1
                        PIN_USE[PIN_NUM_LOOKUP[26]] = 0
                        #dummy out to stop random inputs registering
                        PIN_USE[PIN_NUM_LOOKUP[3]] = 1
                        PIN_USE[PIN_NUM_LOOKUP[5]] = 1
                        PIN_USE[PIN_NUM_LOOKUP[8]] = 1
                        PIN_USE[PIN_NUM_LOOKUP[10]] = 1
                        PIN_USE[PIN_NUM_LOOKUP[22]] = 1
                        SetPinMode()
                        
                        sensor_name = 'switch'
                        bcast_str = 'sensor-update "%s" %d' % (sensor_name, 0)
                        #print 'sending: %s' % bcast_str
                        self.send_scratch_command(bcast_str)
                        
                    if ADDON[i] == "pirocon":
                        PIN_USE[PIN_NUM_LOOKUP[18]] = 1 #tilt servoA
                        PIN_USE[PIN_NUM_LOOKUP[22]] = 1 #pan servoB
                        PIN_USE[PIN_NUM_LOOKUP[19]] = 1 #MotorA 
                        PIN_USE[PIN_NUM_LOOKUP[21]] = 1 #MotorB
                        PIN_USE[PIN_NUM_LOOKUP[26]] = 1 #MotorA 
                        PIN_USE[PIN_NUM_LOOKUP[24]] = 1 #MotorB
                        PIN_USE[PIN_NUM_LOOKUP[16]] = 0 #MotorB
                        PIN_USE[PIN_NUM_LOOKUP[11]] = 0 #ObsRight
                        PIN_USE[PIN_NUM_LOOKUP[12]] = 0 #LFLeft
                        PIN_USE[PIN_NUM_LOOKUP[13]] = 0 #LFRight

                        SetPinMode()
                        os.system("sudo pkill -f servodmotorpitx")
                        os.system('ps -ef | grep -v grep | grep "./servodpirocon" || ./servodpirocon')
                        

            #Listen for Variable changes
            if 'sensor-update' in dataraw:
                #print "sensor-update rcvd" , dataraw
              
                if ADDON_PRESENT[1] == True:
                    #do ladderboard stuff

                    if (('allleds 1' in dataraw) or ('allleds on' in dataraw) or ('allleds high' in dataraw)):
                        for i in range(0, 10): # limit pins to first 10
                            self.index_pin_update(i,1)
                    if (('allleds 0' in dataraw) or ('allleds off' in dataraw) or ('allleds low' in dataraw)):
                        for i in range(0, 10): # limit pins to first 10
                            self.index_pin_update(i,0)
                    
                    for i in range(0, 10): # limit pins to first 10
                        physical_pin = PIN_NUM[i]
                        pin_string = 'led' + str(i + 1)
                        #print "pin string" , pin_string
                        if (((pin_string + ' 1' )in dataraw) or ((pin_string + ' on') in dataraw) or
                             ((pin_string + ' high') in dataraw )):
                            #print "variable detect 1/on/high" , dataraw
                            self.index_pin_update(i,1)
                        if  (((pin_string + ' 0') in dataraw) or ((pin_string + ' off') in dataraw) or
                              ((pin_string + ' low') in dataraw )):
                            #print "variable detect 0/off/low" , dataraw
                            self.index_pin_update(i,0)

                        #check for power variable commands
                        if 'power' + str(i + 1) in dataraw:
                            tempValue = getValue('power' + str(i + 1), dataraw)
                            #print 'power', str(physical_pin) , tempValue

                            if isNumeric(tempValue):
                                if PIN_USE[i] != 2:
                                    PIN_USE[i] = 2
                                    PWM_OUT[i] = PiZyPwm(100, PIN_NUM[i], GPIO.BOARD)
                                    PWM_OUT[i].start(max(0,min(100,int(float(tempValue)))))
                                else:
                                    PWM_OUT[i].changeDutyCycle(max(0,min(100,int(float(tempValue)))))
                                    
                elif ADDON_PRESENT[2] == True:
                    #do MotorPiTx stuff
                    #check for motor variable commands
                    if  'motor1 ' in dataraw:
                        i = PIN_NUM_LOOKUP[23]
                        tempValue = getValue('motor1', dataraw)
                        svalue = int(float(tempValue)) if isNumeric(tempValue) else 0
                        if svalue > 0:
                            #print "motor1 set forward" , svalue
                            self.index_pin_update(PIN_NUM_LOOKUP[21],0)
                            self.index_pin_update(PIN_NUM_LOOKUP[19],1)
                        elif svalue < 0:
                            #print "motor1 set backward", svalue
                            self.index_pin_update(PIN_NUM_LOOKUP[21],1)
                            self.index_pin_update(PIN_NUM_LOOKUP[19],0)
                        else:
                            #print "motor1 set neutral", svalue
                            self.index_pin_update(PIN_NUM_LOOKUP[21],0)
                            self.index_pin_update(PIN_NUM_LOOKUP[19],0)

                        if PIN_USE[i] != 2:
                            PIN_USE[i] = 2
                            PWM_OUT[i] = PiZyPwm(100, PIN_NUM[i], GPIO.BOARD)
                            PWM_OUT[i].start(max(0,min(100,abs(svalue))))
                        else:
                            PWM_OUT[i].changeDutyCycle(max(0,min(100,abs(svalue))))
                    
                    if  'motor2 ' in dataraw:
                        i = PIN_NUM_LOOKUP[22]
                        tempValue = getValue('motor2', dataraw)
                        svalue = int(float(tempValue)) if isNumeric(tempValue) else 0

                        if svalue > 0:
                            print "motor2 set forward" , svalue
                            self.index_pin_update(PIN_NUM_LOOKUP[18],0)
                            self.index_pin_update(PIN_NUM_LOOKUP[16],1)
                        elif svalue < 0:
                            print "motor2 set backward" , svalue
                            self.index_pin_update(PIN_NUM_LOOKUP[18],1)
                            self.index_pin_update(PIN_NUM_LOOKUP[16],0)
                        else:
                            print "motor2 set neutral" , svalue
                            self.index_pin_update(PIN_NUM_LOOKUP[18],0)
                            self.index_pin_update(PIN_NUM_LOOKUP[16],0)

                        if PIN_USE[i] != 2:
                            PIN_USE[i] = 2
                            PWM_OUT[i] = PiZyPwm(100, PIN_NUM[i], GPIO.BOARD)
                            PWM_OUT[i].start(max(0,min(100,abs(svalue))))
                        else:
                            PWM_OUT[i].changeDutyCycle(max(0,min(100,abs(svalue))))
                            
#                    if (('servo1' in dataraw)):
#                        tempValue = getValue('servo1', dataraw)
#                        svalue = int(float(tempValue)) if isNumeric(tempValue) else 180
#                        svalue= min(360,max(svalue,0))
#                        os.system("echo 0=" + str(svalue) + " > /dev/servoblaster")
                    
#                    if (('servo2' in dataraw)):
#                        print "servo2"
#                        tempValue = getValue('servo2', dataraw)
#                        svalue = int(float(tempValue)) if isNumeric(tempValue) else 180
#                        svalue= min(360,max(svalue,0))
#                        os.system("echo 1=" + str(svalue) + " > /dev/servoblaster")
                        
                    servoDict = {'servo1': '0', 'tilt': '0', 'servo2': '1', 'pan': '1' }
                    for key in servoDict:
                        #print key , servoDict[key]
                        checkStr = key
                        if self.dVFind(checkStr):
                            print key , servoDict[key]
                            tempValue = getValue(checkStr, dataraw)
                            if isNumeric(tempValue):
                                degrees = -1 * int(float(tempValue))
                                print "value" , degrees
                                #print key
                                if (key == 'servo1') or (key == 'tilt'):
                                    degrees = min(60,max(degrees,-60))
                                else:
                                    degrees = min(90,max(degrees,-90))
                                #print "convert" , degrees
                                servodvalue = 50+ ((degrees + 90) * 200 / 180)
                                print "servod", servodvalue
                                os.system("echo " + servoDict[key] + "=" + str(servodvalue) + " > /dev/servoblaster")
                            elif tempValue == "off":
                                #print key ,"servod off"
                                os.system("echo " + servoDict[key] + "=0 > /dev/servoblaster")

                        
                elif ((ADDON_PRESENT[3] == True) and (piglow != None)):
                    #do PiGlow stuff but make sure PiGlow physically detected                                  
                    #check LEDS
                    for i in range(1,19):
                        checkStr = 'led' + str(i)
                        if ((checkStr + ' ') in dataraw):
                            tempValue = getValue(checkStr, dataraw)
                            #print tempValue
                            svalue = int(float(tempValue)) if isNumeric(tempValue) else 0
                            svalue= min(255,max(svalue,0))
                            PiGlow_Values[PiGlow_Lookup[i-1]] = svalue
                            piglow.update_pwm_values(PiGlow_Values)
                    for i in range(1,4):
                        checkStr = 'leg' + str(i)
                        if ((checkStr + ' ') in dataraw):
                            tempValue = getValue(checkStr, dataraw)
                            #print tempValue
                            svalue = int(float(tempValue)) if isNumeric(tempValue) else 0
                            svalue= min(255,max(svalue,0))
                            PiGlow_Values[PiGlow_Lookup[((i-1)*6) + 0]] = svalue
                            PiGlow_Values[PiGlow_Lookup[((i-1)*6) + 1]] = svalue
                            PiGlow_Values[PiGlow_Lookup[((i-1)*6) + 2]] = svalue
                            PiGlow_Values[PiGlow_Lookup[((i-1)*6) + 3]] = svalue
                            PiGlow_Values[PiGlow_Lookup[((i-1)*6) + 4]] = svalue
                            PiGlow_Values[PiGlow_Lookup[((i-1)*6) + 5]] = svalue
                            piglow.update_pwm_values(PiGlow_Values)
                            
                        checkStr = 'arm' + str(i)
                        if ((checkStr + ' ') in dataraw):
                            tempValue = getValue(checkStr, dataraw)
                            #print tempValue
                            svalue = int(float(tempValue)) if isNumeric(tempValue) else 0
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
                        checkStr = pcolours[i]
                        if ((checkStr + ' ') in dataraw):
                            tempValue = getValue(checkStr, dataraw)
                            svalue = int(float(tempValue)) if isNumeric(tempValue) else 0
                            svalue= min(255,max(svalue,0))
                            PiGlow_Values[PiGlow_Lookup[i+0]] = svalue
                            PiGlow_Values[PiGlow_Lookup[i+6]] = svalue
                            PiGlow_Values[PiGlow_Lookup[i+12]] = svalue
                            piglow.update_pwm_values(PiGlow_Values)
                        
                            
                    #Use bit pattern to control leds
                    if 'ledpattern' in dataraw:
                        #print 'Found ledpattern'
                        num_of_bits = 18
                        bit_pattern = ('00000000000000000000000000'+getValue('ledpattern', dataraw))[-num_of_bits:]
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
                        
                    if (('bright' + ' ') in dataraw):
                        tempValue = getValue('bright', dataraw)
                        #print tempValue
                        #print isNumeric(tempValue)
                        svalue = int(float(tempValue)) if isNumeric(tempValue) else 0
                        svalue= min(255,max(svalue,0))
                        PiGlow_Brightness = svalue
                        
                elif ADDON_PRESENT[5] == True:
                    #do gPiO stuff
                    
                    if self.dVFindOnOff('allpins'):
                        for i in range(PINS): 
                            self.index_pin_update(i,self.dVRtnOnOff('allpins'))
                            
                    for i in range(PINS):
                        physical_pin = PIN_NUM[i]
                        checkStr = 'pin' + str(physical_pin)
                        if self.dVFindOnOff(checkStr):
                            self.index_pin_update(i,self.dVRtnOnOff(checkStr))
                            
                        #check for power variable commands
                        for k in ['power','motor']:
                            checkStr = k + str(physical_pin)
                            if  self.dVFind(checkStr):
                                tempValue = getValue(checkStr, dataraw)
                                svalue = int(float(tempValue)) if isNumeric(tempValue) else 0
                                self.index_pwm_update(i,svalue)
                            
                    #check for motor variable commands
                    motorList = [['motora',11,12],['motorb',13,15]]
                    #motorList = [['motora',21,26],['motorb',19,24]]
                    for listLoop in range(0,2):
                        #print motorList[listLoop]
                        checkStr = motorList[listLoop][0]
                        if self.dVFind(checkStr):
                            tempValue = getValue(checkStr, dataraw)
                            svalue = int(float(tempValue)) if isNumeric(tempValue) else 0
                            #print "svalue", svalue
                            if svalue > 0:
                                #print motorList[listLoop]
                                #print "motor set forward" , svalue
                                self.index_pin_update(PIN_NUM_LOOKUP[motorList[listLoop][2]],1)
                                self.index_pwm_update(PIN_NUM_LOOKUP[motorList[listLoop][1]],(100-svalue))
                            elif svalue < 0:
                                #print motorList[listLoop]
                                #print "motor set backward", svalue
                                self.index_pin_update(PIN_NUM_LOOKUP[motorList[listLoop][2]],0)
                                self.index_pwm_update(PIN_NUM_LOOKUP[motorList[listLoop][1]],(svalue))
                            else:
                                #print svalue, "zero"
                                self.index_pin_update(PIN_NUM_LOOKUP[motorList[listLoop][1]],0)
                                self.index_pin_update(PIN_NUM_LOOKUP[motorList[listLoop][2]],0)

                    ######### End of gPiO Variable handling
                   
                elif ADDON_PRESENT[6] == True:
                    #do BerryClip stuff
                    leds = [7,11,15,19,21,23]
                    if self.dVFindOnOff('allleds'):
                        for i in leds:
                            self.index_pin_update(PIN_NUM_LOOKUP[i],self.dVRtnOnOff('allleds'))
                    
                    for i in range(0, 6): # go thru 6 LEDS on/off
                        pin_string = 'led' + str(i + 1)
                        if self.dVFindOnOff(pin_string):
                            self.index_pin_update(PIN_NUM_LOOKUP[leds[i]],self.dVRtnOnOff(pin_string))
                            
                        #check for power variable commands
                        checkStr = 'power' + str(i + 1)
                        if  self.dVFind(checkStr):
                            tempValue = getValue(checkStr, dataraw)
                            svalue = int(float(tempValue)) if isNumeric(tempValue) else 0
                            pinIndex = PIN_NUM_LOOKUP[leds[i]]
                            if PIN_USE[pinIndex ] != 2:
                                PIN_USE[pinIndex ] = 2
                                PWM_OUT[pinIndex ] = PiZyPwm(100, PIN_NUM[pinIndex], GPIO.BOARD)
                                PWM_OUT[pinIndex ].start(max(0,min(100,svalue)))
                            else:
                                PWM_OUT[pinIndex ].changeDutyCycle(max(0,min(100,svalue)))
                            
                    if self.dVFindOnOff('buzzer'):
                        self.index_pin_update(PIN_NUM_LOOKUP[24],self.dVRtnOnOff('buzzer'))
                                                
                    for i in range(0,3):
                        led_col = ['red','yellow','green']
                        if (self.dVFindOnOff(led_col[i])):
                            self.index_pin_update(PIN_NUM_LOOKUP[leds[(i * 2)]],self.dVRtnOnOff(led_col[i]))
                            self.index_pin_update(PIN_NUM_LOOKUP[leds[(i * 2) + 1]],self.dVRtnOnOff(led_col[i]))
                            
                    for i in range(0,3):
                        led_col = ['red','yellow','green']
                        for k in range(0,2):
                            if self.dVFindOnOff(led_col[i] + str(k+1)):
                                self.index_pin_update(PIN_NUM_LOOKUP[leds[(i * 2) + k]],self.dVRtnOnOff(led_col[i] + str(k+1)))
                    ######### End of BerryClip Variable handling
                    
                elif ADDON_PRESENT[7] == True:
                    #do PiRoCon stuff
                    #check for motor variable commands
                    motorList = [['motora',21,26],['motorb',19,24]]
                    for listLoop in range(0,2):
                        #print motorList[listLoop]
                        checkStr = motorList[listLoop][0]
                        if self.dVFind(checkStr):
                            tempValue = getValue(checkStr, dataraw)
                            svalue = int(float(tempValue)) if isNumeric(tempValue) else 0
                            print "svalue", svalue
                            if svalue > 0:
                                print motorList[listLoop]
                                print "motor set forward" , svalue
                                self.index_pin_update(PIN_NUM_LOOKUP[motorList[listLoop][2]],1)
                                self.index_pwm_update(PIN_NUM_LOOKUP[motorList[listLoop][1]],(100-svalue))
                            elif svalue < 0:
                                print motorList[listLoop]
                                print "motor set backward", svalue
                                self.index_pin_update(PIN_NUM_LOOKUP[motorList[listLoop][2]],0)
                                self.index_pwm_update(PIN_NUM_LOOKUP[motorList[listLoop][1]],(svalue))
                            else:
                                #print svalue, "zero"
                                self.index_pin_update(PIN_NUM_LOOKUP[motorList[listLoop][1]],0)
                                self.index_pin_update(PIN_NUM_LOOKUP[motorList[listLoop][2]],0)
                    
                    servoDict = {'servoa': '0', 'tilt': '0', 'servob': '1', 'pan': '1' }
                    for key in servoDict:
                        #print key , servoDict[key]
                        checkStr = key
                        if self.dVFind(checkStr):
                            #print key , servoDict[key]
                            tempValue = getValue(checkStr, dataraw)
                            if isNumeric(tempValue):
                                degrees = -1 * int(float(tempValue))
                                #print "value" , degrees
                                #print key
                                if (key == 'servoa') or (key == 'tilt'):
                                    degrees = min(60,max(degrees,-80))
                                else:
                                    degrees = min(90,max(degrees,-90))
                                #print "convert" , degrees
                                servodvalue = 50+ ((degrees + 90) * 200 / 180)
                                #print "servod", servodvalue
                                os.system("echo " + servoDict[key] + "=" + str(servodvalue) + " > /dev/servoblaster")
                            elif tempValue == "off":
                                #print key ,"servod off"
                                os.system("echo " + servoDict[key] + "=0 > /dev/servoblaster")

                    if (pcaPWM != None):
                        for i in range(0, 16): # go thru servos on PCA Board
                            checkStr = 'servo' + str(i + 1) 
                            if  self.dVFind(checkStr):
                                tempValue = getValue(checkStr, dataraw)
                                svalue = int(float(tempValue)) if isNumeric(tempValue) else 180
                                #print i, svalue
                                pcaPWM.setPWM(i, 0, svalue)
                                
                        for i in range(0, 16): # go thru PowerPWM on PCA Board
                            checkStr = 'power' + str(i + 1) 
                            if  self.dVFind(checkStr):
                                tempValue = getValue(checkStr, dataraw)
                                svalue = int(float(tempValue)) if isNumeric(tempValue) else 0
                                svalue = min(4095,max(((svalue * 4096) /100),0))
                                pcaPWM.setPWM(i, 0, svalue)
                                
                    ######### End of PiRoCon Variable handling
                                                            
                else:   #normal variable processing with no add on board
                    #gloablly set all ports
                    if (('allpins 1' in dataraw) or ('allpins on' in dataraw) or ('allpins high' in dataraw)):
                        for i in range(PINS): 
                            self.index_pin_update(i,1)
                    if (('allpins 0' in dataraw) or ('allpins off' in dataraw) or ('allpins low' in dataraw)):
                        for i in range(PINS): 
                            self.index_pin_update(i,0)
                    
                    #check for individual pin on off commands
                    for i in range(PINS):
                        #check_broadcast = str(i) + 'on'
                        #print check_broadcast
                        physical_pin = PIN_NUM[i]
                        pin_string = 'pin' + str(physical_pin)
                        #print "pin string" , pin_string
                        if (((pin_string + ' 1' )in dataraw) or ((pin_string + ' on') in dataraw) or
                             ((pin_string + ' high') in dataraw )):
                            #print "variable detect 1/on/high" , dataraw
                            self.index_pin_update(i,1)
                        if  (((pin_string + ' 0') in dataraw) or ((pin_string + ' off') in dataraw) or
                              ((pin_string + ' low') in dataraw )):
                            #print "variable detect 0/off/low" , dataraw
                            self.index_pin_update(i,0)

                        #check for power variable commands
                        checkStr = 'power' + str(physical_pin)
                        if  (checkStr + ' ') in dataraw:
                            tempValue = getValue(checkStr, dataraw)
                            svalue = int(float(tempValue)) if isNumeric(tempValue) else 0
                            
                            #outputall_pos = dataraw.find('power' + str(physical_pin))
                            #sensor_value = dataraw[(outputall_pos+1+len('power' + str(physical_pin))):].split()
                            #print 'power', str(physical_pin) , sensor_value[0]

                            #if isNumeric(sensor_value[0]):
                            if PIN_USE[i] != 2:
                                PIN_USE[i] = 2
                                PWM_OUT[i] = PiZyPwm(100, PIN_NUM[i], GPIO.BOARD)
                                PWM_OUT[i].start(max(0,min(100,svalue)))
                            else:
                                PWM_OUT[i].changeDutyCycle(max(0,min(100,svalue)))
                                    
                        checkStr = 'motor' + str(physical_pin)
                        if  (checkStr + ' ') in dataraw:
                            tempValue = getValue(checkStr, dataraw)
                            svalue = int(float(tempValue)) if isNumeric(tempValue) else 0
                            
                            if PIN_USE[i] != 2:
                                PIN_USE[i] = 2
                                PWM_OUT[i] = PiZyPwm(100, PIN_NUM[i], GPIO.BOARD)
                                PWM_OUT[i].start(max(0,min(100,svalue)))
                            else:
                                PWM_OUT[i].changeDutyCycle(max(0,min(100,svalue)))
                                
                    checkStr = 'motora'
                    if  (checkStr + ' ') in dataraw:
                        #print "MotorA Received"
                        #print "stepper status" , stepperInUse[STEPPERA]
                        tempValue = getValue(checkStr, dataraw)
                        svalue = int(float(tempValue)) if isNumeric(tempValue) else 0
                        #print "MotorA" , svalue
                        if (stepperInUse[STEPPERA] == True):
                            #print "Stepper A in operation"
                            #print "send change to motora as a stepper" , sensor_value[0]
                            steppera.changeSpeed(max(-100,min(100,svalue)),2123456789)
                            
                        else:
                            i = PIN_NUM_LOOKUP[11] # assume motora is connected to Pin11
                            if PIN_USE[i] != 2:
                                PIN_USE[i] = 2
                                PWM_OUT[i] = PiZyPwm(100, PIN_NUM[i], GPIO.BOARD)
                                PWM_OUT[i].start(max(0,min(100,svalue)))
                            else:
                                PWM_OUT[i].changeDutyCycle(max(0,min(100,svalue)))


                    checkStr = 'motorb'
                    if  (checkStr + ' ') in dataraw:
                        #print "MotorA Received"
                        #print "stepper status" , stepperInUse[STEPPERA]
                        tempValue = getValue(checkStr, dataraw)
                        svalue = int(float(tempValue)) if isNumeric(tempValue) else 0
                        if (stepperInUse[STEPPERB] == True):
                            #print "Stepper B in operation"
                            #print "send change to motorb as a stepper" , sensor_value[0]
                            stepperb.changeSpeed(max(-100,min(100,svalue)),2123456789)
                            
                        else:
                            i = PIN_NUM_LOOKUP[12] # assume motorb is connected to Pin11
                            if PIN_USE[i] != 2:
                                PIN_USE[i] = 2
                                PWM_OUT[i] = PiZyPwm(100, PIN_NUM[i], GPIO.BOARD)
                                PWM_OUT[i].start(max(0,min(100,svalue)))
                            else:
                                PWM_OUT[i].changeDutyCycle(max(0,min(100,svalue)))

                    checkStr = 'motorc'
                    if  (checkStr + ' ') in dataraw:
                        #print "MotorA Received"
                        #print "stepper status" , stepperInUse[STEPPERA]
                        tempValue = getValue(checkStr, dataraw)
                        svalue = int(float(tempValue)) if isNumeric(tempValue) else 0
                        if (stepperInUse[STEPPERC] == True):
                            #print "Stepper C in operation"
                            #print "send change to motorc as a stepper" , sensor_value[0]
                            stepperc.changeSpeed(max(-100,min(100,svalue)),2123456789)
                            
                        else:
                            i = PIN_NUM_LOOKUP[13] # assume motorc is connected to Pin11
                            if PIN_USE[i] != 2:
                                PIN_USE[i] = 2
                                PWM_OUT[i] = PiZyPwm(100, PIN_NUM[i], GPIO.BOARD)
                                PWM_OUT[i].start(max(0,min(100,svalue)))
                            else:
                                PWM_OUT[i].changeDutyCycle(max(0,min(100,svalue)))


                    #Use bit pattern to control ports
                    checkStr = 'pinpattern'
                    if  (checkStr + ' ') in dataraw:
                        #print 'Found pinpattern'
                        num_of_bits = PINS
                        svalue = getValue(checkStr, dataraw)
                        #print sensor_value[0]
                        bit_pattern = ('00000000000000000000000000'+svalue)[-num_of_bits:]
                        #print 'bit_pattern %s' % bit_pattern
                        j = 0
                        for i in range(PINS):
                        #bit_state = ((2**i) & sensor_value) >> i
                        #print 'dummy pin %d state %d' % (i, bit_state)
                            if (PIN_USE[i] == 1):
                                if bit_pattern[-(j+1)] == '0':
                                    self.index_pin_update(i,0)
                                else:
                                    self.index_pin_update(i,1)
                                j = j + 1

                                           



                    checkStr = 'stepdelay'
                    if  (checkStr + ' ') in dataraw:
                        #print "MotorA Received"
                        #print "stepper status" , stepperInUse[STEPPERA]
                        tempValue = getValue(checkStr, dataraw)
                        if isNumeric(tempValue):
                            step_delay = int(float(tempValue))
                            print 'step delay changed to', step_delay

                    if  'positiona' in dataraw:
                        #print "positiona" , dataraw
                        if (stepperInUse[STEPPERA] == True):
                            outputall_pos = dataraw.find('positiona')
                            sensor_value = dataraw[(outputall_pos+1+len('positiona')):].split()
                            if isNumeric(sensor_value[0]):
                                #if int(float(sensor_value[0])) != 0:s
                                if 'steppera' in dataraw:
                                    turnAStep = int(float(sensor_value[0]))
                                else:
                                    steppera.changeSpeed(int(100 * sign(int(float(sensor_value[0])) - turnAStep)),abs(int(float(sensor_value[0])) - turnAStep))
                                    turnAStep = int(float(sensor_value[0]))
                                    #else:
                                    #    turnAStep = 0
                                                
                    if  'positionb' in dataraw:
                        #print "positionb" , dataraw
                        if (stepperInUse[STEPPERB] == True):
                            outputall_pos = dataraw.find('positionb')
                            sensor_value = dataraw[(outputall_pos+1+len('positionb')):].split()
                            #print "sensor" , sensor_value[0]
                            if isNumeric(sensor_value[0]):
                                #if int(float(sensor_value[0])) != 0:
                                if 'stepperb' in dataraw:
                                    turnBStep = int(float(sensor_value[0]))
                                    #print "stepperb found"
                                else:
                                    #print "change posb" , sensor_value[0]
                                    stepperb.changeSpeed(int(100 * sign(int(float(sensor_value[0])) - turnBStep)),abs(int(float(sensor_value[0])) - turnBStep))
                                    turnBStep = int(float(sensor_value[0]))
                                    #else:
                                    #    turnBStep = 0

                    if  'positionc' in dataraw:
                        print "positionc" , dataraw
                        if (stepperInUse[STEPPERC] == True):
                            outputall_pos = dataraw.find('positionc')
                            sensor_value = dataraw[(outputall_pos+1+len('positionc')):].split()
                            #print "sensor" , sensor_value[0]
                            if isNumeric(sensor_value[0]):
                                #if int(float(sensor_value[0])) != 0:
                                if 'stepperc' in dataraw:
                                    turnCStep = int(float(sensor_value[0]))
                                    #print "stepperb found"
                                else:
                                    #print "change posb" , sensor_value[0]
                                    stepperc.changeSpeed(int(100 * sign(int(float(sensor_value[0])) - turnCStep)),abs(int(float(sensor_value[0])) - turnCStep))
                                    turnCStep = int(float(sensor_value[0]))
                                    #else:
                                    #    turnBStep = 0
                            

### Check for Broadcast type messages being received
            if 'broadcast' in dataraw:
                #print 'broadcast in data:' , dataraw

                if ADDON_PRESENT[1] == True: # Gordon's Ladder Board

                    if (('allon' in dataraw) or ('allhigh' in dataraw)):
                        for i in range(0, 10):
                            if (PIN_USE[i] <> 0):
                                self.index_pin_update(i,1)
                    if (('alloff' in dataraw) or ('alllow' in dataraw)):
                        for i in range(0, 10):
                            if (PIN_USE[i] <> 0):
                                self.index_pin_update(i,0)
                                
                        #do ladderboard stuff
                    for i in range(0, 10):
                        #print i
                        physical_pin = PIN_NUM[i]
                        #print "pin" + str(i + 1) + "high"
                        if (('led' + str(i + 1)+'high' in dataraw) or ('led' + str(i + 1)+'on' in dataraw)):
                            #print dataraw
                            self.index_pin_update(i,1)

                        if (('led' + str(i + 1)+'low' in dataraw) or ('led' + str(i + 1)+'off' in dataraw)):
                            #print dataraw
                            self.index_pin_update(i,0)
                            
                elif ADDON_PRESENT[2] == True: # Boeeerb MotorPiTx
                    #Start using ultrasonic sensor on a pin    
                    if (('ultra1' in dataraw)):
                        physical_pin = 13
                        i = PIN_NUM_LOOKUP[physical_pin]
                        PIN_USE[i] = 0
                        GPIO.setup(physical_pin,GPIO.IN,pull_up_down=GPIO.PUD_UP)
                        #print dataraw
                        self.index_pin_update(i,1)
                        print 'start pinging on', str(physical_pin)
                        ULTRA_IN_USE[i] = True
                        
                    if (('ultra2' in dataraw)):
                        physical_pin = 7
                        i = PIN_NUM_LOOKUP[physical_pin]
                        #print dataraw
                        self.index_pin_update(i,1)
                        print 'start pinging on', str(physical_pin)
                        ULTRA_IN_USE[i] = True
                        
                elif ((ADDON_PRESENT[3] == True) and (piglow != None)): # Pimoroni PiGlow
                
                    if (('allon' in dataraw) or ('allhigh' in dataraw)):
                        for i in range(1,19):
                            PiGlow_Values[i-1] = PiGlow_Brightness
                            piglow.update_pwm_values(PiGlow_Values)
                            
                    if (('alloff' in dataraw) or ('alllow' in dataraw)):
                        for i in range(1,19):
                            PiGlow_Values[i-1] = 0
                            piglow.update_pwm_values(PiGlow_Values)
  
                    #check LEDS
                    for i in range(1,19):
                        #check_broadcast = str(i) + 'on'
                        #print check_broadcast
                        if (('led' + str(i)+'high' in dataraw) or ('led' + str(i)+'on' in dataraw)):
                            #print dataraw
                            PiGlow_Values[PiGlow_Lookup[i-1]] = PiGlow_Brightness
                            piglow.update_pwm_values(PiGlow_Values)

                        if (('led' + str(i)+'low' in dataraw) or ('led' + str(i)+'off' in dataraw)):
                            #print dataraw
                            PiGlow_Values[PiGlow_Lookup[i-1]] = 0
                            piglow.update_pwm_values(PiGlow_Values)
                            
                        if (('light' + str(i)+'high' in dataraw) or ('light' + str(i)+'on' in dataraw)):
                            #print dataraw
                            PiGlow_Values[PiGlow_Lookup[i-1]] = PiGlow_Brightness
                            piglow.update_pwm_values(PiGlow_Values)

                        if (('light' + str(i)+'low' in dataraw) or ('light' + str(i)+'off' in dataraw)):
                            #print dataraw
                            PiGlow_Values[PiGlow_Lookup[i-1]] = 0
                            piglow.update_pwm_values(PiGlow_Values)
                            
                    pcolours = ['red','orange','yellow','green','blue','white']
                    for i in range(len(pcolours)):
                        if ((pcolours[i]+'high' in dataraw) or (pcolours[i]+'on' in dataraw)):
                            #print dataraw
                            PiGlow_Values[PiGlow_Lookup[i+0]] = PiGlow_Brightness
                            PiGlow_Values[PiGlow_Lookup[i+6]] = PiGlow_Brightness
                            PiGlow_Values[PiGlow_Lookup[i+12]] = PiGlow_Brightness
                            piglow.update_pwm_values(PiGlow_Values)
                            
                        if ((pcolours[i]+'low' in dataraw) or (pcolours[i]+'off' in dataraw)):
                            #print dataraw
                            PiGlow_Values[PiGlow_Lookup[i+0]] = 0
                            PiGlow_Values[PiGlow_Lookup[i+6]] = 0
                            PiGlow_Values[PiGlow_Lookup[i+12]] = 0
                            piglow.update_pwm_values(PiGlow_Values)
                            
                    for i in range(1,4):
                        if (('leg'+str(i)+'high' in dataraw) or ('leg'+str(i)+'on' in dataraw)):
                            #print dataraw
                            PiGlow_Values[PiGlow_Lookup[((i-1)*6) + 0]] = PiGlow_Brightness
                            PiGlow_Values[PiGlow_Lookup[((i-1)*6) + 1]] = PiGlow_Brightness
                            PiGlow_Values[PiGlow_Lookup[((i-1)*6) + 2]] = PiGlow_Brightness
                            PiGlow_Values[PiGlow_Lookup[((i-1)*6) + 3]] = PiGlow_Brightness
                            PiGlow_Values[PiGlow_Lookup[((i-1)*6) + 4]] = PiGlow_Brightness
                            PiGlow_Values[PiGlow_Lookup[((i-1)*6) + 5]] = PiGlow_Brightness
                            piglow.update_pwm_values(PiGlow_Values)

                        if (('leg'+str(i)+'low' in dataraw) or ('leg'+str(i)+'off' in dataraw)):
                            #print dataraw
                            PiGlow_Values[PiGlow_Lookup[((i-1)*6) + 0]] = 0
                            PiGlow_Values[PiGlow_Lookup[((i-1)*6) + 1]] = 0
                            PiGlow_Values[PiGlow_Lookup[((i-1)*6) + 2]] = 0
                            PiGlow_Values[PiGlow_Lookup[((i-1)*6) + 3]] = 0
                            PiGlow_Values[PiGlow_Lookup[((i-1)*6) + 4]] = 0
                            PiGlow_Values[PiGlow_Lookup[((i-1)*6) + 5]] = 0
                            piglow.update_pwm_values(PiGlow_Values)
                            
                        if (('arm'+str(i)+'high' in dataraw) or ('arm'+str(i)+'on' in dataraw)):
                            #print dataraw
                            PiGlow_Values[PiGlow_Lookup[((i-1)*6) + 0]] = PiGlow_Brightness
                            PiGlow_Values[PiGlow_Lookup[((i-1)*6) + 1]] = PiGlow_Brightness
                            PiGlow_Values[PiGlow_Lookup[((i-1)*6) + 2]] = PiGlow_Brightness
                            PiGlow_Values[PiGlow_Lookup[((i-1)*6) + 3]] = PiGlow_Brightness
                            PiGlow_Values[PiGlow_Lookup[((i-1)*6) + 4]] = PiGlow_Brightness
                            PiGlow_Values[PiGlow_Lookup[((i-1)*6) + 5]] = PiGlow_Brightness
                            piglow.update_pwm_values(PiGlow_Values)

                        if (('arm'+str(i)+'low' in dataraw) or ('arm'+str(i)+'off' in dataraw)):
                            #print dataraw
                            PiGlow_Values[PiGlow_Lookup[((i-1)*6) + 0]] = 0
                            PiGlow_Values[PiGlow_Lookup[((i-1)*6) + 1]] = 0
                            PiGlow_Values[PiGlow_Lookup[((i-1)*6) + 2]] = 0
                            PiGlow_Values[PiGlow_Lookup[((i-1)*6) + 3]] = 0
                            PiGlow_Values[PiGlow_Lookup[((i-1)*6) + 4]] = 0
                            PiGlow_Values[PiGlow_Lookup[((i-1)*6) + 5]] = 0
                            piglow.update_pwm_values(PiGlow_Values)

                elif ADDON_PRESENT[6] == True: # BerryClip

                    if self.dFindOnOff('all'):
                        for i in [7,11,15,19,21,23,24]:
                            if (PIN_USE[PIN_NUM_LOOKUP[i]] <> 0):
                                self.index_pin_update(PIN_NUM_LOOKUP[i],self.dRtnOnOff('all'))
                                                                
                    leds = [7,11,15,19,21,23]
                    for i in range(0,6):
                        if self.dFindOnOff('led' + str(i + 1)):
                            self.index_pin_update(PIN_NUM_LOOKUP[leds[i]],self.dRtnOnOff('led' + str(i + 1)))

                    if self.dFindOnOff('buzzer'):
                        self.index_pin_update(PIN_NUM_LOOKUP[24],self.dRtnOnOff('buzzer'))
                                                
                    for i in range(0,3):
                        led_col = ['red','yellow','green']
                        if (self.dFindOnOff(led_col[i])):
                            self.index_pin_update(PIN_NUM_LOOKUP[leds[(i * 2)]],self.dRtnOnOff(led_col[i]))
                            self.index_pin_update(PIN_NUM_LOOKUP[leds[(i * 2) + 1]],self.dRtnOnOff(led_col[i]))
                            
                    for i in range(0,3):
                        led_col = ['red','yellow','green']
                        for k in range(0,2):
                            if self.dFindOnOff(led_col[i] + str(k+1)):
                                self.index_pin_update(PIN_NUM_LOOKUP[leds[(i * 2) + k]],self.dRtnOnOff(led_col[i] + str(k+1)))
   
                else: # Plain GPIO Broadcast processing

                    if (('allon' in dataraw) or ('allhigh' in dataraw)):
                        for i in range(PINS):
                            if (PIN_USE[i] <> 0):
                                self.index_pin_update(i,1)
                    if (('alloff' in dataraw) or ('alllow' in dataraw)):
                        for i in range(PINS):
                            if (PIN_USE[i] <> 0):
                                self.index_pin_update(i,0)
                                
                    #check pins
                    for i in range(PINS):
                        #check_broadcast = str(i) + 'on'
                        #print check_broadcast
                        physical_pin = PIN_NUM[i]
                        if (('pin' + str(physical_pin)+'high' in dataraw) or ('pin' + str(physical_pin)+'on' in dataraw)):
                            print 'pin' , physical_pin, 'on'
                            self.index_pin_update(i,1)

                        if (('pin' + str(physical_pin)+'low' in dataraw) or ('pin' + str(physical_pin)+'off' in dataraw)):
                            print 'pin' , physical_pin, 'off'
                            self.index_pin_update(i,0)

                        if ('sonar' + str(physical_pin)) in dataraw:
                            self.index_pin_update(i,1)
                            ti = time.time()
                            # setup a array to hold 3 values and then do 3 distance calcs and store them
                            #print 'sonar started'
                            distarray = array('f',[0.0,0.0,0.0])
                            ts=time.time()
                            print
                            for k in range(3):
                                #print "sonar pulse" , k
                                #GPIO.setup(physical_pin,GPIO.OUT)
                                #print physical_pin , i
                                GPIO.output(physical_pin, 1)    # Send Pulse high
                                time.sleep(0.00001)     #  wait
                                GPIO.output(physical_pin, 0)  #  bring it back low - pulse over.
                                t0=time.time() # remember current time
                                GPIO.setup(physical_pin,GPIO.IN)
                                #PIN_USE[i] = 0 don't bother telling system
                                
                                t1=t0
                                # This while loop waits for input pin (7) to be low but with a 0.04sec timeout 
                                while ((GPIO.input(physical_pin)==0) and ((t1-t0) < 0.02)):
                                    #time.sleep(0.00001)
                                    t1=time.time()
                                t1=time.time()
                                #print 'low' , (t1-t0).microseconds
                                t2=t1
                                #  This while loops waits for input pin to go high to indicate pulse detection
                                #  with 0.04 sec timeout
                                while ((GPIO.input(physical_pin)==1) and ((t2-t1) < 0.02)):
                                    #time.sleep(0.00001)
                                    t2=time.time()
                                t2=time.time()
                                #print 'high' , (t2-t1).microseconds
                                t3=(t2-t1)  # t2 contains time taken for pulse to return
                                #print "total time " , t3
                                distance=t3*343/2*100  # calc distance in cm
                                distarray[k]=distance
                                #print distance
                                GPIO.setup(physical_pin,GPIO.OUT)
                            tf = time.time() - ts
                            distance = sorted(distarray)[1] # sort the array and pick middle value as best distance
                            
                            #print "total time " , tf
                            #for k in range(5):
                                #print distarray[k]
                            #print "pulse time" , distance*58
                            #print "total time in microsecs" , (tf-ti).microseconds                    
                            # only update Scratch values if distance is < 500cm
                            if (distance > 280):
                                distance = 299
                            if (distance < 2):
                                distance = 1

                            #print'Distance:',distance,'cm'
                            sensor_name = 'sonar' + str(physical_pin)
                            bcast_str = 'sensor-update "%s" %d' % (sensor_name, distance)
                            #print 'sending: %s' % bcast_str
                            self.send_scratch_command(bcast_str)
                            
                        #Start using ultrasonic sensor on a pin    
                        if (('ultra' + str(physical_pin) in dataraw)):
                            #print dataraw
                            self.index_pin_update(i,1)
                            print 'start pinging on', str(physical_pin)
                            ULTRA_IN_USE[i] = True
#                            tempTotal = 0
#                            for k in range(PINS):
#                                if ULTRA_IN_USE[k] == True:
#                                    tempTotal += 1
#                            ultraTotalInUse = tempTotal
                         
                                      
                    #end of normal pin checking

                if 'pinpattern' in dataraw:
                    #print 'Found pinpattern broadcast'
                    #print dataraw
                    num_of_bits = PINS
                    outputall_pos = dataraw.find('pinpattern')
                    sensor_value = dataraw[(outputall_pos+10):].split()
                    sensor_value[0] = sensor_value[0][:-1]                    
                    #print sensor_value[0]
                    bit_pattern = ('00000000000000000000000000'+sensor_value[0])[-num_of_bits:]
                    #print 'bit_pattern %s' % bit_pattern
                    j = 0
                    for i in range(PINS):
                    #bit_state = ((2**i) & sensor_value) >> i
                    #print 'dummy pin %d state %d' % (i, bit_state)
                        if (PIN_USE[i] == 1):
                            if bit_pattern[-(j+1)] == '0':
                                self.index_pin_update(i,0)
                            else:
                                self.index_pin_update(i,1)
                            j = j + 1
                             

                if ('steppera' in dataraw) or ('turna' in dataraw):
                    if (stepperInUse[STEPPERA] == False):
                        print "StepperA Stasrting"
                        steppera = StepperControl(11,12,13,15,step_delay)
                        steppera.start()
                        stepperInUse[STEPPERA] = True
                        turnAStep = 0
                        steppera.changeSpeed(max(-100,min(100,int(float(0)))),2123456789)
                    else:
                        steppera.changeSpeed(max(-100,min(100,int(float(0)))),2123456789)
                        

                if ('stepperb' in dataraw):
                    if (stepperInUse[STEPPERB] == False):
                        print "StepperB Stasrting"
                        stepperb = StepperControl(16,18,22,7,step_delay)
                        stepperb.start()
                        stepperInUse[STEPPERB] = True
                        turnBStep = 0
                        stepperb.changeSpeed(max(-100,min(100,int(float(0)))),2123456789)
                    else:
                        stepperb.changeSpeed(max(-100,min(100,int(float(0)))),2123456789)
                        
                if ('stepperc' in dataraw):
                    if (stepperInUse[STEPPERC] == False):
                        print "StepperC Stasrting"
                        stepperc = StepperControl(24,26,19,21,step_delay)
                        stepperc.start()
                        stepperInUse[STEPPERC] = True
                        turnCStep = 0 #reset turn variale
                        stepperc.changeSpeed(max(-100,min(100,int(float(0)))),2123456789)
                    else:
                        stepperc.changeSpeed(max(-100,min(100,int(float(0)))),2123456789)


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
                    bcast_str = 'sensor-update "%s" %d' % ("Version", int(Version * 1000))
                    #print 'sending: %s' % bcast_str
                    self.send_scratch_command(bcast_str)

                #end of broadcast check




                        


                

            if 'stop handler' in dataraw:
                cleanup_threads((listener, sender))
                sys.exit()

            #else:
                #print 'received something: %s' % dataraw


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
    for thread in threads:
        thread.stop()

    for thread in threads:
        thread.join()

    for i in range(PINS):
        if PIN_USE[i] == 2:
            print "Stopping ", PIN_NUM[i]
            PWM_OUT[i].stop()
            print "Stopped ", PIN_NUM[i]
            
    if (stepperInUse[STEPPERA] == True):
        print "stopping stepperA"
        steppera.stop()
        print "stepperA stopped"
        
    if (stepperInUse[STEPPERB] == True):
        print "stopping stepperB"
        stepperb.stop()
        print "stepperB stopped"
            
    if (stepperInUse[STEPPERC] == True):
        print "stopping stepperC"
        stepperc.stop()
        print "stepperC stopped"

if __name__ == '__main__':
    if len(sys.argv) > 1:
        host = sys.argv[1]
    else:
        host = DEFAULT_HOST
    host = host.replace("'", "")

cycle_trace = 'start'


SetPinMode()

# setup a fade across the 18 LEDs of values ranging from 0 - 255
values = [0x01,0x02,0x04,0x08,0x10,0x18,0x20,0x30,0x40,0x50,0x60,0x70,0x80,0x90,0xA0,0xC0,0xE0,0xFF]

while True:

    if (cycle_trace == 'disconnected'):
        print "Scratch disconnected"
        cleanup_threads((listener, sender))
        os.system("sudo pkill -f servodpirocon")
        os.system("sudo pkill -f servodmotorpitx")
        time.sleep(1)
        cycle_trace = 'start'

    if (cycle_trace == 'start'):
        # open the socket
        print 'Starting to connect...' ,
        the_socket = create_socket(host, PORT)
        print 'Connected!'
        the_socket.settimeout(SOCKET_TIMEOUT)
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
        cleanup_threads((listener,sender))
        os.system("sudo pkill -f servodpirocon")
        os.system("sudo pkill -f servodmotorpitx")
        GPIO.cleanup()
        sys.exit()
        print "CleanUp complete"
        

