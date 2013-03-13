# This code is copyright Simon Walters under GPL v2
# This code is derived from scratch_handler by Thomas Preston
# Version 5dev 11Aug08 Much better looping supplied by Stein @soilandreyes
# and someone else @MCrRaspJam who've name I've forgotton!
# Version 6dev - Moved Allon/AllOff to be processed before single pins :)
# Vesion 7dev - start to tidy up changes
# Vesion 8dev - use gpio-output system and broadcast allon, 1on system
# V0.1 - change to 6 out 2 in and sanitise the code
# V0.2a - use global variable to trace Scratch disconnect
# V0.3a -   Change to Broadcom GPIO numbering for variables
#           Handle pin broadcasts correctly
# V0.4 - add in more gpio name variants that it can handle
# V0.5 - Use Pinon/off as well as high low and also gpio variables can
#       use high/low on/off as well
# V0.7 - Add in MotorA and MotorB (pin11 and 12) PWM control.
# V0.8b - Add in all pins and try to make in/out configurable and fix input bug
# v0.9c Ultrasonic and better handling of motor variable values
# v1.0  Tidy up Ultrasonic
# V1.1 Change to using distance for ultrasonic
# V1.2 Changes to handle pin configuration
# V1.3 Adding comments and defaulting sonar to pulse on pin23, listen on pin7 


from array import *
import threading
import socket
import time
import sys
import struct
import datetime as dt

import RPi.GPIO as GPIO
GPIO.setmode(GPIO.BOARD)
GPIO.setwarnings(False)
GPIO.cleanup()


def isNumeric(s):
    try:
        float(s)
        return True
    except ValueError:
        return False

def parse_data(dataraw,search_string):
    outputall_pos = dataraw.find(search_string)
    return dataraw[(outputall_pos + 1 + search_string.length):].split()



'''
from Tkinter import Tk
from tkSimpleDialog import askstring
root = Tk()
root.withdraw()
'''

PORT = 42001
DEFAULT_HOST = '127.0.0.1'
#HOST = askstring('Scratch Connector', 'IP:')
BUFFER_SIZE = 240 #used to be 100
SOCKET_TIMEOUT = 1

#Map gpio to real connector P1 Pins
PIN_NUM = array('i',[11,12,13,15,16,18,22,7,3,5,8,10,24,26,19,21,23])
#GPIO_NUM = array('i',[17,18,21,22,23,24,25,4,14,15,8,7,10,9])
PIN_USE = array('i',[1,1,1,1,1,1,0,0,0,0,0,0,0,0,0,1,1])
PINS = len(PIN_NUM)
sonar_listen_pin=7
sonar_pulse_pin=23

#Procedure to set pin mode for each pin
def SetPinMode():
    for i in range(PINS):
        if (PIN_USE[i] == 1):
            GPIO.setup(PIN_NUM[i],GPIO.OUT)
            print 'pin' , PIN_NUM[i] , ' out'
        else:
            GPIO.setup(PIN_NUM[i],GPIO.IN,pull_up_down=GPIO.PUD_UP)
            print 'pin' , PIN_NUM[i] , ' in'

SetPinMode()

GPIO.setup(sonar_pulse_pin,GPIO.OUT)



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
                    #print 'pin' , PIN_NUM[i]
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
            

    def broadcast_changed_pins(self, changed_pin_map, pin_value_map):
        for i in range(PINS):
            # if we care about this pin's value
            if (changed_pin_map >> i) & 0b1:
                pin_value = (pin_value_map >> i) & 0b1
                if (PIN_USE[i] == 0):
                    self.broadcast_pin_update(i, pin_value)

    def broadcast_pin_update(self, pin_index, value):
        #sensor_name = "gpio" + str(GPIO_NUM[pin_index])
        #bcast_str = 'sensor-update "%s" %d' % (sensor_name, value)
        #print 'sending: %s' % bcast_str
        #self.send_scratch_command(bcast_str)
        sensor_name = "pin" + str(PIN_NUM[pin_index])
        bcast_str = 'sensor-update "%s" %d' % (sensor_name, value)
        print 'sending: %s' % bcast_str
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
        
    def send_scratch_command(self, cmd):
        n = len(cmd)
        a = array('c')
        a.append(chr((n >> 24) & 0xFF))
        a.append(chr((n >> 16) & 0xFF))
        a.append(chr((n >>  8) & 0xFF))
        a.append(chr(n & 0xFF))
        self.scratch_socket.send(a.tostring() + cmd)


    def stop(self):
        self._stop.set()

    def stopped(self):
        return self._stop.isSet()

    def physical_pin_update(self, pin_index, value):
        if (PIN_USE[pin_index] == 1):
            #print 'setting gpio %d (physical pin %d) to %d' % (GPIO_NUM[pin_index],PIN_NUM[pin_index],value)
            GPIO.output(PIN_NUM[pin_index], value)

    def run(self):
        global cycle_trace,motorA,motorB
        #This is main listening routine
        while not self.stopped():
            try:
                data = self.scratch_socket.recv(BUFFER_SIZE)
                dataraw = data[4:].lower()
                #print 'Length: %d, Data: %s' % (len(dataraw), dataraw)
                #print 'Cycle trace' , cycle_trace
                if len(dataraw) == 0:
                    #This is probably due to client disconnecting
                    #I'd like the program to retry connecting to the client
                    #tell outer loop that Scratch has disconnected
                    if cycle_trace == 'running':
                        cycle_trace = 'disconnected'
                        break

            except socket.timeout:
                #print "No data received: socket timeout"
                continue

            if 'sensor-update' in dataraw:
                #gloablly set all ports
                if 'allpins" 1' in dataraw:
                    for i in range(PINS):
                        if (PIN_USE[i] == 1):
                            self.physical_pin_update(i,1)
                if 'allpins" 0' in dataraw:
                    for i in range(PINS):
                        if (PIN_USE[i] == 1):
                            self.physical_pin_update(i,0)
                if 'allpins" "on' in dataraw:
                    for i in range(PINS):
                        if (PIN_USE[i] == 1):
                            self.physical_pin_update(i,1)
                if 'allpins" "off' in dataraw:
                    for i in range(PINS):
                        if (PIN_USE[i] == 1):
                            self.physical_pin_update(i,0)
                if 'allpins" "high' in dataraw:
                    for i in range(PINS):
                        if (PIN_USE[i] == 1):
                            self.physical_pin_update(i,1)
                if 'allpins" "low' in dataraw:
                    for i in range(PINS):
                        if (PIN_USE[i] == 1):
                            self.physical_pin_update(i,0)
                
                
                #check for individual port commands
                for i in range(PINS):
                    #check_broadcast = str(i) + 'on'
                    #print check_broadcast
                    physical_pin = PIN_NUM[i]
                    if 'pin' + str(physical_pin) + '" 1' in dataraw:
                        if (PIN_USE[i] == 1):
                            self.physical_pin_update(i,1)
                    if  'pin' + str(physical_pin) + '" 0' in dataraw:
                        if (PIN_USE[i] == 1):
                            self.physical_pin_update(i,0)
                    if  'pin' + str(physical_pin) + '" "on' in dataraw:
                        if (PIN_USE[i] == 1):
                            self.physical_pin_update(i,1)
                    if  'pin' + str(physical_pin) + '" "off' in dataraw:
                        if (PIN_USE[i] == 1):
                            self.physical_pin_update(i,0)
                    if  'pin' + str(physical_pin) + '" "high' in dataraw:
                        if (PIN_USE[i] == 1):
                            self.physical_pin_update(i,1)
                    if  'pin' + str(physical_pin) + '" "low' in dataraw:
                        if (PIN_USE[i] == 1):
                            self.physical_pin_update(i,0)
                        
                #Use bit pattern to control ports
                if 'pinpattern' in dataraw:
                    #print 'Found pinpattern'
                    num_of_bits = PINS
                    outputall_pos = dataraw.find('pinpattern')
                    sensor_value = dataraw[(outputall_pos+12):].split()
                    #print sensor_value[0]
                    bit_pattern = ('00000000000000000000000000'+sensor_value[0])[-num_of_bits:]
                    #print 'bit_pattern %s' % bit_pattern
                    j = 0
                    for i in range(PINS):
                    #bit_state = ((2**i) & sensor_value) >> i
                    #print 'dummy pin %d state %d' % (i, bit_state)
                        if (PIN_USE[i] == 1):
                            if bit_pattern[-(j+1)] == '0':
                                self.physical_pin_update(i,0)
                            else:
                                self.physical_pin_update(i,1)
                            j = j + 1

                #Check for motor commands
                if 'motora' in dataraw:
                    outputall_pos = dataraw.find('motora')
                    sensor_value = dataraw[(outputall_pos+7):].split()
##                    print
##                    print "sensor_value" , sensor_value[0]
##                    print
                    
                    if isNumeric(sensor_value[0]):
                        motorA = max(0,min(100,int(sensor_value[0])))
                    
                if  'motorb' in dataraw:
                    outputall_pos = dataraw.find('motorb')
                    sensor_value = dataraw[(outputall_pos+7):].split()
                    if isNumeric(sensor_value[0]):
                        motorB = max(0,min(100,int(sensor_value[0])))
                    #print "motorB" , motorB
        

                            
            if 'broadcast' in dataraw:
                #print 'received broadcast: %s' % data
                if (('allon' in dataraw) or ('allhigh' in dataraw)):
                    for i in range(PINS):
                        if (PIN_USE[i] == 1):
                            self.physical_pin_update(i,1)
                if (('alloff' in dataraw) or ('alllow' in dataraw)):
                    for i in range(PINS):
                        if (PIN_USE[i] == 1):
                            self.physical_pin_update(i,0)
                for i in range(PINS):
                    #check_broadcast = str(i) + 'on'
                    #print check_broadcast
                    physical_pin = PIN_NUM[i]
                    if 'pin' + str(physical_pin)+'high' in dataraw:
                        self.physical_pin_update(i,1)
                    if 'pin' + str(physical_pin)+'low' in dataraw:
                        self.physical_pin_update(i,0)
                    if 'pin' + str(physical_pin)+'on' in dataraw:
                        self.physical_pin_update(i,1)
                    if 'pin' + str(physical_pin)+'off' in dataraw:
                        self.physical_pin_update(i,0)

                if 'sonar' in dataraw:
                    # setup a array to hold 3 values and then do 3 distance calcs and store them
                    distarray = array('i',[0,0,0])
                    for k in range(3):
                        #print "sonar pulse" , physical_pin
                        GPIO.output(sonar_pulse_pin, True)   # Send Pulse pin(23) high
                        time.sleep(0.00001)     #  wait
                        GPIO.output(sonar_pulse_pin, False)  #  bring it back low - pulse over.
                        t0=dt.datetime.now() # rememeber current time
                        t1=t0
                        # This while loop waits for input pin (7) to be low but with a 0.1sec timeout 
                        while ((GPIO.input(sonar_listen_pin)==False) and ((t1-t0).microseconds < 100000)):
                            t1=dt.datetime.now()
                        t1=dt.datetime.now()
                        t2=t1
                        #  This while loops waits for input pin to go high to indicate ulse detection
                        #  with 0.1 sec timeout
                        while ((GPIO.input(sonar_listen_pin)==True) and ((t2-t1).microseconds < 100000)):
                            t2=dt.datetime.now()
                        t2=dt.datetime.now()
                        t3=(t2-t1).microseconds  # t2 contains time taken for pulse to return
                        #print "t3 in secs" , float(t3/1000000.0)
                        distance=t3/58  # calc distance in cm
                        distarray[k]=distance
                    distance = sorted(distarray)[1] # sort the array and pick middle value as best distance
                    # only update Scratch values if distance is < 500cm
                    if (distance < 500):# and (distance > 4):
                        #print'Distance:',distance,'cm'
                        sensor_name = "sonar" 
                        bcast_str = 'sensor-update "%s" %d' % (sensor_name, distance)
                        #print 'sending: %s' % bcast_str
                        self.send_scratch_command(bcast_str)
                         
                                
                if ('config' in dataraw):
                    for i in range(PINS):
                        #check_broadcast = str(i) + 'on'
                        #print check_broadcast
                        physical_pin = PIN_NUM[i]
                        if 'config' + str(physical_pin)+'out' in dataraw:
                            PIN_USE[i] = 1
                        if 'config' + str(physical_pin)+'in' in dataraw:
                            PIN_USE[i] = 0
                    SetPinMode()
                    
                

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

if __name__ == '__main__':
    if len(sys.argv) > 1:
        host = sys.argv[1]
    else:
        host = DEFAULT_HOST

cycle_trace = 'start'
motorA = 0;
motorB = 0;
motor_timing = array('i',[0,0,100])
motor_order = array('i',[0,1])
while True:

    if (cycle_trace == 'disconnected'):
        print "Scratch disconnected"
        cleanup_threads((listener, sender))
        time.sleep(1)
        cycle_trace = 'start'

    if (cycle_trace == 'start'):
        # open the socket
        print 'Starting to connect...' ,
        the_socket = create_socket(host, PORT)
        print 'Connected!'
        the_socket.settimeout(SOCKET_TIMEOUT)
        listener = ScratchListener(the_socket)
##        data = the_socket.recv(BUFFER_SIZE)
##        print "Discard 1st data buffer" , data[4:].lower()
        sender = ScratchSender(the_socket)
        cycle_trace = 'running'
        print "Running...."
        listener.start()
        sender.start()

    # wait for ctrl+c
    try:
        #just pause
        #print "motorA val:" , motorA

        if ((motorA > 0) or (motorB > 0)):
            if (motorA > motorB):
                motor_order[0]=0
                motor_order[1]=1
                #time before motorB goes off
                motor_timing[0]=motorB
                motor_timing[1]=motorA-motorB
                motor_timing[2]=100-motorA
            else:
                motor_order[0]=1
                motor_order[1]=0
                #time before motorA goes off
                motor_timing[0]=motorA
                motor_timing[1]=motorB-motorA
                motor_timing[2]=100-motorB


            #print 't0 t1 t2', motor_timing[0], motor_timing[1] , motor_timing[2]                
            #print 'pin: ' , GPIO_PINS[0]
            GPIO.output(PIN_NUM[motor_order[0]], 1)
            if (motor_timing[0] > 0 ):
                #print 'pin: ' , PIN_NUM[motor_order[1]]
                GPIO.output(PIN_NUM[motor_order[1]], 1)
                time.sleep(motor_timing[0]/10000.0)
            if (motor_timing[0] > 0 ):
                GPIO.output(PIN_NUM[motor_order[1]], 0)
            time.sleep(motor_timing[1]/10000.0)
            GPIO.output(PIN_NUM[motor_order[0]], 0)
            time.sleep(motor_timing[2]/10000.0)            
        else:
            time.sleep(0.1)
    except KeyboardInterrupt:
        cleanup_threads((listener,sender))
        sys.exit()

