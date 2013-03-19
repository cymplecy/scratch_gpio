# This code is copyright Simon Walters under GPL v2
# This code is derived from scratch_handler by Thomas Preston
# This coe now hosted on Github thanks to Ben Nuttall
# Last change experiment adding in pyzPWM for PWM on All Pins
# Pulse is command name to use for PWM on any pin


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


def parse_data(dataraw, search_string):
    outputall_pos = dataraw.find(search_string)
    return dataraw[(outputall_pos + 1 + search_string.length):].split()

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

PORT = 42001
DEFAULT_HOST = '127.0.0.1'
#  HOST = askstring('Scratch Connector', 'IP:')
BUFFER_SIZE = 240 #used to be 100
SOCKET_TIMEOUT = 1

#  Map gpio to real connector P1 Pins
PIN_NUM = array('i',[11, 12, 13, 15, 16, 18, 22, 7, 3, 5, 8, 10, 24, 26, 19, 21, 23])
#  GPIO_NUM = array('i',[17,18,21,22,23,24,25,4,14,15,8,7,10,9])
PIN_USE = array('i',[1,  1,  1,  1,  1,  1,  0,  0, 0, 0, 0, 0,  0,  0,  0,  1,  1])
PINS = len(PIN_NUM)
sonar_listen_pin = 7
sonar_pulse_pin = 23

PINS = len(PIN_NUM)
PWM_OUT = [None] * PINS


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
        global cycle_trace
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
                        if (PIN_USE[i] == 0):
                            PIN_USE[i] = 1
                            GPIO.setup(PIN_NUM[i],GPIO.OUT)
                            print 'pin' , PIN_NUM[i] , ' used as out'
                        if (PIN_USE[i] == 2):
                            PIN_USE[i] = 1
                            PWM_OUT[i].stop()
                            GPIO.setup(PIN_NUM[i],GPIO.OUT)
                            print 'pin' , PIN_NUM[i] , '  used as out'
                        if (PIN_USE[i] == 1):
                            self.physical_pin_update(i,1)
                    if  'pin' + str(physical_pin) + '" 0' in dataraw:
                        if (PIN_USE[i] == 0):
                            PIN_USE[i] = 1
                            GPIO.setup(PIN_NUM[i],GPIO.OUT)
                            print 'pin' , PIN_NUM[i] , ' used as out'
                        if (PIN_USE[i] == 2):
                            PIN_USE[i] = 1
                            PWM_OUT[i].stop()
                            GPIO.setup(PIN_NUM[i],GPIO.OUT)
                            print 'pin' , PIN_NUM[i] , '  used as out'
                        if (PIN_USE[i] == 1):
                            self.physical_pin_update(i,1)
                    if  'pin' + str(physical_pin) + '" "on' in dataraw:
                        print 'pin13 addressed'
                        print dataraw
                        if (PIN_USE[i] == 0):
                            PIN_USE[i] = 1
                            GPIO.setup(PIN_NUM[i],GPIO.OUT)
                            print 'pin' , PIN_NUM[i] , ' used as out'
                        if (PIN_USE[i] == 2):
                            PIN_USE[i] = 1
                            PWM_OUT[i].stop()
                            GPIO.setup(PIN_NUM[i],GPIO.OUT)
                            print 'pin' , PIN_NUM[i] , '  used as out'
                        if (PIN_USE[i] == 1):
                            self.physical_pin_update(i,1)
                    if  'pin' + str(physical_pin) + '" "off' in dataraw:
                        print 'pin13 addressed'
                        print dataraw
                        if (PIN_USE[i] == 0):
                            PIN_USE[i] = 1
                            GPIO.setup(PIN_NUM[i],GPIO.OUT)
                            print 'pin' , PIN_NUM[i] , ' used as out'
                        if (PIN_USE[i] == 2):
                            PIN_USE[i] = 1
                            PWM_OUT[i].stop()
                            GPIO.setup(PIN_NUM[i],GPIO.OUT)
                            print 'pin' , PIN_NUM[i] , '  used as out'
                        if (PIN_USE[i] == 1):
                            self.physical_pin_update(i,1)
                    if  'pin' + str(physical_pin) + '" "high' in dataraw:
                        if (PIN_USE[i] == 0):
                            PIN_USE[i] = 1
                            GPIO.setup(PIN_NUM[i],GPIO.OUT)
                            print 'pin' , PIN_NUM[i] , ' used as out'
                        if (PIN_USE[i] == 2):
                            PIN_USE[i] = 1
                            PWM_OUT[i].stop()
                            GPIO.setup(PIN_NUM[i],GPIO.OUT)
                            print 'pin' , PIN_NUM[i] , '  used as out'
                        if (PIN_USE[i] == 1):
                            self.physical_pin_update(i,1)
                    if  'pin' + str(physical_pin) + '" "low' in dataraw:
                        if (PIN_USE[i] == 0):
                            PIN_USE[i] = 1
                            GPIO.setup(PIN_NUM[i],GPIO.OUT)
                            print 'pin' , PIN_NUM[i] , ' used as out'
                        if (PIN_USE[i] == 2):
                            PIN_USE[i] = 1
                            PWM_OUT[i].stop()
                            GPIO.setup(PIN_NUM[i],GPIO.OUT)
                            print 'pin' , PIN_NUM[i] , '  used as out'
                        if (PIN_USE[i] == 1):
                            self.physical_pin_update(i,1)

                    if  'pulse' + str(physical_pin) in dataraw:
                        print dataraw
                        outputall_pos = dataraw.find('pulse' + str(physical_pin))
                        sensor_value = dataraw[(outputall_pos+1+len('pulse' + str(physical_pin))):].split()
                        print 'pulse', str(physical_pin) , sensor_value[0]

                        if isNumeric(sensor_value[0]):
                            if PIN_USE[i] != 2:
                                PIN_USE[i] = 2
                                PWM_OUT[i] = PiZyPwm(100, PIN_NUM[i], GPIO.BOARD)
                                PWM_OUT[i].start(max(0,min(100,int(sensor_value[0]))))
                            else:
                                PWM_OUT[i].changeDutyCycle(max(0,min(100,int(sensor_value[0]))))
                    
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


                    
                if  'motora' in dataraw:
                    for i in range(PINS):
                        if PIN_NUM[i] == 11:
                            print dataraw
                            outputall_pos = dataraw.find('motora')
                            sensor_value = dataraw[(outputall_pos+1+len('motora')):].split()
                            print 'motorb', sensor_value[0]

                            if isNumeric(sensor_value[0]):
                                if PIN_USE[i] != 2:
                                    PIN_USE[i] = 2
                                    PWM_OUT[i] = PiZyPwm(100, PIN_NUM[i], GPIO.BOARD)
                                    PWM_OUT[i].start(max(0,min(100,int(sensor_value[0]))))
                                else:
                                    PWM_OUT[i].changeDutyCycle(max(0,min(100,int(sensor_value[0]))))

                if  'motorb' in dataraw:
                    for i in range(PINS):
                        if PIN_NUM[i] == 12:
                            print dataraw
                            outputall_pos = dataraw.find('motorb')
                            sensor_value = dataraw[(outputall_pos+1+len('motorb')):].split()
                            print 'motorb', sensor_value[0]

                            if isNumeric(sensor_value[0]):
                                if PIN_USE[i] != 2:
                                    PIN_USE[i] = 2
                                    PWM_OUT[i] = PiZyPwm(100, PIN_NUM[i], GPIO.BOARD)
                                    PWM_OUT[i].start(max(0,min(100,int(sensor_value[0]))))
                                else:
                                    PWM_OUT[i].changeDutyCycle(max(0,min(100,int(sensor_value[0]))))

                            
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
                    if (('pin' + str(physical_pin)+'high' in dataraw) or ('pin' + str(physical_pin)+'on' in dataraw)):
                        if (PIN_USE[i] == 0):
                            PIN_USE[i] = 1
                            GPIO.setup(PIN_NUM[i],GPIO.OUT)
                            print 'pin' , PIN_NUM[i] , ' used as out'
                        if (PIN_USE[i] == 2):
                            PIN_USE[i] = 1
                            PWM_OUT[i].stop()
                            GPIO.setup(PIN_NUM[i],GPIO.OUT)
                            print 'pin' , PIN_NUM[i] , '  used as out'
                        if (PIN_USE[i] == 1):
                            self.physical_pin_update(i,1)

                    if (('pin' + str(physical_pin)+'low' in dataraw) or ('pin' + str(physical_pin)+'off' in dataraw)):
                        if (PIN_USE[i] == 0):
                            PIN_USE[i] = 1
                            GPIO.setup(PIN_NUM[i],GPIO.OUT)
                            print 'pin' , PIN_NUM[i] , ' used as out'
                        if (PIN_USE[i] == 2):
                            PIN_USE[i] = 1
                            PWM_OUT[i].stop()
                            GPIO.setup(PIN_NUM[i],GPIO.OUT)
                            print 'pin' , PIN_NUM[i] , '  used as out'
                        if (PIN_USE[i] == 1):
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
        time.sleep(0.1)
    except KeyboardInterrupt:
        cleanup_threads((listener,sender,))
        sys.exit()

