#!/usr/bin/env python
#Original Code Martin Bateman 2013
#Modified by Simon Walters
#GPLv2 applies
#V0.2 10Aug13

import sys
from socket import *
from subprocess import Popen, call
import shlex
import os
import sys
import time

def getserial():
  # Extract serial from cpuinfo file
  cpuserial = "0000000000000000"
  try:
    f = open('/proc/cpuinfo','r')
    for line in f:
      if line[0:6]=='Serial':
        cpuserial = line[10:26]
    f.close()
  except:
    cpuserial = "ERROR000000000"

  return cpuserial
  
myserial = getserial()
print myserial
print myserial[-4:]
s = socket(AF_INET, SOCK_DGRAM)
s.bind(('', 50000))
s.settimeout(300)

os.system("echo none >/sys/class/leds/led0/trigger")
os.system("echo 0 >/sys/class/leds/led0/brightness")
time.sleep(5)


while 1:
    try:
        #print "try reading socket"
        os.system("echo 1 >/sys/class/leds/led0/brightness")
        
        data, wherefrom = s.recvfrom(1500, 0) # get the data from the socket

    except (KeyboardInterrupt, SystemExit):
        #print "reraise error"
        raise
    except timeout:
        print "No data received: socket timeout"
        #print sys.exc_info()[0]
        break
    except:
        print "Unknown error occured with receiving data"
        break    

    print (data + " " + repr(wherefrom[0]))

    if (data.find("Start SID" + myserial[-4:]) != -1):

#        call(['sudo', 'python', '/home/pi/simplesi_scratch_handler/scratch_gpio_handler2.py', str(repr(wherefrom[0]))],shell=True)
        os.system('sudo python /home/pi/simplesi_scratch_handler/scratch_gpio_handler2.py '+ str(repr(wherefrom[0])) +' &')
        for i in range (0,20):
            os.system("echo 0 >/sys/class/leds/led0/brightness")
            time.sleep(0.2)
            os.system("echo 1 >/sys/class/leds/led0/brightness")
            time.sleep(0.2)
        break

#for i in range (0,10):
#    os.system("echo 0 >/sys/class/leds/led0/brightness")
#    time.sleep(1)
#    os.system("echo 1 >/sys/class/leds/led0/brightness")
#    time.sleep(1)
os.system("echo mmc0 >/sys/class/leds/led0/trigger")
s.close()
sys.exit()