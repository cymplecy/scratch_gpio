#!/usr/bin/env python
#Original Code Martin Bateman 2013
#Modified by Simon Walters
#GPLv2 applies
#V0.22 14Sep13

import sys
from socket import *
import subprocess
#from subprocess import Popen, call
import shlex
import os
import sys
import time
import threading
import getopt
import shlex

class Blink(threading.Thread):

    def __init__(self, on,off):
        super(Blink, self).__init__()
        self.on = on
        self.off = off
        self.terminated = False
        self.toTerminate = False

    def start(self):
        self.thread = threading.Thread(None, self.run, None, (), {})
        self.thread.start()


    def run(self):
        while self.toTerminate == False:
            os.system("echo 0 >/sys/class/leds/led0/brightness")
            time.sleep(self.on )
            os.system("echo 1 >/sys/class/leds/led0/brightness")
            time.sleep(self.off )
        self.terminated = True

    def stop(self):
        self.toTerminate = True
        while self.terminated == False:
            # Just wait
            time.sleep(0.01)
            
    def set_delays(self,on,off):
        self.on = on
        self.off = off
        
            



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

if __name__ == "__main__":
   timeout = '300'
   try:
      opts, args = getopt.getopt(sys.argv[1:],"ht:",["timeout="])
   except getopt.GetoptError:
      print 'sudo sid.py -t <timeout>'
      sys.exit(2)
   for opt, arg in opts:
      if opt == '-h':
         print 'sudo sid.py -t <timeout>'
         sys.exit()
      elif opt in ("-t", "--timeout"):
         timeout = arg
         timeout = timeout.strip()
   print 'Timeout is:', timeout


   
s = socket(AF_INET, SOCK_DGRAM)
s.bind(('', 50000))
s.settimeout(int(float(timeout)))

os.system("echo none >/sys/class/leds/led0/trigger")

blinkthread = Blink(0.3,0.3)       
blinkthread.start()

    #blinkthread.join()
    
while 1:
    try:
        #print "try reading socket"
        os.system("echo 1 >/sys/class/leds/led0/brightness")
        
        data, wherefrom = s.recvfrom(1500, 0) # get the data from the socket

    except (KeyboardInterrupt, SystemExit):
        print "Program Ending - please wait a few secs"
        break
    except timeout:
        print "No data received: socket timeout"
        #print sys.exc_info()[0]
        break
    except:
        print "Unknown error occured with receiving data"
        break    

    print (data + " " + repr(wherefrom[0]))

    if (data.find("Start SID" + myserial[-4:]) != -1):
        #os.system('sudo python /home/pi/simplesi_scratch_handler/scratch_gpio_handler2.py '+ str(repr(wherefrom[0])) +' &')
        #print shlex.split("""x-terminal-emulator -e 'bash -c "sudo python /home/pi/simplesi_scratch_handler/scratch_gpio_handler2.py """ + str(repr(wherefrom[0])) + """"'""")
        process = subprocess.Popen(shlex.split("""x-terminal-emulator -e 'bash -c "sudo python /home/pi/simplesi_scratch_handler/scratch_gpio_handler2.py """ + str(repr(wherefrom[0])) + """"'"""), stdout=subprocess.PIPE)
       
        blinkthread.set_delays(1,1)
        time.sleep(30)
        break


try:
    blinkthread.stop()
except:
    pass
time.sleep(1)
os.system("echo 0 >/sys/class/leds/led0/brightness")
time.sleep(1)
os.system("echo mmc0 >/sys/class/leds/led0/trigger")
s.close()
sys.exit()