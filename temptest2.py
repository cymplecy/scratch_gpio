#!/usr/bin/env python

import time
import os

os.system('sudo modprobe w1-gpio')
os.system('sudo modprobe  w1-therm')
possSensors  = os.listdir('/sys/bus/w1/devices')
#print possSensors
dsSensorId = "28"
for loop in possSensors:
    if loop[:2] == "28":
        dsSensorId = loop

temperatures = []

for polltime in range(0,5):
        tfile = open("/sys/bus/w1/devices/"+ dsSensorId +"/w1_slave")
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
        # Put the decimal point in the right place and display it.
        
        temperatures.append(temperature )
        print temperature
       # time.sleep(1)
temperatures = sorted(temperatures)
#del temperatures[6]
#del temperatures[0]
avgtemperature =(sum(temperatures) / float(len(temperatures)))
        
print "average" , avgtemperature
