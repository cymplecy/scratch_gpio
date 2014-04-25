# Testing PCF8591 ADC Chip

import time
import sys
import RPi.GPIO as gpio
from sgh_PCF8591P import sgh_PCF8591P

pcfSensor = None
try:
    pcfSensor = sgh_PCF8591P(0) #i2c, 0x48)
    print pcfSensor
    print "PCF8591P Detected"
except:
    print "No PCF8591 Detected"
    
for i in range(5):
    adc = pcfSensor.readADC(0) # get a value
    print'ADC1:',adc
    time.sleep(1)

