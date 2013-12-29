#!/usr/bin/env python
import time,  RPi.GPIO as GPIO


GPIO.setwarnings(False)


GPIO.setmode(GPIO.BOARD)
GPIO.setup(11,GPIO.OUT)
GPIO.setup(12,GPIO.OUT)

while 1:
    GPIO.output(11, 1)
    GPIO.output(12, 1)
    time.sleep(2)
    GPIO.output(11, 0)
    GPIO.output(12, 0)    
    time.sleep(2)
    



