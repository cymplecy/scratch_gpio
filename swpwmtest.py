#!/usr/bin/env python
import RPi.GPIO as GPIO
import time
import datetime as dt

GPIO.setmode(GPIO.BOARD)
GPIO.setwarnings(False)


#PWM pin
GPIO.setup(37,GPIO.OUT)


count = 0
while True:
    pinRef = GPIO.PWM(37,50) # create new PWM instance
    pinRef.start(10) # update PWM value    
    time.sleep(0.05)
    pinRef.stop()
    GPIO.output(37,0)
    time.sleep(0.05)
    count = count + 1
    print count
