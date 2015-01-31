#!/usr/bin/env python
import RPi.GPIO as GPIO
import time as time

GPIO.setmode(GPIO.BOARD)
GPIO.setwarnings(False)

GPIO.setup(7,GPIO.IN)

while True:
    print GPIO.input(7)
    time.sleep(1)