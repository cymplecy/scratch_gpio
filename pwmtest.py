#!/usr/bin/env python

import RPi.GPIO as GPIO
import time

GPIO.setmode(GPIO.BOARD)
GPIO.setup(19, GPIO.OUT)

p = GPIO.PWM(19, 50)
p.start(100)
time.sleep(3)
p.stop()
time.sleep(3)
p.start(20)
time.sleep(3)
p.stop()

GPIO.cleanup()

