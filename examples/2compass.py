#!/usr/bin/python
import time
from sense_hat import SenseHat

sense = SenseHat()

while True:
    compass = sense.get_compass()
    print "compass", compass
    print
    time.sleep(0.1)

