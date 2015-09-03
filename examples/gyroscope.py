#!/usr/bin/python
import time
from sense_hat import SenseHat

sense = SenseHat()

while True:
    gyroscope = sense.get_gyroscope()
    print "gyroscope", gyroscope
    print
    time.sleep(0.1)

