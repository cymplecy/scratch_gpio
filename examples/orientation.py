#!/usr/bin/python
import time
from sense_hat import SenseHat

sense = SenseHat()

while True:
    orientation = sense.get_orientation()
    print "pitch", orientation["pitch"]
    print "roll", orientation["roll"]
    print
    time.sleep(0.1)

