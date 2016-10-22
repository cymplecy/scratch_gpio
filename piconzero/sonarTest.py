#! /usr/bin/env python
#
# Basic test of HC-SR04 ultrasonic sensor on Picon Zero

import hcsr04, time

hcsr04.init()

try:
    while True:
        distance = int(hcsr04.getDistance())
        print "Distance:", distance
        time.sleep(1)
except KeyboardInterrupt:
    print
finally:
    hcsr04.cleanup()

