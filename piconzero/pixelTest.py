#! /usr/bin/env python

# GNU GPL V3
# Test code for 4tronix Picon Zero

import piconzero as pz, time

pz.init()
pz.setOutputConfig(5, 3)    # set output 5 to WS2812
rev = pz.getRevision()
print rev[0], rev[1]
try:
    while True:
        pz.setAllPixels(255,255,255)
        time.sleep(1)
        pz.setAllPixels(0,0,0)
        time.sleep(1)
except KeyboardInterrupt:
    print
finally:
    pz.cleanup()

