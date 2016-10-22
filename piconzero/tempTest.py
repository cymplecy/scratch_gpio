#! /usr/bin/env python

# GNU GPL V3
# Test code for 4tronix Picon Zero

import piconzero as pz, time

pz.init()
pz.setInputConfig(0, 2)     # set input 0 to DS18B20
pz.setInputConfig(2, 2)     # set input 2 to DS18B20

try:
    while True:
        ana0 = pz.readInput(0)
        if (ana0>32767):
            ana0 -= 65536
        ana2 = pz.readInput(2)
        if (ana2>32767):
            ana2 -= 65536
        print ana0*0.0625, ana2*0.0625
        time.sleep(1)
except KeyboardInterrupt:
    print
finally:
    pz.cleanup()

