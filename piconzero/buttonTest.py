#! /usr/bin/env python

# GNU GPL V3
# Test code for 4tronix Picon Zero

import piconzero as pz, time

pz.init()

pz.setInputConfig(0, 0, True)   # request pullup on input

vsn = pz.getRevision()
if (vsn[1] == 2):
    print "Board Type:", "Picon Zero"
else:
    print "Board Type:", vsn[1]
print "Firmware version:", vsn[0]
print

try:
    while True:
        switch = pz.readInput(0)
        if (switch == 0):
            print "Switch Pressed", switch
        else:
            print "Switch Released", switch
        time.sleep(5)
except KeyboardInterrupt:
    print
finally:
    pz.cleanup()

