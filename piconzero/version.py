#! /usr/bin/env python

# GNU GPL V3
# Test code for 4tronix Picon Zero

import piconzero as pz

pz.init()
vsn = pz.getRevision()
if (vsn[1] == 2):
    print "Board Type:", "Picon Zero"
else:
    print "Board Type:", vsn[1]
print "Firmware version:", vsn[0]
print
pz.cleanup()

