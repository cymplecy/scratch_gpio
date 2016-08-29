#! /usr/bin/env python

# GNU GPL V3
# Test code for 4tronix Picon Zero

import piconzero as pz, time

lastPix = 0
numpixels = 8

pz.init()
pz.setInputConfig(0, 1)     # set input 0 to Analog
pz.setOutputConfig(0, 1)    # set output 0 to PWM
pz.setOutputConfig(2, 2)    # set output 2 to Servo
pz.setOutputConfig(5, 3)    # set output 5 to WS2812
rev = pz.getRevision()
print rev[0], rev[1]
try:
    while True:
        ana0 = pz.readInput(0)
        #print ana0, int(0.5 + ana0/113.7), int(ana0/7)
        pz.setOutput(0, ana0/10)
        #setMotor(0, ana0/10)
        if (int(0.4 + ana0*numpixels/1000) != lastPix):
            lastPix = int(0.5 + ana0*numpixels/1000)
            for i in range (64):
                if i < lastPix:
                    pz.setPixel(i, 0, 0, 255, False)
                else:
                    pz.setPixel(i, 0, 0, 0, False)
        pz.updatePixels()
        pz.setOutput(2, int(ana0/7))
        time.sleep(0.1)
except KeyboardInterrupt:
    print
finally:
    pz.cleanup()

