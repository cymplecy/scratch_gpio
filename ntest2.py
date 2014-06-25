from __future__ import print_function       # import Python 3 print function
from nck import nunchuck
from time import sleep
import sys

lineup = "\x1b[A" * 5   # set lineup to 5 "up one line" ASCII esc chars

wii = nunchuck()

while True:
    print ('Joystick x: ',"{0:03d}".format(wii.joystick()[0]),
           '  y: ', "{0:03d}".format(wii.joystick()[1]))

    print ("Accelerometer x: ", "{0:03d}".format((wii.accelerometer()[0])),
           " y: ", "{0:03d}".format((wii.accelerometer()[1])),
           " z: ", "{0:03d}".format((wii.accelerometer()[2])))
    if wii.button_c():
        print ("Button C:   pressed")
    else:
        print ("Button C: unpressed")
    if wii.button_z():
        print ("Button Z:   pressed")
    else:
        print ("Button Z: unpressed")
    print ("                                           CTRL+C to Exit",'\r')
    sys.stdout.write(lineup)      # Move the cursor up 5 lines
    sleep(0.05)

