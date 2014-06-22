from nunchuck import nunchuck
import time as time
wii = nunchuck()
while 1:
    print wii.raw()    # Returns just the X and Y positions of the joystick

