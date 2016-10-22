# Picon Zero Servo Test
# Use arrow keys to move 2 servos on outputs 0 and 1 for Pan and Tilt
# Use G and H to open and close the Gripper arm
# Press Ctrl-C to stop
#

import piconzero as pz, time

#======================================================================
# Reading single character by forcing stdin to raw mode
import sys
import tty
import termios

def readchar():
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setraw(sys.stdin.fileno())
        ch = sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
    if ch == '0x03':
        raise KeyboardInterrupt
    return ch

def readkey(getchar_fn=None):
    getchar = getchar_fn or readchar
    c1 = getchar()
    if ord(c1) != 0x1b:
        return c1
    c2 = getchar()
    if ord(c2) != 0x5b:
        return c1
    c3 = getchar()
    return chr(0x10 + ord(c3) - 65)  # 16=Up, 17=Down, 18=Right, 19=Left arrows

# End of single character reading
#======================================================================

speed = 60

print "Tests the servos by using the arrow keys to control"
print "Press <space> key to centre"
print "Press Ctrl-C to end"
print

# Define which pins are the servos
pan = 0
tilt = 1
grip = 2

pz.init()

# Set output mode to Servo
pz.setOutputConfig(pan, 2)
pz.setOutputConfig(tilt, 2)
pz.setOutputConfig(grip, 2)

# Centre all servos
panVal = 90
tiltVal = 90
gripVal = 90
pz.setOutput (pan, panVal)
pz.setOutput (tilt, tiltVal)
pz.setOutput (grip, gripVal)

# main loop
try:
    while True:
        keyp = readkey()
        if keyp == 'w' or ord(keyp) == 16:
            panVal = max (0, panVal - 5)
            print 'Up', panVal
        elif keyp == 'z' or ord(keyp) == 17:
            panVal = min (180, panVal + 5)
            print 'Down', panVal
        elif keyp == 's' or ord(keyp) == 18:
            tiltVal = max (0, tiltVal - 5)
            print 'Right', tiltVal
        elif keyp == 'a' or ord(keyp) == 19:
            tiltVal = min (180, tiltVal + 5)
            print 'Left', tiltVal
        elif keyp == 'g':
            gripVal = max (0, gripVal - 5)
            print 'Open', gripVal
        elif keyp == 'h':
            gripVal = min (180, gripVal + 5)
            print 'Close', gripVal
        elif keyp == ' ':
            panVal = tiltVal = gripVal = 90
            print 'Centre'
        elif ord(keyp) == 3:
            break
        pz.setOutput (pan, panVal)
        pz.setOutput (tilt, tiltVal)
        pz.setOutput (grip, gripVal)

except KeyboardInterrupt:
    print

finally:
    pz.cleanup()
    
