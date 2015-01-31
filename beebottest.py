#!/usr/bin/env python

# Pi2Go Wheel Sensor Test (only if addon is fitted)
# Moves: Forward, Reverse, spin Right, spin Left, Stop using arrow keys and space bare
# moves one wheel revolution for each step
# Press , . or < > to speed up slow down
# Press Ctrl-C to stop
#
# To check wiring is correct ensure the order of movement as above is correct
# Run using: sudo python beebot.py


import time, threading
import RPi.GPIO as GPIO

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

running = True
countL = 0
countR = 0
speed = 30
lineLeft = 12
lineRight = 13
# Pins 24, 26 Left Motor
# Pins 19, 21 Right Motor
L1 = 26
L2 = 24
R1 = 19
R2 = 21

def wheelCount():
    global running, countL, countR
    lastValidL = 2
    lastValidR = 2
    lastL = GPIO.input(lineLeft)
    lastR = GPIO.input(lineRight)
    while running:
        time.sleep(0.002)
        val = GPIO.input(lineLeft)
        if val == lastL and val != lastValidL:
            countL += 1
            lastValidL = val
            #print countL
        lastL = val
        val = GPIO.input(lineRight)
        if val == lastR and val != lastValidR:
            countR += 1
            lastValidR = val
        lastR = val

def initGPIO():
    global p, q, a, b
    #use physical pin numbering
    GPIO.setmode(GPIO.BOARD)

    #set up digital line detectors as inputs
    GPIO.setup(lineRight, GPIO.IN) # Right line sensor
    GPIO.setup(lineLeft, GPIO.IN) # Left line sensor

    #use pwm on inputs so motors don't go too fast
    GPIO.setup(L1, GPIO.OUT)
    p = GPIO.PWM(L1, 20)
    p.start(0)

    GPIO.setup(L2, GPIO.OUT)
    q = GPIO.PWM(L2, 20)
    q.start(0)

    GPIO.setup(R1, GPIO.OUT)
    a = GPIO.PWM(R1, 20)
    a.start(0)

    GPIO.setup(R2, GPIO.OUT)
    b = GPIO.PWM(R2, 20)
    b.start(0)

    threadC = threading.Thread(target=wheelCount)
    threadC.start()

# spinLeft(speed): Sets motors to turn opposite directions at speed. 0 <= speed <= 100
def spinLeft(leftSpeed, rightSpeed):
    p.ChangeDutyCycle(0)
    q.ChangeDutyCycle(leftSpeed)
    a.ChangeDutyCycle(rightSpeed)
    b.ChangeDutyCycle(0)
    q.ChangeFrequency(leftSpeed + 5)
    a.ChangeFrequency(rightSpeed + 5)
    
# spinRight(speed): Sets motors to turn opposite directions at speed. 0 <= speed <= 100
def spinRight(leftSpeed, rightSpeed):
    p.ChangeDutyCycle(leftSpeed)
    q.ChangeDutyCycle(0)
    a.ChangeDutyCycle(0)
    b.ChangeDutyCycle(rightSpeed)
    p.ChangeFrequency(leftSpeed + 5)
    b.ChangeFrequency(rightSpeed + 5)
    
# turnForward(leftSpeed, rightSpeed): Moves forwards in an arc by setting different speeds. 0 <= leftSpeed,rightSpeed <= 100
def turnForward(leftSpeed, rightSpeed):
    p.ChangeDutyCycle(leftSpeed)
    q.ChangeDutyCycle(0)
    a.ChangeDutyCycle(rightSpeed)
    b.ChangeDutyCycle(0)
    p.ChangeFrequency(leftSpeed + 5)
    a.ChangeFrequency(rightSpeed + 5)
    
# turnReverse(leftSpeed, rightSpeed): Moves backwards in an arc by setting different speeds. 0 <= leftSpeed,rightSpeed <= 100
def turnReverse(leftSpeed, rightSpeed):
    p.ChangeDutyCycle(0)
    q.ChangeDutyCycle(leftSpeed)
    a.ChangeDutyCycle(0)
    b.ChangeDutyCycle(rightSpeed)
    q.ChangeFrequency(leftSpeed + 5)
    b.ChangeFrequency(rightSpeed + 5)

def stopL():
    p.ChangeDutyCycle(0)
    q.ChangeDutyCycle(0)

def stopR():
    a.ChangeDutyCycle(0)
    b.ChangeDutyCycle(0)

# stepForward(speed, steps): Moves forward specified number of counts, then stops
def stepForward(speed, counts):
    global countL, countR
    countL = 0
    countR = 0
    runL = True
    runR = True
    turnForward(speed, speed)
    while runL or runR:
        time.sleep(0.002)
        if countL == counts:
            stopL()
            runL = False
        if countR == counts:
            stopR()
            runR = False
            
# stepReverse(speed, steps): Moves backward specified number of counts, then stops
def stepReverse(speed, counts):
    global countL, countR
    countL = 0
    countR = 0
    runL = True
    runR = True
    turnReverse(speed, speed)
    while runL or runR:
        time.sleep(0.002)
        if countL == counts:
            stopL()
            runL = False
        if countR == counts:
            stopR()
            runR = False
            
# stepSpinL(speed, steps): Spins left specified number of counts, then stops
def stepSpinL(speed, counts):
    global countL, countR
    countL = 0
    countR = 0
    spinLeft(speed, speed)
    while countL!=counts or countR!=counts:
        time.sleep(0.002)
        if countL == counts:
            stopL()
        if countR == counts:
            stopR()
            
# stepSpinR(speed, steps): Spins right specified number of counts, then stops
def stepSpinR(speed, counts):
    global countL, countR
    countL = 0
    countR = 0
    spinRight(speed, speed)
    while countL!=counts or countR!=counts:
        time.sleep(0.002)
        if countL == counts:
            stopL()
        if countR == counts:
            stopR()
            
initGPIO()
# main loop
try:
    while True:
        keyp = readkey()
        if keyp == 'w' or ord(keyp) == 16:
            stepForward(speed, 16)
            print 'Forward', speed
        elif keyp == 'z' or ord(keyp) == 17:
            stepReverse(speed, 16)
            print 'Reverse', speed
        elif keyp == 's' or ord(keyp) == 18:
            stepSpinR(speed, 7)
            print 'Spin Right', speed
        elif keyp == 'a' or ord(keyp) == 19:
            stepSpinL(speed, 7)
            print 'Spin Left', speed
        elif keyp == 'n':
            turnForward(0, speed)
            print 'Stop Left wheel', speed
        elif keyp == 'm':
            turnForward(speed, 0)
            print 'Stop Right wheel', speed
        elif keyp == 'p':
            stepForward(speed, 16, 16)
            print 'Step Forward', speed, 16, 16
        elif keyp == '.' or keyp == '>':
            speed = min(100, speed+10)
            print 'Speed+', speed
        elif keyp == ',' or keyp == '<':
            speed = max (0, speed-10)
            print 'Speed-', speed
        elif keyp == ' ':
            stopL()
            stopR()
            print 'Stop'
        elif keyp == '0':
            leftCount = 0
            rightCount = 0
        elif ord(keyp) == 3:
            break
        time.sleep(0.1)
        print "Left, Right:", countL, countR

except KeyboardInterrupt:
    print

finally:
    pass
    running = False
# wait for thread to complete
    time.sleep(0.25)
    GPIO.cleanup()
