# Example Python code for PiRingo
#
# Runs a series of flashing demos on PiRingo
# Must be run as root - sudo python flashdemo.py 

# Set up the necessary libraries
import time, random, RPi.GPIO as GPIO

# Ignore GPIO warnings that may be due to earlier programs
GPIO.setwarnings(False)

# Tell GPIO library to use physical GPIO connector pins
GPIO.setmode(GPIO.BOARD)

# Define pins to use for switches and LEDs
SWITCH1_PIN = 19
SWITCH2_PIN = 21
LEDS = [7, 11, 12, 13, 15, 16, 18, 22, 24, 26, 8, 10]

# Define value to use for LEDs ON and OFF states
LEDOFF = 1
LEDON = 0
NUMLEDS = 12

# Function to initialise all GPIO pins
# Set up all LED pins as outputs
# Set up both Switches as Inputs with pullup resistors
def setupgpio():
    a = 0
    while a < NUMLEDS:
        GPIO.setup(LEDS[a], GPIO.OUT)
        a += 1
    GPIO.setup(SWITCH1_PIN,GPIO.IN,pull_up_down=GPIO.PUD_UP)
    GPIO.setup(SWITCH2_PIN,GPIO.IN,pull_up_down=GPIO.PUD_UP)

# Function to flash the LEDs randomly
def randomflash(val):
    count = 0
    a = 0
    while count < val:
        b = random.randrange(0, NUMLEDS)
        GPIO.output(LEDS[a], LEDOFF)
        GPIO.output(LEDS[b], LEDON)
        a = b
        count += 1
        time.sleep(0.1)
    alloff()

# Function to switch all LEDs OFF
def alloff ():
    for val in LEDS:
        GPIO.output(val,LEDOFF)

# Function to switch all LEDs ON
def allon ():
    for val in LEDS:
        GPIO.output(val,LEDON)

# Function to sweep a single LED clockwise around the board
# val argument defines how many steps to do
def chase1 (val):
    a = 0
    count = 0
    while count < val:
        b = a - 1
        if b < 0:
            b = NUMLEDS - 1
        GPIO.output(LEDS[b],LEDOFF)        
        GPIO.output(LEDS[a],LEDON)
        a = a + 1
        if a >= NUMLEDS:
            a = 0
        time.sleep(0.1)
        count += 1

# Function to sweep 2 individual LEDs in opposite directions
# val argument defines how many steps to do
def chase2 (val):
    a = 0
    c = 0
    count = 0
    while count < val:
        b = a - 1
        if b < 0:
            b = NUMLEDS - 1
        d = c + 1
        if d >= NUMLEDS:
            d = 0
        GPIO.output(LEDS[b],LEDOFF)        
        GPIO.output(LEDS[a],LEDON)
        GPIO.output(LEDS[d],LEDOFF)        
        GPIO.output(LEDS[c],LEDON)
        a += 1
        if a >= NUMLEDS:
            a = 0
        c -= 1
        if c < 0:
            c = NUMLEDS - 1
        time.sleep(0.1)
        count += 1
    alloff()

# designed for the Ice Blue PiRingo
# Switches all Blue LEDs on or off
def blue (val):
    GPIO.output(LEDS[1],val)
    GPIO.output(LEDS[3],val)
    GPIO.output(LEDS[5],val)
    GPIO.output(LEDS[7],val)
    GPIO.output(LEDS[9],val)
    GPIO.output(LEDS[11],val)

# designed for the Ice Blue PiRingo
# Switches all White LEDs on or off
def white (val):
    GPIO.output(LEDS[0],val)
    GPIO.output(LEDS[2],val)
    GPIO.output(LEDS[4],val)
    GPIO.output(LEDS[6],val)
    GPIO.output(LEDS[8],val)
    GPIO.output(LEDS[10],val)

def alternate (val):
    count = 0
    while count < val:
        blue (LEDON)
        white (LEDOFF)
        time.sleep(0.3)
        blue (LEDOFF)
        white (LEDON)
        time.sleep(0.3)
        count += 1
    alloff()

# Run some of the functions one after the other
setupgpio()
alloff()
randomflash(50)
chase1(50)
chase2(50)
alternate(20)

GPIO.cleanup()

