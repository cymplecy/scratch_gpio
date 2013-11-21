#
# motorpitx v0.1 Python module for the MotorPiTX addon board for the Raspberry Pi
# Copyright (c) 2013 Jason Barnett <jase@boeeerb.co.uk>
#
# This module will allow you to control all the interfaces of the MotorPiTX
# addon board easily. This has been tested with a Raspberry Pi Model A with
# MotorPiTX v0.2 beta board. There is no reason why it shouldn't work with 
# other configurations, if it doesn't, email me with as much information and
# let me know!
#
# This program is free software; you can redistribute it and/or modify
# it under the terms of the GNU General Public License version 2 as
# published by the Free Software Foundation.
#
# Servod is a ServoBlaster daemon by Richard Hirst <richardghirst@gmail.com>
# https://github.com/richardghirst/PiBits/tree/master/ServoBlaster
#
# Revisions
#
# 0.1 - Initial release (Functions are; blink, motor1, motor2, out1, out2, in1, in2, servo1, servo2, cleanup)
#
#
#


from time import sleep
import RPi.GPIO as GPIO
import os
from random import randint

print "Setting up pins"

GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)

READY = 7
MOTA1 = 9
MOTA2 = 10
MOTAE = 11
MOTB1 = 24
MOTB2 = 23
MOTBE = 25
OUT1 = 22
OUT2 = 17
if GPIO.RPI_REVISION == 1:
	IN1 = 21
else:
	IN1 = 27
#IN1 = 27
IN2 = 4
SERVO1 = 18
SERVO2 = 15

GPIO.setup(READY, GPIO.OUT)
GPIO.setup(MOTA1, GPIO.OUT)
GPIO.setup(MOTA2, GPIO.OUT)
GPIO.setup(MOTAE, GPIO.OUT)
GPIO.setup(MOTB1, GPIO.OUT)
GPIO.setup(MOTB2, GPIO.OUT)
GPIO.setup(MOTBE, GPIO.OUT)
GPIO.setup(OUT1, GPIO.OUT)
GPIO.setup(OUT2, GPIO.OUT)
GPIO.setup(IN1, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
GPIO.setup(IN2, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
GPIO.setup(SERVO1, GPIO.OUT, initial=GPIO.LOW)
GPIO.setup(SERVO2, GPIO.OUT, initial=GPIO.LOW)

MOTAPWM = GPIO.PWM(MOTAE,100)
MOTAPWM.start(0)
MOTBPWM = GPIO.PWM(MOTBE,100)
MOTBPWM.start(0)

OUT1PWM = GPIO.PWM(OUT1,100)
OUT1PWM.start(0)
OUT2PWM = GPIO.PWM(OUT2,100)
OUT2PWM.start(0)

## Blink the Ready light, perfect first test
def blink():
	GPIO.output(READY, GPIO.LOW)
	sleep(1)
	GPIO.output(READY, GPIO.HIGH)
	sleep(1)

def motor1(value):
	Error = False
	while True:
		try:
			value = int(value)
			break
		except:
			Error = True
			print "Motor 1: Please enter a number - " + value + " isn't valid"
			break
	while Error != True:
		if -100 <= value <= 100:
			break
		else:
			print "Motor 1: Number must be between -100 and 100 - '" + str(value) + "' isn't valid"
			Error = True
			break
	
	if Error != True:
		if value > 0:
			#print "Forward"
			GPIO.output(MOTA1, GPIO.HIGH)
			GPIO.output(MOTA2, GPIO.LOW)
			MOTAPWM.ChangeDutyCycle(value)
		elif value < 0:
			#print "Backwards"
			GPIO.output(MOTA1, GPIO.LOW)
			GPIO.output(MOTA2, GPIO.HIGH)
			MOTAPWM.ChangeDutyCycle(abs(value))
		else:
			#print "Stopped"
			GPIO.output(MOTA1, GPIO.LOW)
			GPIO.output(MOTA2, GPIO.LOW)
			MOTAPWM.ChangeDutyCycle(0)

def motor2(value):
	Error = False
	while True:
		try:
			value = int(value)
			break
		except:
			Error = True
			print "Motor 2: Please enter a number - " + value + " isn't valid"
			break
	while Error != True:
		if -100 <= value <= 100:
			break
		else:
			print "Motor 2: Number must be between -100 and 100 - '" + str(value) + "' isn't valid"
			Error = True
			break
	
	if Error != True:
		if value > 0:
			#print "Forward"
			GPIO.output(MOTB1, GPIO.HIGH)
			GPIO.output(MOTB2, GPIO.LOW)
			MOTBPWM.ChangeDutyCycle(value)
		elif value < 0:
			#print "Backwards"
			GPIO.output(MOTB1, GPIO.LOW)
			GPIO.output(MOTB2, GPIO.HIGH)
			MOTBPWM.ChangeDutyCycle(abs(value))
		else:
			#print "Stopped"
			GPIO.output(MOTB1, GPIO.LOW)
			GPIO.output(MOTB2, GPIO.LOW)
			MOTBPWM.ChangeDutyCycle(0)


def out1(value):
	if value == True:
		OUT1PWM.ChangeDutyCycle(100)
	elif value == int(value):
		OUT1PWM.ChangeDutyCycle(value)
	else:
		GPIO.output(OUT1, GPIO.LOW)

def out2(value):
	if value == True:
		OUT2PWM.ChangeDutyCycle(100)
	elif value == int(value):
		OUT2PWM.ChangeDutyCycle(value)
	else:
		GPIO.output(OUT2, GPIO.LOW)

def in1():
	if GPIO.input(IN1):
		return True
	else:
		return False

def in2():
	if GPIO.input(IN2):
		return True
	else:
		return False


def servo1(value):
	Error = False
	while True:
		try:
			value = int(value)
			break
		except:
			Error = True
			print "Servo 1: Please enter a number - " + value + " isn't valid"
			break
	while Error != True:
		if 0 <= value <= 180:
			break
		else:
			print "Servo 1: Number must be between 0 and 180 - '" + str(value) + "' isn't valid"
			Error = True
			break
	
	if Error != True:
		servo = "echo 0=%d > /dev/servoblaster" % value
		os.system(servo)
		sleep(0.2)
		servo = "echo 0=0 > /dev/servoblaster"
		os.system(servo)

def servo2(value):
	Error = False
	while True:
		try:
			value = int(value)
			break
		except:
			Error = True
			print "Servo 2: Please enter a number - " + value + " isn't valid"
			break
	while Error != True:
		if 0 <= value <= 180:
			break
		else:
			print "Servo 2: Number must be between 0 and 180 - '" + str(value) + "' isn't valid"
			Error = True
			break
	
	if Error != True:
		servo = "echo 1=%d > /dev/servoblaster" % value
		os.system(servo)
		sleep(0.2)
		servo = "echo 1=0 > /dev/servoblaster"
		os.system(servo)
		
def cleanup():
	GPIO.cleanup()
