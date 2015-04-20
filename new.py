#!/usr/bin/env python
# ScratchGPIO - control Raspberry Pi GPIO ports using Scratch.
# Copyright (C) 2013-2015 by Simon Walters based on original code for
# PiFace by Thomas Preston

# This program is free software; you can redistribute it and/or
# modify it under the terms of the GNU General Public License
# as published by the Free Software Foundation; either version 2
# of the License, or (at your option) any later version.

# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.

# You should have received a copy of the GNU General Public License
# along with this program; if not, write to the Free Software
# Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA
# 02110-1301, USA.

# This code hosted on Github thanks to Ben Nuttall who taught me how to be
# a git(ter)lly
# Perf improvements with ADC and change to Pi2Go from p2g4
Version = 'v7.0.016'
import threading
import socket
import time
import sys
import struct
import datetime as dt
import shlex
import os
import math
import re
import sgh_GPIOController
import sgh_PiGlow
import sgh_PiMatrix
import sgh_Stepper
import sgh_Minecraft
import sgh_pnbLCD
import logging
import subprocess
import sgh_RasPiCamera
# import pygame removed because causing random failures
import random
import Queue
from sgh_cheerlights import CheerLights
#import uinput

#ui = UInput()


try:
    import meArm

    print "meArm imported OK"
except:
    print "meArm  NOT imported OK"
    pass

try:
    from sgh_MCP23008 import sgh_MCP23008

    print "MCP23008 imported OK"
except:
    print "MCP23008  NOT imported OK"
    pass

try:
    from sgh_MCP230xx import Adafruit_MCP230XX

    print "sgh_MCP230xx imported OK"
except:
    print "sgh_MCP230xx  NOT imported OK"
    pass

try:
    import spidev

    print "spidev imported OK"
except:
    print "Not imported OK"
    pass

try:
    from sgh_Adafruit_PWM_Servo_Driver import PWM

    print "PWM/Servo imported OK"
except:
    print "PWM/Servo NOT imported OK"
    pass

try:
    from sgh_PCF8591P import sgh_PCF8591P

    print "ADC/DAC imported OK"
except:
    print "ADC/DAC NOT imported OK"
    pass

try:
    from sgh_Adafruit_8x8 import sgh_EightByEight
    from sgh_Adafruit_8x8 import ColorEightByEight

    print "8x8 imported OK"
except:
    print "8x8 NOT imported OK"
    pass

try:
    from nunchuck import nunchuck

    print "nunchuck imported"
except:
    print "nunchuck not imported - check I2c is setup"


try:
    import mcpi.minecraft as minecraft

    print "Minecraft imported OK"
except:
    print "Minecraft NOT imported OK"
    pass

sghCT = None  # reserve for captouch


# try and inport smbus but don't worry if not installed
# try:
#    from smbus import SMBus
# except:
#    pass

#import RPi.GPIO as GPIO


class Compass:
    __scales = {
        0.88: [0, 0.73],
        1.30: [1, 0.92],
        1.90: [2, 1.22],
        2.50: [3, 1.52],
        4.00: [4, 2.27],
        4.70: [5, 2.56],
        5.60: [6, 3.03],
        8.10: [7, 4.35],
    }

    def __init__(self, port=0, address=0x1E, gauss=1.3, declination=(0, 0)):
        self.bus = SMBus(port)
        self.address = address

        (degrees, minutes) = declination
        self.__declDegrees = degrees
        self.__declMinutes = minutes
        self.__declination = (degrees + minutes / 60) * math.pi / 180

        (reg, self.__scale) = self.__scales[gauss]
        # 8 Average, 15 Hz, normal measurement
        self.bus.write_byte_data(self.address, 0x00, 0x70)
        self.bus.write_byte_data(self.address, 0x01, reg << 5)  # Scale
        # Continuous measurement
        self.bus.write_byte_data(self.address, 0x02, 0x00)

    def declination(self):
        return self.__declDegrees, self.__declMinutes

    def twos_complement(self, val, len):
        # Convert twos compliment to integer
        if val & (1 << len - 1):
            val -= 1 << len
        return val

    def __convert(self, data, offset):
        val = self.twos_complement(data[offset] << 8 | data[offset + 1], 16)
        if val == -4096:
            return None
        else:
            return round(val * self.__scale, 4)

    def axes(self):
        data = self.bus.read_i2c_block_data(self.address, 0x00)
        # print map(hex, data)
        x = self.__convert(data, 3)
        y = self.__convert(data, 7)
        z = self.__convert(data, 5)
        return (x, y, z)

    def heading(self):
        (x, y, z) = self.axes()
        headingRad = float(math.atan2(y, x))
        headingRad += self.__declination

        # Correct for reversed heading
        if headingRad < 0:
            headingRad += 2 * math.pi

        # Check for wrap and compensate
        elif headingRad > 2 * math.pi:
            headingRad -= 2 * math.pi

        # Convert to degrees from radians
        headingDeg = headingRad * 180 / math.pi
        degrees = math.floor(headingDeg)
        minutes = round((headingDeg - degrees) * 60)
        return headingDeg

    def degrees(self, (degrees, minutes)):
        return str(degrees) + "*" + str(minutes) + "'"

    def degreesdecimal(self, (degrees, minutes)):
        return str(degrees + (minutes / 60.0)) if (degrees >= 0) else str(degrees - (minutes / 60.0))

    def __str__(self):
        (x, y, z) = self.axes()
        return "Axis X: " + str(x) + "\n" \
                                     "Axis Y: " + str(y) + "\n" \
                                                           "Axis Z: " + str(z) + "\n" \
                                                                                 "dec deg: " + str(
            self.__declDegrees) + "\n" \
                                  "dec min: " + str(self.__declMinutes) + "\n" \
                                                                          "Declination: " + self.degreesdecimal(
            self.declination()) + "\n" \
                                  "Heading: " + str(self.heading()) + "\n"


### End Compasss #########################################################

def isNumeric(s):
    try:
        float(s)
        return True
    except ValueError:
        return False


def rtnNumeric(value, default):
    try:
        return float(value)
    except ValueError:
        return default


def rtnSign(s):
    try:
        if abs(s) == s:
            return 1
        else:
            return -1
    except ValueError:
        return 0


def removeNonAscii(s):
    return "".join(i for i in s if ord(i) < 128)


def getValue(searchString, dataString):
    outputall_pos = dataString.find((searchString + ' '))
    sensor_value = dataString[(outputall_pos + 1 + len(searchString)):].split()
    return sensor_value[0]


def sign(number): return cmp(number, 0)


def parse_data(dataraw, search_string):
    outputall_pos = dataraw.find(search_string)
    return dataraw[(outputall_pos + 1 + search_string.length):].split()


class MyError(Exception):

    def __init__(self, value):
        self.value = value

    def __str__(self):
        return repr(self.value)


class ultra(threading.Thread):

    def __init__(self, pinTrig, pinEcho, socket):
        threading.Thread.__init__(self)
        self.scratch_socket = socket
        self._stop = threading.Event()
        self.pinTrig = pinTrig
        self.pinEcho = pinEcho

    def stop(self):
        self._stop.set()
        print "Ultra Stop Set"

    def stopped(self):
        return self._stop.isSet()

    def run(self):
        while not self.stopped():
            startTime = time.time()
            if self.pinEcho == 0:
                distance = sghGC.pinSonar(self.pinTrig)  # do a ping
                sensor_name = 'ultra' + str(self.pinTrig)
                if "pi2go" in ADDON:
                    sensor_name = 'ultra'
            else:
                distance = sghGC.pinSonar2(self.pinTrig, self.pinEcho)
                sensor_name = 'ultra' + str(self.pinEcho)
            bcast_str = 'sensor-update "%s" %s' % (sensor_name, str(distance))
            # print 'sending: %s' % bcast_str
            msgQueue.put(((5, bcast_str)))
            timeTaken = time.time() - startTime
            # print "time taken:",timeTaken
            if timeTaken < sghGC.ultraFreq:
                time.sleep(sghGC.ultraFreq - timeTaken)
        print "ultra run ended for pin:", self.pinTrig


class ScratchSender(threading.Thread):

    def __init__(self, socket):
        threading.Thread.__init__(self)
        self.scratch_socket = socket
        self.scratch_socket2 = None
        self._stop = threading.Event()
        self.time_last_ping = 0.0
        self.time_last_compass = 0.0
        self.distlist = [0.0, 0.0, 0.0]
        self.sleepTime = 0.1
        print "Sender Init"

    def stop(self):
        self._stop.set()
        print "Sender Stop Set"

    def stopped(self):
        return self._stop.isSet()

    def broadcast_pin_update(self, pin, value):
        # print ADDON
        # print "sending",pin,value
        #sensor_name = "gpio" + str(GPIO_NUM[pin_index])
        #bcast_str = 'sensor-update "%s" %d' % (sensor_name, value)
        # print 'sending: %s' % bcast_str
        # msgQueue.put(((5,bcast_str)))

        # Normal action is to just send updates to pin values but this can be
        # modified if known addon in use
        sensor_name = "pin" + str(pin)
        sensorValue = str(value)
        if "ladder" in ADDON:
            # do ladderboard stuff
            sensor_name = "switch" + str([0, 21, 19, 24, 26].index(pin))
        elif "motorpitx" in ADDON:
            # do MotorPiTx stuff
            if pin == 13:
                sensor_name = "input1"
            if pin == 7:
                sensor_name = "input2"
            sensorValue = ("off", "on")[value == 1]
        elif "berry" in ADDON:
            # do berryclip stuff
            if pin == 26:
                sensor_name = "switch"
            if pin == 22:
                sensor_name = "switch2"
        elif "piringo" in ADDON:
            # do PiRingo stuff
            sensor_name = "switch" + str(1 + [19, 21].index(pin))
            sensorValue = ("on", "off")[value == 1]
        elif "pibrella" in ADDON:
            # print pin
            #sensor_name = "in" + str([0,19,21,24,26,23].index(pin))
            try:
                sensor_name = "Input" + \
                    ["NA", "A", "B", "C", "D", "E"][
                        ([0, 21, 26, 24, 19, 23].index(pin))]
                if sensor_name == "InputE":
                    sensor_name = "switch"
            except:
                print "pibrella input out of range"
                sensor_name = "pin" + str(pin)
                pass
            sensorValue = ("off", "on")[value == 1]

        elif "pidie" in ADDON:
            # print pin
            #sensor_name = "in" + str([0,19,21,24,26,23].index(pin))
            try:
                sensor_name = ["green", "red", "blue", "yellow"][
                    ([19, 21, 26, 24].index(pin))]
            except:
                print "pidie input out of range"
                sensor_name = "pin" + str(pin)
                pass
            sensorValue = ("on", "off")[value == 1]
        elif "playhat" in ADDON:
            # print pin
            #sensor_name = "in" + str([0,19,21,24,26,23].index(pin))
            try:
                sensor_name = ["red", "green", "yellow", "blue"][
                    ([7, 11, 13, 15].index(pin))]
            except:
                print "parthat out of range"
                sensor_name = "pin" + str(pin)
                pass
            sensorValue = ("on", "off")[value == 1]
        elif "p2g3" in ADDON:
            # print pin
            try:
                sensor_name = ["left", "front", "right", "lineleft", "lineright", "switch1", "switch2", "switch3"][
                    ([7, 15, 11, 12, 13, 16, 18, 22].index(pin))]
            except:
                print "p2g3 input out of range"
                sensor_name = "pin" + str(pin)
                pass
            sensorValue = ("on", "off")[value == 1]
        elif "pi2golite" in ADDON:
            # print pin
            try:
                sensor_name = ["left", "right", "switch", "lineleft", "lineright"][
                    ([7, 11, 23, 12, 13].index(pin))]
            except:
                print "pi2golite input ", pin, " out of range"
                sensor_name = "pin" + str(pin)
                pass
            sensorValue = ("on", "off")[value == 1]
        elif "pi2go" in ADDON:
            # print pin
            try:
                sensor_name = ["irleft", "irfront", "irright", "lineleft", "lineright", "switch"][
                    ([11, 13, 7, 12, 15, 16].index(pin))]
            except:
                print "pi2go input out of range"
                sensor_name = "pin" + str(pin)
                print sensor_name
                pass
            sensorValue = ("on", "off")[value == 1]
        elif "apb01" in ADDON:
            # print pin
            try:
                sensor_name = ["lineleft", "lineright"][([7, 11].index(pin))]
            except:
                print "agb01 input out of range"
                sensor_name = "pin" + str(pin)
                pass
            sensorValue = ("on", "off")[value == 1]
        elif "agobo" in ADDON:
            # print pin
            try:
                sensor_name = ["lineleft", "lineright", "mode"][
                    ([7, 11, 16].index(pin))]
            except:
                # print "agobo input out of range"
                sensor_name = "pin" + str(pin)
                pass
            sensorValue = ("on", "off")[value == 0]
            if sensor_name == "mode":
                sensorValue = ("on", "off")[value == 1]
        elif "pizazz" in ADDON:
            # print pin
            try:
                sensor_name = ["left", "right", ][([13, 12].index(pin))]
            except:
                print "pizazz input out of range"
                sensor_name = "pin" + str(pin)
                pass
            sensorValue = ("on", "off")[value == 1]
        elif "raspibot2" in ADDON:
            sensor_name = ["switch1", "switch2"][([23, 21].index(pin))]
            sensorValue = ("closed", "open")[value == 1]
        elif "simpie" in ADDON:
            sensor_name = ["red", "amber", "green"][([11, 13, 15].index(pin))]
            sensorValue = ("on", "off")[value == 1]
        elif "pitt" in ADDON:
            sensorValue = ("1", "0")[value == 1]
        elif "explorer" in ADDON:
            sensor_name = ["input1", "input2", "input3", "input4"][
                ([16, 15, 18, 22].index(pin))]
            sensorValue = ("0", "1")[value == 1]

        if ("fishdish" in ADDON):
            sensor_name = "switch"
            sensorValue = ("on", "off")[value == 1]

        if ("traffichat" in ADDON):
            sensor_name = "switch"
            sensorValue = ("on", "off")[value == 1]

        bcast_str = '"' + sensor_name + '" ' + sensorValue
        msgQueue.put(((5, "sensor-update " + bcast_str)))
        # print pin , sghGC.pinTrigger[pin]
        if sghGC.pinTrigger[pin] == 1:
            # print dt.datetime.now()
            print "trigger being sent for:", sensor_name
            msgQueue.put((5, 'broadcast "Trigger' + sensor_name + '"'))
            sghGC.pinTriggerName[pin] = sensor_name
            sghGC.pinTrigger[pin] = 2
            if sghGC.anyTrigger == 0:
                # print "Any trigger broadcast"
                msgQueue.put((5, 'broadcast "Trigger"'))
                sghGC.anyTrigger = 2

                # def send_scratch_command(self, cmd):
                # n = len(cmd)
                # b = (chr((n >> 24) & 0xFF)) + (chr((n >> 16) & 0xFF)) + (chr((n >>  8) & 0xFF)) + (chr(n & 0xFF))
                # self.scratch_socket.send(b + cmd)

                # time.sleep(2)

    def setsleepTime(self, sleepTime):
        self.sleepTime = sleepTime
        #print("sleeptime:%s", self.sleepTime )

    # Function to read SPI data from MCP3008 chip
    # Channel must be an integer 0-7
    def ReadChannel(self, channel):
        global spi
        # print spi
        try:
            adc = spi.xfer2([1, (8 + channel) << 4, 0])
            data = ((adc[1] & 3) << 8) + adc[2]
            # print "channel ok", channel
            return data
        except:
            print "spi exception - one of these is normal"
            spi = spidev.SpiDev()
            spi.open(0, 0)
            return 0

    def run(self):
        global firstRun, ADDON, compass, wii
        print lock
        # while firstRun:
        # print "first run running"
        # time.sleep(5)
        # set last pin pattern to inverse of current state
        pin_bit_pattern = [0] * len(sghGC.validPins)
        last_bit_pattern = [1] * len(sghGC.validPins)

        lastPinUpdateTime = time.time()
        lastTimeSinceLastSleep = time.time()
        lastTimeSinceMCPAnalogRead = time.time()
        self.sleepTime = 0.1
        tick = 0
        lastADC = [256, 256, 256, 256, 256, 256, 256, 256]
        lastpiAndBash = [1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1]

        joyx, joyy, accelx, accely, accelz, button = [0, 0, 0, 0, 0, 0]
        lastAngle = 0
        if wii is not None:
            sensor_name = 'angle'
            bcast_str = '"' + sensor_name + '" ' + str(int(0))
            msgQueue.put((5, "sensor-update " + bcast_str))
        lastmcpInput = 0

        if "pipiano" in ADDON:
            with lock:
                for digitalIO in range(0, 13):
                    if (sghGC.piAndBash[digitalIO] in [sghGC.PINPUT, sghGC.PINPUTNONE, sghGC.PINPUTDOWN]):
                        try:
                            lastpiAndBash[digitalIO] = mcp.input(digitalIO)
                        except:
                            pass

        while not self.stopped():

            loopTime = time.time() - lastTimeSinceLastSleep
            # print "how many millisecs since last loop", (loopTime * 1000)
            if loopTime < self.sleepTime:
                sleepdelay = self.sleepTime - loopTime
                # print "add in sleep of milliesec:",(sleepdelay) * 1000
                time.sleep(sleepdelay)  # be kind to cpu  :)
                tick += 1
                if tick == sys.maxint:
                    tick = 0
            lastTimeSinceLastSleep = time.time()

            # print "before lock"
            with lock:
                for listIndex in range(len(sghGC.validPins)):
                    pin = sghGC.validPins[listIndex]
                    pin_bit_pattern[listIndex] = 0
                    if (sghGC.pinUse[pin] in [sghGC.PINPUT, sghGC.PINPUTNONE, sghGC.PINPUTDOWN]):
                        #logging.debug("Checking event on pin:%s", pin )
                        pinEvent = sghGC.pinEvent(pin)
                        pinValue = sghGC.pinRead(pin)
                        pin_bit_pattern[listIndex] = pinValue
                        if pinEvent:
                            logging.debug(" ")
                            logging.debug("pinEvent Detected on pin:%s", pin)
                            #logging.debug("before updating pin patterm,last pattern :",pin_bit_pattern[listIndex],last_bit_pattern[listIndex] )
                            #logging.debug("afte uipdating pin patterm:",pin_bit_pattern[listIndex] )
                            if pin_bit_pattern[listIndex] == last_bit_pattern[listIndex]:
                                logging.debug(
                                    "pinEvent but pin state the same as before...")
                                # pin_bit_pattern[listIndex] = 1 - pin_bit_pattern[listIndex] #change pin pattern - warning pin_bit_pattern now has !pin state
                            #logging.debug("after checking states pin patterm,last pattern:",pin_bit_pattern[listIndex],last_bit_pattern[listIndex])

                            if sghGC.pinTrigger[pin] == 0:
                                sghGC.pinTrigger[pin] = 1
                                # print " "
                                # print "pin trigged:",pin,sghGC.pinTrigger[pin]
                                # print " "
                            time.sleep(0)

            # if PCF ADC found
            if (pcfSensor is not None) and ("explorer" not in ADDON):
                sensor_str = ""
                if tick % 5 == 0:
                    for channel in range(0, 4):  # loop thru all 4 inputs
                        adc = -1
                        try:
                            # get each value
                            adc = pcfSensor.readADC(channel) / 2.56
                            # print "adc",channel,adc
                        except:
                            pass
                        adc = int((adc + lastADC[channel]) / 2.0)
                        if adc <> lastADC[channel]:
                            # print "channel,adc:",(channel+1),adc
                            sensor_name = [
                                'lightfrontright', 'lightfrontleft', 'lightbackleft', 'lightbackright'][channel]
                            sensor_value = str(adc)
                            sensor_str += '"%s" %s ' % (sensor_name,
                                                        sensor_value)
                        lastADC[channel] = adc
                    if sensor_str != "":
                        msgQueue.put((0, "sensor-update " + sensor_str))

                    if sghGC.lightInfo:
                        sghGC.lightDirection = math.degrees(math.atan2(
                            lastADC[0] + lastADC[3] - lastADC[1] - lastADC[2], lastADC[0] + lastADC[1] - lastADC[2] - lastADC[3]))
                        sghGC.lightValue = max(
                            lastADC[0], lastADC[1], lastADC[2], lastADC[3])
                        sensor_str = 'sensor-update "%s" %s "%s" %s' % ("lightdirection", str(
                            int(sghGC.lightDirection)), "lightvalue", str(int(sghGC.lightValue)))
                        # print 'sending: %s' % bcast_str
                        msgQueue.put((0, sensor_str))

            if "piandbash" in ADDON:

                with lock:
                    bcast_str = ""
                    for digitalIO in range(0, 8):
                        if (sghGC.piAndBash[digitalIO] in [sghGC.PINPUT, sghGC.PINPUTNONE, sghGC.PINPUTDOWN]):
                            if mcp.input(digitalIO) != lastpiAndBash[digitalIO]:
                                sensor_name = 'input' + str(digitalIO)
                                bcast_str += '"' + sensor_name + '" ' + \
                                    str(mcp.input(digitalIO)
                                        >> digitalIO) + " "
                            lastpiAndBash[digitalIO] = mcp.input(digitalIO)
                    if bcast_str != "":
                        msgQueue.put((5, "sensor-update " + bcast_str))

                if (mcp.input(15) + mcp.input(14)
                        + mcp.input(13) + mcp.input(11) + mcp.input(9)) <> lastmcpInput:
                    dict2 = {
                        15: 'botsel', 14: 'topsel', 13: 'down', 11: 'enter', 9: 'up'}
                    for i in dict2.keys():
                        sensor_name = dict2[i]
                        sensor_value = ("on", "off")[mcp.input(i) > 0]
                        bcast_str = '"' + sensor_name + '" ' + sensor_value
                        msgQueue.put((5, "sensor-update " + bcast_str))
                    lastmcpInput = (
                        mcp.input(15) + mcp.input(14) + mcp.input(13) + mcp.input(11) + mcp.input(9))
                if (time.time() - lastTimeSinceMCPAnalogRead) > 0.5:
                    bcast_str = ""
                    for channel in range(0, 8):
                        adc = (
                            self.ReadChannel(channel) + (2 * lastADC[channel])) / 3
                        # (lastADC[channel] - 2) <= adc <= (lastADC[channel] +2):
                        if adc != lastADC[channel]:
                            sensor_name = 'adc' + str(channel)
                            bcast_str += '"' + sensor_name + \
                                '" ' + str(adc) + " "
                            if channel == 0:
                                sensor_name = 'temperature'
                                bcast_str += '"' + sensor_name + '" ' + \
                                    str(float(
                                        "{0:.1f}".format(((adc * 500) / float(1023)) - 50)))
                        lastADC[channel] = adc
                    msgQueue.put((5, "sensor-update " + bcast_str))

                    lastTimeSinceMCPAnalogRead = time.time()

            if "pipiano" in ADDON:

                with lock:
                    bcast_str = ""
                    for digitalIO in range(0, 13):
                        if (sghGC.piAndBash[digitalIO] in [sghGC.PINPUT, sghGC.PINPUTNONE, sghGC.PINPUTDOWN]):
                            try:
                                if mcp.input(digitalIO) != lastpiAndBash[digitalIO]:
                                    # + str([60,62,64,65,67,69,71,72,61,63,66,68,70][digitalIO])
                                    sensor_name = 'note'
                                    if (mcp.input(digitalIO) >> digitalIO) == 1:
                                        # str(mcp.input(digitalIO)>>digitalIO) + " "
                                        bcast_str += ' "' + sensor_name + '" 0'
                                    else:
                                        # str(mcp.input(digitalIO)>>digitalIO) + " "
                                        bcast_str += ' "' + sensor_name + '" ' + \
                                            str([60, 62, 64, 65, 67, 69, 71, 72, 61, 63, 66, 68, 70][
                                                digitalIO]) + " "
                                    msgQueue.put(
                                        (5, "sensor-update" + bcast_str))
                                    msgQueue.put(
                                        (5, 'broadcast "Trigger' + sensor_name + '"'))
                                lastpiAndBash[digitalIO] = mcp.input(digitalIO)
                            except:
                                pass
                    # if bcast_str != "":
                    #    msgQueue.put((5,"sensor-update" + bcast_str))
                    time.sleep(0.1)

                    # print wii
            if wii is not None:  # if wii  found
                # time.sleep(1)
                try:
                    joyx, joyy, accelx, accely, accelz, button = wii.raw()

                    if (joyx == 255) or (joyy == 255) or (accelx == 255) or (accely == 255) or (accelz == 255):
                        print dt.datetime.now()
                        print "try ok but seems to have locked up"
                        print joyx, joyy, accelx, accely, accelz
                        print "restarting wii"
                        wii = nunchuck()
                        print "wii restarted"
                        print wii.raw()
                        raise Escape
                except:
                    logging.debug(dt.datetime.now())
                    logging.debug("exception:Read from Nunchuck failed")
                    continue

                    # Always send button info
                button &= 0x3
                # print button, wii.button_c(),wii.button_z()
                sensor_name = 'buttonc'
                bcast_str = '"' + sensor_name + '" '
                if (button == 1 or button == 0):
                    bcast_str += "on"
                else:
                    bcast_str += "off"
                msgQueue.put((5, "sensor-update " + bcast_str))

                sensor_name = 'buttonz'
                bcast_str = '"' + sensor_name + '" '
                if (button == 2 or button == 0):
                    bcast_str += "on"
                else:
                    bcast_str += "off"
                msgQueue.put((5, "sensor-update " + bcast_str))

                # If simple mode just reading joystick digitally
                if sghGC.nunchuckLevel == 1:

                    joyx -= 0x7E
                    joyy -= 0x7B

                    joyx = math.copysign(100, joyx) if abs(joyx) > 50 else 0

                    joyy = math.copysign(100, joyy) if abs(joyy) > 50 else 0
                    # if (abs(joyx) + abs(joyy)) == 200:
                    #    joyx = math.copysign(70, joyx)
                    #    joyy = math.copysign(70, joyy)

                    jx = "middle"
                    jy = "middle"

                    if joyx > 25:
                        jx = "right"
                    elif joyx < -25:
                        jx = "left"

                    if joyy > 25:
                        jy = "up"
                    elif joyy < -25:
                        jy = "down"

                    sensor_name = 'joystickx'
                    bcast_str = '"' + sensor_name + '" ' + jx
                    msgQueue.put((5, "sensor-update " + bcast_str))
                    sensor_name = 'joysticky'
                    bcast_str = '"' + sensor_name + '" ' + jy
                    msgQueue.put((5, "sensor-update " + bcast_str))

                    if (abs(joyx) + abs(joyy)) != 0:
                        # print joyx,joyy
                        angle = (math.atan2(-joyy, -joyx) * 180.0 / math.pi)
                        angle += 180  # "Normal" non-scratch angle
                        angle -= 90
                        #angle = angle - 90
                        angle = (angle + 720) % 360
                        angle = (-angle) if (angle) < 180 else (360 - angle)
                        sensor_name = 'angle'
                        bcast_str = '"' + sensor_name + '" ' + str(int(angle))
                        msgQueue.put((5, "sensor-update " + bcast_str))

                    accelx = int(min(max(1.8 * (accelx - 0x7F), -90), 90))
                    accely = - int(min(max(1.8 * (accely - 0x7F), -90), 90))
                    accelz = int(min(max(1.8 * (accelz - 0x7F), -90), 90))

                    #accelx = 45
                    #accely = 45
                    turnx = accelx
                    turny = accely
                    if accelz < 0:
                        if turnx >= 0:
                            turnx = 90
                        else:
                            turnx = -90

                    if abs(turny) > 80:
                        turnx = 0

                    leftright = turnx

                    sensor_name = 'tiltx'
                    bcast_str = '"' + sensor_name + '" ' + str(leftright)
                    msgQueue.put((5, "sensor-update " + bcast_str))

                    updown = turny

                    sensor_name = 'tilty'
                    bcast_str = '"' + sensor_name + '" ' + str(updown)
                    msgQueue.put((5, "sensor-update " + bcast_str))

                if sghGC.nunchuckLevel == 2:

                    joyx -= 0x7E
                    joyy -= 0x7B
                    sensor_name = 'joystickx'
                    bcast_str = '"' + sensor_name + '" ' + str(joyx)
                    msgQueue.put((5, "sensor-update " + bcast_str))
                    sensor_name = 'joysticky'
                    bcast_str = '"' + sensor_name + '" ' + str(joyy)
                    msgQueue.put((5, "sensor-update " + bcast_str))

                    accelx = int(min(max(1.8 * (accelx - 0x7F), -90), 90))
                    accely = - int(min(max(1.8 * (accely - 0x7F), -90), 90))
                    accelz = int(min(max(1.8 * (accelz - 0x7F), -90), 90))

                    #accelx = 45
                    #accely = 45
                    turnx = accelx
                    turny = accely
                    if accelz < 0:
                        if turnx >= 0:
                            turnx = 90
                        else:
                            turnx = -90

                    if abs(turny) > 80:
                        turnx = 0

                    button &= 0x3
                    sensor_name = 'buttonc'
                    bcast_str = '"' + sensor_name + '" '
                    if (button == 1 or button == 0):
                        bcast_str += "on"
                    else:
                        bcast_str += "off"
                    msgQueue.put((5, "sensor-update " + bcast_str))

                    sensor_name = 'buttonz'
                    bcast_str = '"' + sensor_name + '" '
                    if (button == 2 or button == 0):
                        bcast_str += "on"
                    else:
                        bcast_str += "off"
                    msgQueue.put((5, "sensor-update " + bcast_str))

                    if (button == 2 or button == 0):
                        angle = 0 - \
                            (math.atan2(-turny, -turnx) * 180.0 / math.pi) - 90
                        sensor_name = 'angle'
                        bcast_str = '"' + sensor_name + '" ' + str(angle)
                        msgQueue.put((5, "sensor-update " + bcast_str))
                    else:
                        angle = 0 - \
                            (math.atan2(-turny, -turnx) * 180.0 / math.pi) - 90
                        sensor_name = 'angle'
                        bcast_str = '"' + sensor_name + '" ' + str(angle)
                        msgQueue.put((5, "sensor-update " + bcast_str))

                        leftright = turnx * 100 / 90

                        sensor_name = 'leftright'
                        bcast_str = '"' + sensor_name + '" ' + str(leftright)
                        msgQueue.put((5, "sensor-update " + bcast_str))

                        updown = turny * 100 / 90

                        sensor_name = 'updown'
                        bcast_str = '"' + sensor_name + '" ' + str(updown)
                        msgQueue.put((5, "sensor-update " + bcast_str))

                    sensor_name = 'accelx'
                    bcast_str = '"' + sensor_name + '" ' + str(accelx)
                    msgQueue.put((5, "sensor-update " + bcast_str))

                    sensor_name = 'accely'
                    bcast_str = '"' + sensor_name + '" ' + str(accely)
                    msgQueue.put((5, "sensor-update " + bcast_str))

                    sensor_name = 'accelz'
                    bcast_str = '"' + sensor_name + '" ' + str(accelz)
                    msgQueue.put((5, "sensor-update " + bcast_str))

            # if there is a change in the input pins
            for listIndex in range(len(sghGC.validPins)):
                pin = sghGC.validPins[listIndex]
                if (pin_bit_pattern[listIndex] != last_bit_pattern[listIndex]) or sghGC.pinTrigger[pin] == 1:
                    logging.debug("Change or triger detected in pin:%s changed to:%s, trigger val:%s", pin,
                                  pin_bit_pattern[listIndex], sghGC.pinTrigger[pin])
                    if (sghGC.pinUse[pin] in [sghGC.PINPUT, sghGC.PINPUTNONE, sghGC.PINPUTDOWN]):
                        # print pin , pin_value
                        self.broadcast_pin_update(
                            pin, pin_bit_pattern[listIndex])
                        # just to give Scratch a better chance to react to
                        # event
                        time.sleep(0.05)

            last_bit_pattern = list(pin_bit_pattern)
            #print ("last:%s",last_bit_pattern)
            #print ("this:%s",pin_bit_pattern)

            if ("pitt" in ADDON):
                sensor_name = "input"
                nibble = sghGC.pinRead(
                    15) + 2 * sghGC.pinRead(19) + 4 * sghGC.pinRead(21) + 8 * sghGC.pinRead(23)
                nibble = 15 - nibble
                bcast_str = '"' + sensor_name + '" ' + str(nibble)
                msgQueue.put((5, "sensor-update " + bcast_str))

            if (
                    time.time() - lastPinUpdateTime) > 3:  # This is to force the pin names to be read out even if they don't change
                # print int(time.time())
                lastPinUpdateTime = time.time()
                for listIndex in range(len(sghGC.validPins)):
                    pin = sghGC.validPins[listIndex]
                    if (sghGC.pinUse[pin] in [sghGC.PINPUT, sghGC.PINPUTNONE, sghGC.PINPUTDOWN]):
                        pin_bit_pattern[listIndex] = sghGC.pinRead(pin)
                        self.broadcast_pin_update(
                            pin, pin_bit_pattern[listIndex])

            if (time.time() - self.time_last_compass) > 0.25:
                # print "time up"
                # print compass
                # If Compass board truely present
                if (compass is not None):
                    # print "compass code"
                    heading = compass.heading()
                    sensor_name = 'heading'
                    bcast_str = 'sensor-update "%s" %d' % (
                        sensor_name, heading)
                    # print 'sending: %s' % bcast_str
                    msgQueue.put((5, bcast_str))
                self.time_last_compass = time.time()

            if "explorer" in ADDON:
                # print "explorer addon found"
                if sghGC.capTouch is None:
                    import sgh_captouch
                    print "SGH_captouch imported OK"
                    import sgh_captouch_helper
                    print "SGH_captouch_helper imported OK"
                    sghGC.capTouch = sgh_captouch.Cap1208()
                    sghCT = sgh_captouch_helper.sgh_captouch_helper()
                    sghGC.capTouchHelper = sghCT
                    print "sghCT", sghCT
                    import sgh_explorer_analog
                    sghGC.ADS1015 = sgh_explorer_analog.ADS1015()
                    print "ADS1015", sghGC.ADS1015
                    sensor_str = ""
                    for loop in range(0, 8):
                        sghGC.capTouch.on(loop, "press", sghCT.ctHandler)
                        # print "loop:",loop,sghCT.ctTrigStatus[loop]
                        sensor_value = "0"
                        sensor_name = "touch" + \
                            str(sghCT.ctTrigStatus[loop][0])
                        sensor_str += '"%s" %s ' % (sensor_name, sensor_value)
                    msgQueue.put((0, "sensor-update " + sensor_str))

                else:
                    sensor_str = ""
                    for loop in range(0, 8):

                        if sghCT.ctTrigStatus[loop][1] == 1:
                            # print "loop:",loop,sghCT.ctTrigStatus[loop]
                            # print dt.datetime.now()
                            sensor_name = "touch" + \
                                str(sghCT.ctTrigStatus[loop][0])
                            sensor_value = "1"
                            # print "trigger being sent for:", sensor_name
                            sensor_str += '"%s" %s ' % (sensor_name,
                                                        sensor_value)
                            # print sensor_name,sensor_value
                            bcast_str = 'broadcast "%s"' % (sensor_name)
                            msgQueue.put((0, bcast_str))
                            sghCT.ctTrigStatus[loop][1] = 2
                    if sensor_str != "":
                        msgQueue.put((0, "sensor-update " + sensor_str))

                    if sghCT.ctTrigStatus[8][1] == 1:
                        msgQueue.put((0, 'broadcast "Touch"'))
                        sghCT.ctTrigStatus[8][1] = 2

                    sensor_str = ""
                    if tick % 5 == 0:
                        for loop in range(0, 4):
                            sensor_name = "analog" + str(4 - loop)
                            sensor_value = str(sghGC.ADS1015.read_se_adc(loop))
                            # print sensor_name,sensor_value
                            sensor_str += '"%s" %s ' % (sensor_name,
                                                        sensor_value)
                        if sensor_str != "":
                            msgQueue.put((0, "sensor-update " + sensor_str))

        print "Sender Stopped"

        # time.sleep(2)


class ScratchListener(threading.Thread):

    def __init__(self, socket):
        threading.Thread.__init__(self)
        self.scratch_socket = socket
        self.scratch_socket2 = None
        self._stop = threading.Event()
        self.dataraw = ''
        self.value = None
        self.valueNumeric = None
        self.valueIsNumeric = None
        self.OnOrOff = None
        self.searchPos = 0
        self.encoderDiff = 0
        self.turnSpeed = 40
        self.turnSpeedAdj = 0

        self.matrixX = 0
        self.matrixY = 0
        self.matrixUse = 64
        self.matrixColour = 'FFFFFF'
        self.matrixRed = 255
        self.matrixGreen = 255
        self.matrixBlue = 255
        self.matrixMult = 1
        self.matrixLimit = 1
        self.matrixRangemax = 8
        self.arm = None
        self.carryOn = True
        self.carryOnInUse = False

    def meArmGotoPoint(self, meHorizontal, meDistance, meVertical):
        self.arm.gotoPoint(int(max(-50, min(50, meHorizontal))), int(max(70, min(150, meDistance))),
                           int(max(0, min(60, meVertical))))
        print "moved"

        # def send_scratch_command(self, cmd):
        # n = len(cmd)
        # b = (chr((n >> 24) & 0xFF)) + (chr((n >> 16) & 0xFF)) + (chr((n >>  8) & 0xFF)) + (chr(n & 0xFF))
        # self.scratch_socket.send(b + cmd)

    def getValue(self, searchString):
        outputall_pos = self.dataraw.find((searchString + ' '))
        sensor_value = self.dataraw[
            (outputall_pos + 1 + len(searchString)):].split()
        try:
            return sensor_value[0]
        except IndexError:
            return ""

    # Find pos of searchStr - must be preceded by a deself.matrixLimiting
    # space to be found
    def bFind(self, searchStr):
        # print "looking in" ,self.dataraw , "for" , searchStr
        self.searchPos = self.dataraw.find(' ' + searchStr) + 1
        # time.sleep(0.1)
        # if (' '+searchStr in self.dataraw):
        # print "Found"
        return (' ' + searchStr in self.dataraw)

    def bFindOn(self, searchStr):
        return (self.bFind(searchStr + 'on ') or self.bFind(searchStr + 'high ') or self.bFind(searchStr + '1 '))

    def bFindOff(self, searchStr):
        return (self.bFind(searchStr + 'off ') or self.bFind(searchStr + 'low ') or self.bFind(searchStr + '0 '))

    def bFindOnOff(self, searchStr):
        # print "searching for" ,searchStr
        self.OnOrOff = None
        if (self.bFind(searchStr + 'on ') or self.bFind(searchStr + 'high ') or self.bFind(
                searchStr + '1 ') or self.bFind(searchStr + 'true ')):
            self.OnOrOff = 1
            self.valueNumeric = 1
            self.value = "on"
            return True
        elif (self.bFind(searchStr + 'off ') or self.bFind(searchStr + 'low ') or self.bFind(
                searchStr + '0 ') or self.bFind(searchStr + 'false ')):
            self.OnOrOff = 0
            self.valueNumeric = 01
            self.value = "off"
            return True
        else:
            return False

    def bCheckAll(self, default=True, pinList=None):
        if self.bFindOnOff('all'):
            if default:
                pinList = sghGC.validPins
            for pin in pinList:
                # print pin
                if sghGC.pinUse[pin] in [sghGC.POUTPUT, sghGC.PPWM, sghGC.PPWMMOTOR]:
                    # print pin
                    sghGC.pinUpdate(pin, self.OnOrOff)

    def bPinCheck(self, pinList):
        for pin in pinList:
            logging.debug("bPinCheck:%s", pin)
            if self.bFindOnOff('pin' + str(pin)):
                sghGC.pinUpdate(pin, self.OnOrOff)
            if self.bFindOnOff('gpio' + str(sghGC.gpioLookup[pin])):
                sghGC.pinUpdate(pin, self.OnOrOff)
            if self.bFindValue('power' + str(pin)):
                print pin, self.value
                if self.valueIsNumeric:
                    sghGC.pinUpdate(pin, self.valueNumeric, type="pwm")
                else:
                    sghGC.pinUpdate(pin, 0, type="pwm")

    def bLEDCheck(self, ledList):
        for led in range(1, (1 + len(ledList))):  # loop thru led numbers
            if self.bFindOnOff('led' + str(led)):
                sghGC.pinUpdate(ledList[led - 1], self.OnOrOff)

    def bListCheck(self, pinList, nameList):
        for loop in range(0, len(pinList)):  # loop thru list
            # print str(nameList[loop]) , pinList[loop]
            if self.bFindOnOff(str(nameList[loop])):
                # print str(nameList[loop]) , "found"
                sghGC.pinUpdate(pinList[loop], self.OnOrOff)

            if self.bFindValue('power' + str(nameList[loop]) + ","):
                if self.valueIsNumeric:
                    sghGC.pinUpdate(
                        pinList[loop], self.valueNumeric, type="pwm")
                else:
                    sghGC.pinUpdate(pinList[loop], 0, type="pwm")

    def bListCheckPowerOnly(self, pinList, nameList):
        for loop in range(0, len(pinList)):  # loop thru list
            if self.bFindValue('power' + str(nameList[loop]) + ","):
                if self.valueIsNumeric:
                    sghGC.pinUpdate(
                        pinList[loop], self.valueNumeric, type="pwm")
                else:
                    sghGC.pinUpdate(pinList[loop], 0, type="pwm")

    def bFindValue(self, searchStr, searchSuffix=''):
        #logging.debug("Searching for:%s",searchStr )
        # return the value of the charachters following the searchstr as float if possible
        # If not then try to return string
        # If not then return ""
        self.value = None
        self.valueNumeric = None
        self.valueIsNumeric = False

        if self.bFind(searchStr):
            if searchSuffix == '':
                # print "$$$" + self.dataraw + "$$$"
                # print "search" , searchStr
                # print "pos", self.searchPos
                # print "svalue",(self.dataraw[(self.searchPos + len(searchStr)):] + "   ")
                # print "bfind",(self.dataraw[(self.searchPos +
                # len(searchStr)):] + "    ").split()
                self.value = (
                    self.dataraw[(self.searchPos + len(searchStr)):] + "   ").strip()
                if len(self.value) > 0:
                    self.value = self.value.split()[0]
                # print "1 s value",self.value
                # print self.value
                if isNumeric(self.value):
                    self.valueNumeric = float(self.value)
                    self.valueIsNumeric = True
                    # print "numeric" , self.valueNumeric
                return True
            else:
                self.value = (
                    self.dataraw[(self.searchPos + len(searchStr)):] + "   ").strip()
                if len(self.value) > 0:
                    self.value = self.value.split()[0]
                if self.value.endswith(searchSuffix):
                    self.value = (self.value[:-len(searchSuffix)]).strip()
                    # print "2 s value",self.value
                    # print self.value
                    if isNumeric(self.value):
                        self.valueNumeric = float(self.value)
                        self.valueIsNumeric = True
                        # print "numeric" , self.valueNumeric
                    return True
                else:
                    return False
        else:
            return False

    def bLEDPowerCheck(self, ledList):
        for led in range(1, (1 + len(ledList))):  # loop thru led numbers
            # print "power" +str(led) + ","
            if self.bFindValue('power' + str(led) + ","):
                if self.valueIsNumeric:
                    sghGC.pinUpdate(
                        ledList[led - 1], self.valueNumeric, type="pwm")
                else:
                    sghGC.pinUpdate(ledList[led - 1], 0, type="pwm")

    def vFind(self, searchStr):
        return ((' ' + searchStr + ' ') in self.dataraw)

    def vFindOn(self, searchStr):
        return (self.vFind(searchStr + 'on') or self.vFind(searchStr + 'high') or self.vFind(searchStr + '1'))

    def vFindOff(self, searchStr):
        return (self.vFind(searchStr + 'off') or self.vFind(searchStr + 'low') or self.vFind(searchStr + '0'))

    def vFindOnOff(self, searchStr):
        self.value = None
        self.valueNumeric = None
        self.valueIsNumeric = False
        self.OnOrOff = None
        if self.vFind(searchStr):

            self.value = self.getValue(searchStr)
            if str(self.value) in ["high", "on", "1"]:
                self.valueNumeric = 1
                self.OnOrOff = 1
            else:
                self.valueNumeric = 0
                self.OnOrOff = 0
            return True
        else:
            return False

    def vFindValue(self, searchStr):
        # print "searching for ", searchStr
        self.value = None
        self.valueNumeric = None
        self.valueIsNumeric = False
        if self.vFind(searchStr):
            # print "found"
            self.value = self.getValue(searchStr)
            # print self.value
            if isNumeric(self.value):
                self.valueNumeric = float(self.value)
                self.valueIsNumeric = True
                # print "numeric" , self.valueNumeric
            return True
        else:
            return False

    def vAllCheck(self, searchStr):
        if self.vFindOnOff(searchStr):
            for pin in sghGC.validPins:
                if sghGC.pinUse[pin] in [sghGC.POUTPUT, sghGC.PPWM, sghGC.PPWMMOTOR]:
                    sghGC.pinUpdate(pin, self.valueNumeric)

    def vPinCheck(self):
        for pin in sghGC.validPins:
            # print "checking pin" ,pin
            if self.vFindValue('pin' + str(pin)):
                if self.valueIsNumeric:
                    sghGC.pinUpdate(pin, self.valueNumeric)
                else:
                    sghGC.pinUpdate(pin, 0)

            if self.vFindValue('power' + str(pin)):
                # print pin , "found"
                if self.valueIsNumeric:
                    sghGC.pinUpdate(pin, self.valueNumeric, type="pwm")
                else:
                    sghGC.pinUpdate(pin, 0, type="pwm")

            if self.vFindValue('motor' + str(pin)):
                if self.valueIsNumeric:
                    sghGC.pinUpdate(pin, self.valueNumeric, type="pwmmotor")
                else:
                    sghGC.pinUpdate(pin, 0, type="pwmmotor")

            if self.vFindValue('gpio' + str(sghGC.gpioLookup[pin])):
                logging.debug("gpio lookup %s", str(sghGC.gpioLookup[pin]))
                if self.valueIsNumeric:
                    sghGC.pinUpdate(pin, self.valueNumeric)
                else:
                    sghGC.pinUpdate(pin, 0)
                    # time.sleep(1)

            if self.vFindValue('powergpio' + str(sghGC.gpioLookup[pin])):
                logging.debug("pin %s", pin)
                logging.debug(
                    "gpiopower lookup %s", str(sghGC.gpioLookup[pin]))
                if self.valueIsNumeric:
                    sghGC.pinUpdate(pin, self.valueNumeric, type="pwm")
                else:
                    sghGC.pinUpdate(pin, 0, type="pwm")

    def vLEDCheck(self, ledList):
        for led in range(1, (1 + len(ledList))):  # loop thru led numbers
            if self.vFindOnOff('led' + str(led)):
                sghGC.pinUpdate(ledList[led - 1], self.OnOrOff)
                #logging.debug("pin %s %s",ledList[led - 1],self.OnOrOff )

            if self.vFindValue('power' + str(led)):
                if self.valueIsNumeric:
                    sghGC.pinUpdate(
                        ledList[led - 1], self.valueNumeric, type="pwm")
                else:
                    sghGC.pinUpdate(ledList[led - 1], 0, type="pwm")

    def vListCheck(self, pinList, nameList):
        for loop in range(0, len(pinList)):  # loop thru pinlist numbers
            if self.vFindOnOff(str(nameList[loop])):
                sghGC.pinUpdate(pinList[loop], self.OnOrOff)

            if self.vFindValue('power' + str(nameList[loop])):
                if self.valueIsNumeric:
                    sghGC.pinUpdate(
                        pinList[loop], self.valueNumeric, type="pwm")
                else:
                    sghGC.pinUpdate(pinList[loop], 0, type="pwm")
            if self.vFindValue('motor' + str(nameList[loop])):
                if self.valueIsNumeric:
                    sghGC.pinUpdate(
                        pinList[loop], self.valueNumeric, type="pwmmotor")
                else:
                    sghGC.pinUpdate(pinList[loop], 0, type="pwmmotor")

    def vListCheckPowerOnly(self, pinList, nameList):
        for loop in range(0, len(pinList)):  # loop thru pinlist numbers
            if self.vFindValue('power' + str(nameList[loop])):
                if self.valueIsNumeric:
                    sghGC.pinUpdate(
                        pinList[loop], self.valueNumeric, type="pwm")
                else:
                    sghGC.pinUpdate(pinList[loop], 0, type="pwm")

    def vListCheckMotorOnly(self, pinList, nameList):
        for loop in range(0, len(pinList)):  # loop thru pinlist numbers
            if self.vFindValue('motor' + str(nameList[loop])):
                if self.valueIsNumeric:
                    sghGC.pinUpdate(
                        pinList[loop], self.valueNumeric, type="pwmmotor")
                else:
                    sghGC.pinUpdate(pinList[loop], 0, type="pwmmotor")

    def stop(self):
        self._stop.set()
        print "Listener Stop Set"

    def stopped(self):
        return self._stop.isSet()

    def stepperUpdate(self, pins, value, steps=2123456789, stepDelay=0.003):
        # print "pin" , pins , "value" , value
        # print "Stepper type", sgh_Stepper.sghStepper, "this one",
        # type(sghGC.pinRef[pins[0]])
        try:
            sghGC.pinRef[pins[0]].changeSpeed(
                max(-100, min(100, value)), steps)  # just update Stepper value
            # print "stepper updated"
            # print ("pin",pins, "set to", value)
        except:
            try:
                print ("Stopping PWM")
                sghGC.pinRef[pins[0]].stop()

            except:
                pass
            sghGC.pinRef[pins[0]] = None
            # time.sleep(5)
            #print ("New Stepper instance started", pins)
            sghGC.pinRef[pins[0]] = sgh_Stepper.sghStepper(
                sghGC, pins, stepDelay)  # create new Stepper instance
            sghGC.pinRef[pins[0]].changeSpeed(
                max(-100, min(100, value)), steps)  # update Stepper value
            sghGC.pinRef[pins[0]].start()  # update Stepper value
            # print 'pin' , pins , ' changed to Stepper'
            #print ("pins",pins, "set to", value)
        sghGC.pinUse[pins[0]] = sghGC.POUTPUT

    def encoderCount(self, pin):
        lastL = sghGC.pinRead(pin)
        print "start", pin, lastL
        lastValidL = lastL
        while not sghGC.encoderStopCounting[pin]:
            time.sleep(0.002)
            val = sghGC.pinRead(pin)
            # print "val", countingPin, val
            if val == lastL and val != lastValidL:
                sghGC.pinCount[pin] += (sghGC.countDirection[pin] * 1)
                sghGC.encoderTimeDiff[
                    pin] = time.time() - sghGC.encoderTime[pin]
                sghGC.encoderTime[pin] = time.time()
                lastValidL = val
                # print "count" ,pin , sghGC.pinCount[pin]
            lastL = val
        print "encoderCountExit for pin", pin

    def moveMotor(self, motorList, count, pin):
        speed = self.turnSpeed

        # This thread gets invoked when a command is given to turn the motor a
        # number of steps
        print "encoder count at thread start", sghGC.encoderInUse
        print "counting pin", pin
        print "motor pins", sghGC.pinValue[motorList[1]], sghGC.pinValue[motorList[2]]
        # countingPin = motorList[1][3] # use 1st motor counting pin only
        print "Previous EncoderDiff:", sghGC.pinEncoderDiff[pin]
        #sghGC.pinEncoderDiff[countingPin] = 0
        sghGC.encoderStopCounting[pin] = False
        # start encoder count thread for this motor encoder
        encoderMoveThread = threading.Thread(
            target=self.encoderCount, args=[pin])
        encoderMoveThread.start()
        startCount = sghGC.pinCount[pin]
        # modifiy count based on previous result
        countwanted = startCount + count + sghGC.pinEncoderDiff[pin]
        if (rtnSign(sghGC.pinEncoderDiff[pin]) != rtnSign(count)):
            print "doubling diff on pin ", pin
            # allow for modified behaviour
            countattempted = startCount + count + \
                int(2 * sghGC.pinEncoderDiff[pin])
        else:
            # allow for modified behaviour
            countattempted = startCount + count + \
                int(1 * sghGC.pinEncoderDiff[pin])
        print "extra count wanted/going to attempt", (countwanted - startCount), (countattempted - startCount)
        turningStartTime = time.time()  # used to timeout if necessary
        thisTurnSpeed = self.turnSpeed
        if pin == 12:
            thisTurnSpeed = self.turnSpeed + self.turnSpeedAdj
        print "pin turnspeed at start", pin, (self.turnSpeed + self.turnSpeedAdj)

        if count >= 0:
            sghGC.motorUpdate(motorList[1], motorList[2], thisTurnSpeed)
            while ((sghGC.pinCount[pin] < int(countattempted)) and ((time.time() - turningStartTime) < 20)):
                if pin == 13:
                    if sghGC.pinCount[13] > sghGC.pinCount[12]:
                        self.turnSpeedAdj = 0 - (self.turnSpeed / 2)
                        # print "turnspeeed sub" ,self.turnSpeedAdj
                        time.sleep(0.005)
                    if sghGC.pinCount[13] < sghGC.pinCount[12]:
                        self.turnSpeedAdj = (self.turnSpeed / 2)
                        # print "turnspeeed add" ,self.turnSpeedAdj
                        time.sleep(0.005)
                    if sghGC.pinCount[13] == sghGC.pinCount[12]:
                        self.turnSpeedAdj = 0
                        # print "turnspeeed stay" ,self.turnSpeedAdj
                        time.sleep(0.005)
                    sghGC.motorUpdate(motorList[1], motorList[2],
                                      max(0, min(100, (self.turnSpeed + self.turnSpeedAdj))))
                else:
                    sghGC.motorUpdate(
                        motorList[1], motorList[2], thisTurnSpeed)
                    print "encoder time diff", sghGC.encoderTimeDiff[pin]
                    # if ((sghGC.encoderTimeDiff[pin] > 0.04) and (sghGC.encoderTimeDiff[pin] < 1)):
                    #thisTurnSpeed += 1

                time.sleep(0.002)
        else:
            sghGC.motorUpdate(motorList[1], motorList[2], 0 - thisTurnSpeed)
            while ((sghGC.pinCount[pin] > int(countattempted)) and ((time.time() - turningStartTime) < 20)):
                if pin == 13:
                    if sghGC.pinCount[13] < sghGC.pinCount[12]:
                        self.turnSpeedAdj = 0 - (self.turnSpeed / 2)
                        # print "turnspeeed sub" ,self.turnSpeedAdj
                        time.sleep(0.005)
                    if sghGC.pinCount[13] > sghGC.pinCount[12]:
                        self.turnSpeedAdj = (self.turnSpeed / 2)
                        # print "turnspeeed add" ,self.turnSpeedAdj
                        time.sleep(0.005)
                    if sghGC.pinCount[13] == sghGC.pinCount[12]:
                        self.turnSpeedAdj = 0
                        # print "turnspeeed stay" ,self.turnSpeedAdj
                        time.sleep(0.005)
                    sghGC.motorUpdate(motorList[1], motorList[2],
                                      0 - max(0, min(100, (self.turnSpeed + self.turnSpeedAdj))))
                else:
                    sghGC.motorUpdate(
                        motorList[1], motorList[2], 0 - thisTurnSpeed)

                time.sleep(0.002)
        if pin == 13:
            print "pin turnspeed at end", pin, (self.turnSpeed + self.turnSpeedAdj)

        sghGC.motorUpdate(motorList[1], motorList[2], 0)
        print "motors off ", pin

        time.sleep(0.2)  # wait until motors have actually stopped
        sghGC.encoderStopCounting[pin] = True
        print ("how many moved", (sghGC.pinCount[pin] - startCount))
        # work out new error in position
        sghGC.pinEncoderDiff[pin] = (countwanted - (sghGC.pinCount[pin]))
        msgQueue.put((5, 'sensor-update "encoderdiff' + str(pin)) + '"' + str(
            sghGC.pinEncoderDiff[pin]) + '"')  # inform Scratch that turning is finished

        print "count wantedDiff:", countwanted, " / ", sghGC.pinEncoderDiff[pin]

        print "turning finished"
        print " "
        with lock:
            sghGC.encoderInUse -= 1
            print "encoders in use count at end of moveMotor for pin", pin, sghGC.encoderInUse

    def beep(self, pin, freq, duration):
        logging.debug("Freq:%s", freq)
        # Checks use of pin if not PWM mode then
        if sghGC.pinUse != sghGC.PPWM:
            sghGC.pinUpdate(pin, 0, "pwm")  # Set pin to PWM mode
        startCount = time.time()  # Get current time
        sghGC.pinFreq(pin, freq)  # Set freq used for PWM cycle
        # Set duty cycle to 50% to produce square wave
        sghGC.pinUpdate(pin, 50, "pwm")
        # Wait until duration has passed
        while (time.time() - startCount) < (duration * 1.0):
            time.sleep(0.01)
        sghGC.pinUpdate(pin, 0, "pwm")  # Turn pin off

    def vListHBridge2(self, motorlist):
        for loop in motorlist:
            if self.vFindValue(loop[0]):
                svalue = min(
                    100, max(-100, int(self.valueNumeric))) if self.valueIsNumeric else 0
                logging.debug("motor:%s valuee:%s", loop[0], svalue)
                sghGC.motorUpdate(loop[1], loop[2], svalue)

    def startUltra(self, pinTrig, pinEcho, OnOrOff):
        if OnOrOff == 0:
            try:
                sghGC.pinUltraRef[pinTrig].stop()
                sghGC.pinUse[pinTrig] = sghGC.PUNUSED
                sghGC.pinUltraRef[pinTrig] = None
                print "ultra stopped"
            except:
                pass
        else:
            print "Attemping to start ultra on pin:", pinTrig
            print sghGC.pinUltraRef[pinTrig]
            # if sghGC.pinUltraRef[pinTrig] is None:  NEEDS INVESTIGATING
            if True:
                sghGC.pinUse[pinTrig] = sghGC.PSONAR
                sghGC.pinUltraRef[pinTrig] = ultra(
                    pinTrig, pinEcho, self.scratch_socket)
                sghGC.pinUltraRef[pinTrig].start()
                print 'Ultra started pinging on', str(pinTrig)

    # noinspection PyPep8Naming
    def run(self):
        global firstRun, cycle_trace, step_delay, stepType, INVERT, \
            Ultra, ultraTotalInUse, piglow, PiGlow_Brightness, compass, ADDON, \
            meVertical, meHorizontal, meDistance, host

        # firstRun = True #Used for testing in overcoming Scratch "bug/feature"
        firstRunData = ''
        anyAddOns = False
        ADDON = ""
        #ultraThread = None

        # semi global variables used for servos in PiRoCon
        panoffset = 0
        tiltoffset = 0
        pan = 0
        tilt = 0
        steppersInUse = None
        beepDuration = 0.5
        beepNote = 60
        self.arm = None
        meHorizontal = 0
        meDistance = 100
        meVertical = 50
        tcolours = None  # set tcolours to None so it can be detected later
        pnblcd = None
        cheerList = None
        UH = None

        if not GPIOPlus:
            with lock:
                print "set pins standard"
                for pin in sghGC.validPins:
                    sghGC.pinUse[pin] = sghGC.PINPUT
                sghGC.pinUse[3] = sghGC.PUNUSED
                sghGC.pinUse[5] = sghGC.PUNUSED
                sghGC.pinUse[11] = sghGC.POUTPUT
                sghGC.pinUse[12] = sghGC.POUTPUT
                sghGC.pinUse[13] = sghGC.POUTPUT
                sghGC.pinUse[15] = sghGC.POUTPUT
                sghGC.pinUse[16] = sghGC.POUTPUT
                sghGC.pinUse[18] = sghGC.POUTPUT
                sghGC.setPinMode()

        if piglow is not None:
            PiGlow_Values = [0] * 18
            PiGlow_Lookup = [
                0, 1, 2, 3, 14, 12, 17, 16, 15, 13, 11, 10, 6, 7, 8, 5, 4, 9]
            PiGlow_Brightness = 255

            # This is main listening routine
        lcount = 0
        dataPrevious = ""
        debugLogging = False

        listenLoopTime = time.time() + 10000
        datawithCAPS = ''
        # This is the main loop that listens for messages from Scratch and
        # sends appropriate commands off to various routines
        while not self.stopped():

            # print "ListenLoopTime",listenLoopTime-time.time()
            listenLoopTime = time.time()
            #lcount += 1
            # print lcount
            try:
                # print "try reading socket"
                # This size will accomdate normal Scratch Control 'droid app
                # sensor updates
                BUFFER_SIZE = 512
                data = dataPrevious + self.scratch_socket.recv(
                    BUFFER_SIZE)  # get the data from the socket plus any data not yet processed
                logging.debug("datalen: %s", len(data))
                logging.debug("RAW: %s", data)

                if "send-vars" in data:
                    # Reset if New project detected from Scratch
                    # tell outer loop that Scratch has disconnected
                    if cycle_trace == 'running':
                        cycle_trace = 'disconnected'
                        print "cycle_trace has changed to", cycle_trace
                        break

                # Connection still valid so process the data received
                if len(data) > 0:

                    dataIn = data
                    datawithCAPS = data
                    #dataOut = ""
                    # used to hold series of broadcasts or sensor updates
                    dataList = []
                    # data to be re-added onto front of incoming data
                    dataPrefix = ""
                    while len(dataIn) > 0:  # loop thru data
                        # If whole length not received then break out of loop
                        if len(dataIn) < 4:
                            # print "<4 chrs received"
                            # store data and tag it onto next data read
                            dataPrevious = dataIn
                            break
                        sizeInfo = dataIn[0:4]
                        # get size of Scratch msg
                        size = struct.unpack(">L", sizeInfo)[0]
                        # print "size:", size
                        if size > 0:
                            # print dataIn[4:size + 4]
                            # turn msg into lower case
                            dataMsg = dataIn[4:size + 4].lower()
                            # print "msg:",dataMsg
                            # if msg recieved is too small
                            if len(dataMsg) < size:
                                # print "half msg found"
                                # print size, len(dataMsg)
                                # store data and tag it onto next data read
                                dataPrevious = dataIn
                                break

                            # if msg recieved is correct
                            if len(dataMsg) == size:
                                if "alloff" in dataMsg:
                                    allSplit = dataMsg.find("alloff")
                                    logging.debug(
                                        "not sure why this code is here Whole message:%s", dataIn)

                            # no data needs tagging to next read
                            dataPrevious = ""
                            if ("alloff" in dataMsg) or ("allon" in dataMsg):
                                dataList.append(dataMsg)
                            else:
                                # removed redundant "broadcast" and
                                # "sensor-update" txt
                                if dataMsg[0:2] == "br":
                                    if dataPrefix == "br":
                                        dataList[-1] = dataList[-1] + \
                                            " " + dataMsg[10:]
                                    else:
                                        dataList.append(dataMsg)
                                        dataPrefix = "br"
                                else:
                                    if dataPrefix == "se":
                                        # changr from 10 to 13
                                        dataList[-1] += dataMsg[13:]
                                    else:
                                        dataList.append(dataMsg)
                                        dataPrefix = "se"

                            # cut data down that's been processed
                            dataIn = dataIn[size + 4:]

                            # print "previous:", dataPrevious

                # print 'Cycle trace' , cycle_trace
                if len(data) == 0:
                    # This is due to client disconnecting or user loading new Scratch program so temp disconnect
                    # I'd like the program to retry connecting to the client
                    # tell outer loop that Scratch has disconnected
                    if cycle_trace == 'running':
                        cycle_trace = 'disconnected'
                        print "cycle_trace has changed to", cycle_trace
                        break

            except (KeyboardInterrupt, SystemExit):
                print "reraise error"
                raise
            except socket.timeout:
                # print "No data received: socket timeout"
                continue
            except:
                print "Unknown error occured with receiving data"
                # raise
                continue

            # At this point dataList[] contains a series of strings either broadcast or sensor-updates
            # print "data being processed:" , dataraw
            # This section is only enabled if flag set - I am in 2 minds as to whether to use it or not!
            # if (firstRun == True) or (anyAddOns == False):
            # print
            #logging.debug("dataList: %s",dataList)
            # print
            # print
            # print "old datalist" , dataList
            # or any("cheerlight" in s for s in dataList):
            if any("move" in s for s in dataList) or any("turn" in s for s in dataList):
                # print "move/turn found in dataList so going to expandList"

                newList = []
                for item in dataList:
                    # print "item" , item
                    if "sensor-update" in item:
                        newList.append(item)
                    if "broadcast" in item:
                        bList = shlex.split(item)  # item.split(" ")
                        for bItem in bList[1:]:
                            newList.append('broadcast "' + bItem + '"')
                dataList = newList
                # print "new dataList" ,dataList

            # print "GPIOPLus" , GPIOPlus
            # print "dataList to be processed", dataList
            for dataItem in dataList:
                # print dataItem
                #dataraw = ' '.join([item.replace(' ','') for item in shlex.split(dataItem)])
                dataraw = ' '
                # print "CAPS", datawithCAPS
                for item in shlex.split(dataItem):
                    # print "item in space remover" ,item
                    if item[0:4] == 'line':
                        origpos = datawithCAPS.lower().find(item)
                        item = datawithCAPS[origpos:origpos + len(item)]
                        item = 'line' + item[4:].strip()
                        item = item[0:5] + item[5:].lstrip()
                        dataraw = dataraw + \
                            ''.join(item.replace(' ', chr(254))) + ' '
                    else:
                        dataraw = dataraw + \
                            ''.join(item.replace(' ', '')) + ' '
                self.dataraw = dataraw

                logging.debug("processing dataItems: %s", self.dataraw)
                # print "Loop processing"
                # print dataItem, " has been converted to " ,self.dataraw
                # print
                if 'sensor-update' in self.dataraw:
                    # print "this data ignored" , dataraw
                    firstRunData = self.dataraw
                    #dataraw = ''
                    #firstRun = False
                    if self.vFindValue("autostart"):
                        if self.value == "true":
                            print "Autostart GreenFlag event"
                            msgQueue.put((5, "broadcast Scratch-StartClicked"))
                            time.sleep(1)
                            #fred = subprocess.Popen(['xdotool', 'getactivewindow', 'key', 'Return'])
                            # with open('info.txt', "w") as outfile:
                            output = subprocess.Popen(
                                "xwininfo -tree -root | grep squeak | awk '{print $5}' | tr 'x' ',' | tr '+' ','",
                                shell=True, stdout=subprocess.PIPE).communicate()
                            # fred = subprocess.call(['xwininfo','-tree','-root','|','grep','squeak'], stdout = outfile)##'|', 'awk', "'{print $5}'", '|', 'tr', "'x'" ,"','", '|' ,'tr' ,"'+'", "','"
                            # sizes = output[0][0:-1].split(',')
                            # print sizes
                            # xmid = (int(sizes[0]) + int(sizes[2]))/2
                            # ymid = (int(sizes[1]) + int(sizes[3]))/2
                            # print "sizes" ,sizes
                            # fred = subprocess.Popen(['xdotool', 'mousemove', str(xmid), str(ymid)]).wait()
                            # fred = subprocess.Popen(['xdotool', 'click', '1',]).wait()
                            # fred = subprocess.Popen(['xdotool', 'key', 'Return'])
                            # #print "fred",fred

                    if self.vFindValue("sghdebug"):
                        if (self.value == "1") and (debugLogging == False):
                            logging.getLogger().setLevel(logging.DEBUG)
                            debugLogging = True
                        if (self.value == "0") and (debugLogging == True):
                            logging.getLogger().setLevel(logging.INFO)
                            debugLogging = False

                    if self.vFindValue("bright"):
                        sghGC.ledDim = int(
                            self.valueNumeric) if self.valueIsNumeric else 20
                        PiGlow_Brightness = sghGC.ledDim
                        bcast_str = 'sensor-update "%s" %d' % (
                            'bright', sghGC.ledDim)
                        # print 'sending: %s' % bcast_str
                        msgQueue.put((5, bcast_str))
                        try:
                            UH.brightness(
                                max(0, min(1, float(float(sghGC.ledDim) / 100))))
                            UH.show()
                        except:
                            pass
                            # print sghGC.ledDim

                    if self.vFindValue("turnspeedadj"):
                        self.turnSpeedAdj = int(
                            self.valueNumeric) if self.valueIsNumeric else 0
                        print "TurnSpeedAdj", self.turnSpeed

                    if self.vFindValue("turnspeed"):
                        self.turnSpeed = int(
                            self.valueNumeric) if self.valueIsNumeric else 0
                        print "TurnSpeed", self.turnSpeed

                    if self.vFindValue("mfreq"):
                        sghGC.mFreq = int(
                            self.valueNumeric) if self.valueIsNumeric else 20
                        print "mFreq", sghGC.mFreq

                    if self.vFindValue("pfreq"):
                        sghGC.pFreq = int(
                            self.valueNumeric) if self.valueIsNumeric else 200
                        sghGC.changePWMFreq()
                        print "pFreq", sghGC.pFreq

                    pinsoraddon = None
                    if self.vFindValue("setpins"):
                        setupValue = self.value
                        pinsoraddon = "pins"
                    if self.vFindValue("addon"):
                        setupValue = self.value
                        pinsoraddon = "addon"

                    if pinsoraddon is not None:
                        ADDON = setupValue
                        print (ADDON, " declared")

                        if "setpinslow" in ADDON:
                            with lock:
                                print "set pins to input with pulldown low"
                                for pin in sghGC.validPins:
                                    sghGC.pinUse[pin] = sghGC.PINPUTDOWN
                                sghGC.pinUse[3] = sghGC.PUNUSED
                                sghGC.pinUse[5] = sghGC.PUNUSED
                                sghGC.setPinMode()
                                anyAddOns = True
                        if "setpinshigh" in ADDON:
                            with lock:
                                print "set pins to input"
                                for pin in sghGC.validPins:
                                    sghGC.pinUse[pin] = sghGC.PINPUT
                                sghGC.pinUse[3] = sghGC.PUNUSED
                                sghGC.pinUse[5] = sghGC.PUNUSED
                                sghGC.setPinMode()
                                anyAddOns = True
                        if "setpinsnone" in ADDON:
                            with lock:
                                print "set pins to input"
                                for pin in sghGC.validPins:
                                    sghGC.pinUse[pin] = sghGC.PINPUTNONE
                                sghGC.pinUse[3] = sghGC.PUNUSED
                                sghGC.pinUse[5] = sghGC.PUNUSED
                                sghGC.setPinMode()
                                anyAddOns = True
                        if "setpinsnormal" in ADDON:
                            with lock:
                                sghGC.pinUse[11] = sghGC.POUTPUT
                                sghGC.pinUse[12] = sghGC.POUTPUT
                                sghGC.pinUse[13] = sghGC.POUTPUT
                                sghGC.pinUse[15] = sghGC.POUTPUT
                                sghGC.pinUse[16] = sghGC.POUTPUT
                                sghGC.pinUse[18] = sghGC.POUTPUT
                                sghGC.pinUse[22] = sghGC.PINPUT
                                sghGC.pinUse[7] = sghGC.PINPUT

                                sghGC.setPinMode()
                                anyAddOns = True

                        if "ladder" in ADDON:
                            with lock:
                                sghGC.resetPinMode()
                                ladderOutputs = [
                                    11, 12, 13, 15, 16, 18, 22, 7, 5, 3]
                                for pin in ladderOutputs:
                                    sghGC.pinUse[pin] = sghGC.POUTPUT
                                for pin in [24, 26, 19, 21]:
                                    sghGC.pinUse[pin] = sghGC.PINPUT
                                sghGC.setPinMode()
                                anyAddOns = True

                        if "motorpitx" in ADDON:
                            with lock:
                                sghGC.resetPinMode()
                                sghGC.pinUse[11] = sghGC.POUTPUT  # Out2
                                sghGC.pinUse[15] = sghGC.POUTPUT  # Out1
                                sghGC.pinUse[16] = sghGC.POUTPUT  # Motor2 B
                                sghGC.pinUse[18] = sghGC.POUTPUT  # Motor2 A
                                sghGC.pinUse[19] = sghGC.POUTPUT  # Motor1
                                sghGC.pinUse[21] = sghGC.POUTPUT  # Motor1
                                # Motr 2 Enable
                                sghGC.pinUse[22] = sghGC.POUTPUT
                                # Motor1 Enable
                                sghGC.pinUse[23] = sghGC.POUTPUT

                                sghGC.pinUse[13] = sghGC.PINPUT  # Input 1
                                sghGC.pinUse[7] = sghGC.PINPUT  # Input 2

                                sghGC.setPinMode()
                                sghGC.startServod([12, 10])  # servos
                                print "MotorPiTx setup"
                                anyAddOns = True

                        if "piglow" in ADDON:
                            with lock:
                                sghGC.resetPinMode()
                                anyAddOns = True

                        if "gpio" in ADDON:
                            with lock:
                                print sghGC.pinUse
                                sghGC.resetPinMode()
                                print sghGC.pinUse
                                sghGC.pinUse[11] = sghGC.POUTPUT
                                sghGC.pinUse[12] = sghGC.POUTPUT
                                sghGC.pinUse[13] = sghGC.POUTPUT
                                sghGC.pinUse[15] = sghGC.POUTPUT
                                sghGC.pinUse[16] = sghGC.POUTPUT
                                sghGC.pinUse[18] = sghGC.POUTPUT
                                sghGC.pinUse[7] = sghGC.PINPUT
                                sghGC.pinUse[8] = sghGC.PINPUT
                                sghGC.pinUse[10] = sghGC.PINPUT
                                sghGC.pinUse[22] = sghGC.PINPUT
                                sghGC.setPinMode()
                                print "gPiO setup"
                                print sghGC.pinUse
                                anyAddOns = True

                        if "berry" in ADDON:
                            with lock:
                                sghGC.resetPinMode()
                                berryOutputs = [7, 11, 15, 19, 21, 23, 24]
                                for pin in berryOutputs:
                                    sghGC.pinUse[pin] = sghGC.POUTPUT
                                sghGC.pinUse[26] = sghGC.PINPUT
                                sghGC.pinUse[22] = sghGC.PINPUT

                                sghGC.setPinMode()
                                anyAddOns = True

                        if "pirocon" in ADDON:
                            with lock:
                                sghGC.resetPinMode()
                                sghGC.pinUse[19] = sghGC.POUTPUT  # MotorA
                                # MotorB (MotorA in v1.2)
                                sghGC.pinUse[21] = sghGC.POUTPUT
                                # MotorA (MotorB in V1.2)
                                sghGC.pinUse[26] = sghGC.POUTPUT
                                sghGC.pinUse[24] = sghGC.POUTPUT  # MotorB
                                sghGC.pinUse[7] = sghGC.PINPUT  # ObsLeft
                                sghGC.pinUse[11] = sghGC.PINPUT  # ObsRight
                                sghGC.pinUse[12] = sghGC.PINPUT  # LFLeft
                                sghGC.pinUse[13] = sghGC.PINPUT  # LFRight

                                if "encoders" in ADDON:
                                    logging.debug("Encoders Found:%s", ADDON)
                                    sghGC.pinUse[7] = sghGC.PCOUNT
                                    sghGC.pinUse[11] = sghGC.PCOUNT
                                    msgQueue.put(
                                        (5, 'sensor-update "encoder" "stopped"'))
                                    msgQueue.put(
                                        (5, 'sensor-update "count7" "0"'))
                                sghGC.setPinMode()
                                sghGC.startServod([18, 22])  # servos orig
                                # sghGC.startServod([12,10]) # servos testing
                                # motorpitx

                                print "pirocon setup"
                                anyAddOns = True

                        if "piringo" in ADDON:
                            with lock:
                                sghGC.resetPinMode()
                                print "piringo detected"
                                sghGC.setAllInvert(
                                    True)  # GPIO pull down each led so need to invert 0 to 1 and vice versa
                                piringoOutputs = [7, 11, 12, 13, 15, 16, 18, 22, 24, 26, 8,
                                                  10]  # these are pins used for LEDS for PiRingo
                                # These are the pins connected to the switches
                                piringoInputs = [19, 21]
                                for pin in piringoOutputs:
                                    # set leds as outputs
                                    sghGC.pinUse[pin] = sghGC.POUTPUT
                                for pin in piringoInputs:
                                    # set switches as inputs
                                    sghGC.pinUse[pin] = sghGC.PINPUT
                                sghGC.setPinMode()  # execute pin assignment
                                anyAddOns = True  # add on declared

                        if "pibrella" in ADDON:
                            with lock:
                                sghGC.resetPinMode()
                                pibrellaOutputs = [
                                    7, 11, 13, 15, 16, 18, 22, 12]
                                for pin in pibrellaOutputs:
                                    sghGC.pinUse[pin] = sghGC.POUTPUT
                                pibrellaInputs = [21, 26, 24, 19, 23]
                                for pin in pibrellaInputs:
                                    sghGC.pinUse[pin] = sghGC.PINPUTDOWN

                                sghGC.setPinMode()
                                anyAddOns = True

                        if "rgbled" in ADDON:
                            with lock:
                                sghGC.resetPinMode()
                                rgbOutputs = [12, 16, 18, 22, 7, 11, 13, 15]
                                for pin in rgbOutputs:
                                    sghGC.pinUse[pin] = sghGC.POUTPUT

                                sghGC.setPinMode()
                                anyAddOns = True

                        if "rtkrpimcb" in ADDON:
                            with lock:
                                sghGC.resetPinMode()
                                sghGC.pinUse[11] = sghGC.POUTPUT  # Motor1
                                sghGC.pinUse[12] = sghGC.POUTPUT  # Motor1
                                sghGC.pinUse[15] = sghGC.POUTPUT  # Motor2
                                sghGC.pinUse[16] = sghGC.POUTPUT  # Motor2
                                sghGC.pinUse[18] = sghGC.POUTPUT
                                sghGC.pinUse[22] = sghGC.POUTPUT

                                sghGC.setPinMode()
                                sghGC.pinUpdate(18, 1)
                                sghGC.pinUpdate(22, 1)
                                print "rtkmotorcon setup"
                                anyAddOns = True

                        if "traffichat" in ADDON:
                            with lock:
                                sghGC.resetPinMode()
                                traffichatOutputs = [15, 16, 18, 29]
                                for pin in traffichatOutputs:
                                    sghGC.pinUse[pin] = sghGC.POUTPUT
                                sghGC.pinUse[22] = sghGC.PINPUT

                                sghGC.setPinMode()
                                anyAddOns = True

                        if "pidie" in ADDON:
                            print "pidie enabled"

                            with lock:
                                sghGC.resetPinMode()
                                sghGC.setAllInvert(
                                    True)  # GPIO pull down each led so need to invert 0 to 1 and vice versa
                                pidieOutputs = [
                                    7, 11, 12, 13, 15, 16, 18, 22, 8]
                                for pin in pidieOutputs:
                                    sghGC.pinUse[pin] = sghGC.POUTPUT
                                pidieInputs = [21, 19, 24, 26]
                                for pin in pidieInputs:
                                    sghGC.pinUse[pin] = sghGC.PINPUT
                                sghGC.setPinMode()
                                anyAddOns = True

                        if "playhat" in ADDON:
                            print "playhat enabled"

                            with lock:
                                sghGC.resetPinMode()
                                sghGC.pinUse[16] = sghGC.POUTPUT
                                inputs = [7, 11, 13, 15]
                                for pin in inputs:
                                    sghGC.pinUse[pin] = sghGC.PINPUT
                                sghGC.setPinMode()
                                anyAddOns = True

                        if "fishdish" in ADDON:
                            with lock:
                                sghGC.resetPinMode()
                                fishOutputs = [7, 15, 21, 24]
                                for pin in fishOutputs:
                                    sghGC.pinUse[pin] = sghGC.POUTPUT
                                sghGC.pinUse[26] = sghGC.PINPUT

                                sghGC.setPinMode()
                                anyAddOns = True

                        if "p2g3" in ADDON:
                            with lock:
                                sghGC.resetPinMode()
                                # sghGC.pinUse[19] = sghGC.POUTPUT #MotorA
                                # sghGC.pinUse[21] = sghGC.POUTPUT #MotorA
                                # sghGC.pinUse[26] = sghGC.POUTPUT #MotorB
                                # sghGC.pinUse[24] = sghGC.POUTPUT #MotorB
                                sghGC.pinUse[7] = sghGC.PINPUT  # ObjLeft
                                sghGC.pinUse[11] = sghGC.PINPUT  # ObjRight
                                sghGC.pinUse[15] = sghGC.PINPUT  # ObjMid
                                sghGC.pinUse[12] = sghGC.PINPUT  # LFLeft
                                sghGC.pinUse[13] = sghGC.PINPUT  # LFRight
                                sghGC.pinUse[16] = sghGC.PINPUT
                                sghGC.pinUse[18] = sghGC.PINPUT
                                sghGC.pinUse[22] = sghGC.PINPUT

                                sghGC.setPinMode()
                                sghGC.motorUpdate(19, 21, 0)
                                sghGC.motorUpdate(26, 24, 0)

                                try:
                                    # go thru PowerPWM on PCA Board
                                    for i in range(0, 16):
                                        pcaPWM.setPWM(i, 0, 4095)
                                except:
                                    pass

                                self.startUltra(8, 0, self.OnOrOff)

                                #sghGC.pinEventEnabled = 0
                            # sghGC.startServod([12,10]) # servos testing
                            # motorpitx

                            print "p2g3 setup"
                            anyAddOns = True

                        if "pi2go" in ADDON:
                            if "pi2golite" in ADDON:
                                print "pi2golite found in", ADDON
                                with lock:
                                    sghGC.resetPinMode()
                                    # sghGC.pinUse[19] = sghGC.POUTPUT #MotorA
                                    # sghGC.pinUse[21] = sghGC.POUTPUT #MotorA
                                    # sghGC.pinUse[26] = sghGC.POUTPUT #MotorB
                                    # sghGC.pinUse[24] = sghGC.POUTPUT #MotorB
                                    sghGC.pinUse[7] = sghGC.PINPUT  # ObjLeft
                                    sghGC.pinUse[11] = sghGC.PINPUT  # ObjRight
                                    sghGC.pinUse[12] = sghGC.PINPUT  # LFLeft
                                    sghGC.pinUse[13] = sghGC.PINPUT  # LFRight
                                    sghGC.pinUse[23] = sghGC.PINPUT
                                    sghGC.pinUse[18] = sghGC.POUTPUT
                                    sghGC.pinUse[22] = sghGC.POUTPUT
                                    sghGC.pinInvert[15] = True
                                    sghGC.pinInvert[16] = True
                                    sghGC.pinUse[15] = sghGC.POUTPUT
                                    sghGC.pinUse[16] = sghGC.POUTPUT
                                    if "encoders" in ADDON:
                                        logging.debug(
                                            "Encoders Found:%s", ADDON)
                                        sghGC.pinUse[12] = sghGC.PCOUNT
                                        sghGC.pinUse[13] = sghGC.PCOUNT
                                        msgQueue.put(
                                            (5, 'sensor-update "motors" "stopped"'))

                                    sghGC.setPinMode()
                                    sghGC.startServod([18, 22])  # servos
                                    sghGC.motorUpdate(19, 21, 0)
                                    sghGC.motorUpdate(26, 24, 0)

                                    self.startUltra(8, 0, self.OnOrOff)

                                print "pi2golite setup"
                                if "encoders" in ADDON:
                                    print "with encoders"
                                anyAddOns = True
                            elif "pi2go" in ADDON:
                                with lock:
                                    sghGC.resetPinMode()
                                    # sghGC.pinUse[19] = sghGC.POUTPUT #MotorA
                                    # sghGC.pinUse[21] = sghGC.POUTPUT #MotorA
                                    # sghGC.pinUse[26] = sghGC.POUTPUT #MotorB
                                    # sghGC.pinUse[24] = sghGC.POUTPUT #MotorB
                                    sghGC.pinUse[7] = sghGC.PINPUT  # ObjLeft
                                    sghGC.pinUse[11] = sghGC.PINPUT  # ObjRight
                                    sghGC.pinUse[15] = sghGC.PINPUT  # ObjMid
                                    sghGC.pinUse[12] = sghGC.PINPUT  # LFLeft
                                    sghGC.pinUse[13] = sghGC.PINPUT  # LFRight
                                    sghGC.pinUse[16] = sghGC.PINPUT
                                    #sghGC.pinUse[18] = sghGC.PINPUT
                                    #sghGC.pinUse[22] = sghGC.PINPUT

                                    sghGC.setPinMode()
                                    sghGC.motorUpdate(19, 21, 0)
                                    sghGC.motorUpdate(26, 24, 0)

                                    try:
                                        # go thru PowerPWM on PCA Board
                                        for i in range(0, 12):
                                            pcaPWM.setPWM(i, 0, 0)
                                    except:
                                        print "SOFT ERROR - PWM not set for pi2go"
                                        pass

                                    self.startUltra(8, 0, self.OnOrOff)

                                    #sghGC.pinEventEnabled = 0
                                # sghGC.startServod([12,10]) # servos testing
                                # motorpitx

                                print "p2go setup"
                                anyAddOns = True

                        if "apb01" in ADDON:
                            with lock:
                                sghGC.resetPinMode()
                                sghGC.pinUse[7] = sghGC.PINPUT  # LFLeft
                                sghGC.pinUse[11] = sghGC.PINPUT  # LFRight

                                sghGC.setPinMode()
                                sghGC.motorUpdate(21, 19, 0)
                                sghGC.motorUpdate(24, 26, 0)

                                self.startUltra(23, 0, self.OnOrOff)

                            print "apb01 setup"
                            anyAddOns = True

                            # if "agobop2" in ADDON:
                            # with lock:
                            # sghGC.resetPinMode()
                            # sghGC.pinUse[7]  = sghGC.PINPUT #LFLeft
                            # sghGC.pinUse[11] = sghGC.PINPUT #LFRight
                            # sghGC.pinUse[15]  = sghGC.POUTPUT
                            # sghGC.pinUse[13] = sghGC.POUTPUT

                            # sghGC.setPinMode()
                            # sghGC.motorUpdate(19,21,0)
                            # sghGC.motorUpdate(26,24,0)

                            # self.startUltra(23,0,self.OnOrOff)

                            # print "Agobo2 setup"
                            # anyAddOns = True

                        if "agobo" in ADDON:
                            with lock:
                                sghGC.resetPinMode()
                                sghGC.pinUse[7] = sghGC.PINPUT  # LFLeft
                                sghGC.pinUse[11] = sghGC.PINPUT  # LFRight
                                sghGC.pinUse[16] = sghGC.PINPUT  # Switch
                                sghGC.pinUse[15] = sghGC.POUTPUT
                                sghGC.pinUse[13] = sghGC.POUTPUT

                                sghGC.setPinMode()
                                sghGC.motorUpdate(19, 21, 0)
                                sghGC.motorUpdate(26, 24, 0)

                                self.startUltra(23, 0, self.OnOrOff)

                            print "Agobo setup"
                            anyAddOns = True

                        if "happi" in ADDON:
                            with lock:
                                sghGC.resetPinMode()
                                sghGC.pinUse[11] = sghGC.POUTPUT  # Motor1
                                sghGC.pinUse[12] = sghGC.POUTPUT  # Motor1
                                sghGC.pinUse[15] = sghGC.POUTPUT  # Motor2
                                sghGC.pinUse[16] = sghGC.POUTPUT  # Motor2
                                sghGC.pinUse[3] = sghGC.PINPUT
                                sghGC.pinUse[5] = sghGC.PINPUT
                                sghGC.pinUse[7] = sghGC.PINPUT
                                sghGC.pinUse[8] = sghGC.PINPUT
                                sghGC.pinUse[10] = sghGC.PINPUT
                                sghGC.pinUse[13] = sghGC.PINPUT
                                sghGC.pinUse[18] = sghGC.PINPUT
                                sghGC.pinUse[19] = sghGC.PINPUT
                                sghGC.pinUse[21] = sghGC.PINPUT
                                sghGC.pinUse[22] = sghGC.PINPUT
                                sghGC.pinUse[23] = sghGC.PINPUT
                                sghGC.pinUse[24] = sghGC.PINPUT
                                sghGC.pinUse[26] = sghGC.PINPUT

                                sghGC.setPinMode()

                                # sghGC.startServod([12,10]) # servos testing
                                # motorpitx

                                print "HapPi setup"
                                anyAddOns = True

                        if "raspibot2" in ADDON:
                            with lock:
                                sghGC.resetPinMode()

                                sghGC.pinUse[11] = sghGC.POUTPUT  # left go
                                sghGC.pinUse[7] = sghGC.POUTPUT  # left dir
                                sghGC.pinUse[19] = sghGC.POUTPUT  # right go
                                sghGC.pinUse[22] = sghGC.POUTPUT  # right dir
                                sghGC.pinUse[15] = sghGC.POUTPUT  # oc1
                                sghGC.pinUse[13] = sghGC.POUTPUT  # oc2
                                sghGC.pinUse[23] = sghGC.PINPUT  # sw1 pin
                                sghGC.pinUse[21] = sghGC.PINPUT  # sw2 pin
                                sghGC.pinUse[26] = sghGC.POUTPUT  # LED1
                                sghGC.pinUse[24] = sghGC.POUTPUT  # LED2
                                sghGC.pinUse[12] = sghGC.PSONAR  # trigger
                                sghGC.pinUse[16] = sghGC.PSONAR  # echo

                                sghGC.setPinMode()

                                print "RaspPiBot2 setup"
                                anyAddOns = True

                        if "pizazz" in ADDON:
                            with lock:
                                sghGC.resetPinMode()
                                sghGC.setAllInvert(
                                    True)  # GPIO pull down each led so need to invert 0 to 1 and vice versa
                                # sghGC.pinUse[19] = sghGC.POUTPUT #MotorA
                                # sghGC.pinUse[21] = sghGC.POUTPUT #MotorA
                                # sghGC.pinUse[26] = sghGC.POUTPUT #MotorB
                                # sghGC.pinUse[24] = sghGC.POUTPUT #MotorB
                                sghGC.pinUse[7] = sghGC.POUTPUT  # LED
                                sghGC.pinUse[11] = sghGC.POUTPUT  # LED
                                sghGC.pinUse[18] = sghGC.POUTPUT  # LED
                                sghGC.pinUse[22] = sghGC.POUTPUT  # LED
                                sghGC.pinUse[12] = sghGC.PINPUT  # LFLeft
                                sghGC.pinUse[13] = sghGC.PINPUT  # LFRight

                                sghGC.setPinMode()
                                sghGC.motorUpdate(19, 21, 0)
                                sghGC.motorUpdate(24, 26, 0)
                                #sghGC.pinEventEnabled = 0

                                self.startUltra(8, 0, self.OnOrOff)

                                print "Pizazz setup"
                                anyAddOns = True
                        if "simpie" in ADDON:
                            with lock:
                                sghGC.resetPinMode()
                                sghGC.setAllInvert(
                                    True)  # GPIO pull down each led so need to invert 0 to 1 and vice versa
                                sghGC.pinUse[11] = sghGC.PINPUT  # Red
                                sghGC.pinUse[13] = sghGC.PINPUT  # Amber
                                sghGC.pinUse[15] = sghGC.PINPUT  # Green
                                sghGC.pinUse[12] = sghGC.POUTPUT  # Red
                                sghGC.pinUse[16] = sghGC.POUTPUT  # Green
                                sghGC.pinUse[18] = sghGC.POUTPUT  # Blue
                                sghGC.pinUse[7] = sghGC.POUTPUT  # Buzzer

                                sghGC.setPinMode()
                                sghGC.pinUpdate(7, 0)

                                print "SimPie setup"
                                anyAddOns = True

                        if "techtom" in ADDON:
                            with lock:
                                sghGC.resetPinMode()
                                # sghGC.INVERT = True # GPIO pull down each led
                                # so need to invert 0 to 1 and vice versa
                                sghGC.pinUse[15] = sghGC.PINPUT  # Red
                                sghGC.pinUse[19] = sghGC.PINPUT  # Amber
                                sghGC.pinUse[21] = sghGC.PINPUT  # Green
                                sghGC.pinUse[23] = sghGC.PINPUT  # Green
                                sghGC.pinUse[8] = sghGC.POUTPUT  # Red
                                sghGC.pinUse[10] = sghGC.POUTPUT  # Green
                                sghGC.pinUse[12] = sghGC.POUTPUT  # Blue
                                sghGC.pinUse[16] = sghGC.POUTPUT  # Buzzer
                                sghGC.pinUse[18] = sghGC.POUTPUT  # Red
                                sghGC.pinUse[22] = sghGC.POUTPUT  # Green
                                sghGC.pinUse[24] = sghGC.POUTPUT  # Blue
                                sghGC.pinUse[24] = sghGC.POUTPUT  # Buzzer

                                sghGC.setPinMode()

                                print "SimPie setup"
                                anyAddOns = True

                        if "ledborg" in ADDON:
                            with lock:
                                sghGC.resetPinMode()
                                for pin in [11, 13, 15]:
                                    sghGC.pinUse[pin] = sghGC.POUTPUT
                                sghGC.setPinMode()
                                anyAddOns = True

                        if "mearm" in ADDON:
                            with lock:
                                sghGC.resetPinMode()
                                # sghGC.INVERT = True # GPIO pull down each led
                                # so need to invert 0 to 1 and vice versa
                                sghGC.setPinMode()
                                if pcaPWM is not None:
                                    self.arm = meArm.meArm()
                                    self.arm.begin()

                                print "MeArm setup"
                                anyAddOns = True

                        if "flotilla" in ADDON:
                            print "flotilla", ADDON
                            with lock:
                                sghGC.resetPinMode()
                                anyAddOns = True

                        if "pitt" in ADDON:
                            with lock:
                                sghGC.resetPinMode()
                                for pin in [8, 10, 12, 16, 18, 22, 24, 26]:
                                    sghGC.pinUse[pin] = sghGC.POUTPUT
                                for pin in [15, 19, 21, 23]:
                                    sghGC.pinUse[pin] = sghGC.PINPUT
                                sghGC.setPinMode()
                                for pin in [8, 10, 12, 16, 18, 22, 24, 26]:
                                    # turn all the pins physically off
                                    sghGC.pinUpdate(pin, 1)
                                anyAddOns = True

                        if "piandbash" in ADDON:
                            mcp.config(0, mcp.OUTPUT)  # LCD Backlight

                            # for loop in range(1,4):
                            #     mcp.config(loop, mcp.OUTPUT)
                            #     sghGC.piAndBash[loop] = sghGC.POUTPUT
                            #     mcp.output(loop, 0)

                            for loop in range(1, 8):
                                mcp.config(loop, mcp.INPUT)
                                mcp.pullup(loop, 1)
                                sghGC.piAndBash[loop] = sghGC.PINPUT

                            mcp.config(8, mcp.OUTPUT)  # Green LED

                            # Inputs:
                            mcp.config(9, mcp.INPUT)  # Up Button
                            mcp.pullup(9, 1)
                            mcp.config(10, mcp.OUTPUT)  # Amber LED
                            mcp.config(11, mcp.INPUT)  # Enter Button
                            mcp.pullup(11, 1)
                            mcp.config(12, mcp.OUTPUT)  # Red LED
                            mcp.config(13, mcp.INPUT)  # Down Button
                            mcp.pullup(13, 1)
                            # LCD Upper Line Select Button
                            mcp.config(14, mcp.INPUT)
                            mcp.pullup(14, 1)
                            # LCD Lower Line Select Button
                            mcp.config(15, mcp.INPUT)
                            mcp.pullup(15, 1)

                            mcp.output(8, 0)
                            mcp.output(10, 0)
                            mcp.output(12, 0)

                            pnblcd = sgh_pnbLCD.sgh_pnbLCD()
                            # Open SPI bus
                            spi = spidev.SpiDev()
                            spi.open(0, 0)

                        if "pipiano" in ADDON:
                            for loop in range(0, 13):
                                mcp.config(loop, mcp.INPUT)
                                mcp.pullup(loop, 1)
                                sghGC.piAndBash[loop] = sghGC.PINPUT

                            # mcp.config(8, mcp.OUTPUT)  # Green LED

                        if "explorer" in ADDON:
                            with lock:
                                sghGC.resetPinMode()
                                for pin in [7, 11, 13, 29, 31, 32, 33, 36, 40, 37, 38, 35]:
                                    sghGC.pinUse[pin] = sghGC.POUTPUT
                                for pin in [16, 15, 18, 22]:
                                    sghGC.pinUse[pin] = sghGC.PINPUT
                                sghGC.setPinMode()
                                for pin in [7, 11, 13, 29, 31, 32, 33, 36, 40, 37, 38, 35]:
                                    # turn all the pins physically off
                                    sghGC.pinUpdate(pin, 0)
                                anyAddOns = True

                        for pin in sghGC.validPins:
                            if (sghGC.pinUse[pin] in [sghGC.PINPUT, sghGC.PINPUTNONE, sghGC.PINPUTDOWN]):
                                sghGC.pinTriggerLastState[
                                    pin] = sghGC.pinRead(pin)
                                print "pinTriggerLastState", pin, sghGC.pinTriggerLastState[pin]

                                # if (firstRun == True) and (anyAddOns == False): # if no addon found in firstrun then assume default configuration
                                # with lock:
                                # print "no AddOns Declared"
                                # sghGC.pinUse[11] = sghGC.POUTPUT
                                # sghGC.pinUse[12] = sghGC.POUTPUT
                                # sghGC.pinUse[13] = sghGC.POUTPUT
                                # sghGC.pinUse[15] = sghGC.POUTPUT
                                # sghGC.pinUse[16] = sghGC.POUTPUT
                                # sghGC.pinUse[18] = sghGC.POUTPUT
                                # sghGC.pinUse[7]  = sghGC.PINPUT
                                # sghGC.pinUse[22] = sghGC.PINPUT
                                # sghGC.setPinMode()

                                # firstRun = False

                # If outputs need  inverting (7 segment common anode needs it -
                # PiRingo etc)

                if self.bFind("invert"):  # update pin count values
                    if self.bFindOnOff('invert'):
                        print "global invert set"
                        for pin in sghGC.validPins:  # loop thru all pins
                            sghGC.pinInvert[pin] = self.OnOrOff
                    else:
                        for pin in sghGC.validPins:  # loop thru all pins
                            if self.bFindOnOff('invert' + str(pin)):
                                sghGC.pinInvert[pin] = self.OnOrOff
                                print "invert status pin", pin, "is", sghGC.pinInvert[pin]

                # Change pins from input to output if more needed
                if self.bFind('config'):
                    with lock:
                        for pin in sghGC.validPins:
                            # print "checking pin" ,pin
                            if self.bFindValue('config' + str(pin)):
                                if self.value == "in":
                                    sghGC.pinUse[pin] = sghGC.PINPUT
                                if self.value == "inpulldown":
                                    sghGC.pinUse[pin] = sghGC.PINPUTDOWN
                                if self.value == "inpullnone":
                                    sghGC.pinUse[pin] = sghGC.PINPUTNONE

                        sghGC.setPinMode()
                        # Check for AddOn boards being declared

                # Listen for Variable changes
                if 'sensor-update' in self.dataraw:
                    if "pitt" in ADDON:
                        if self.vFindValue("output"):
                            if self.valueIsNumeric:
                                binstring = bin(
                                    max(0, min(int(self.valueNumeric), 255)))[2:]
                                print binstring
                                bit_pattern = ('00000000' + binstring)[-8:]
                                print 'bit_pattern %s' % bit_pattern
                                j = 0
                                for pin in [8, 10, 12, 16, 18, 22, 24, 26]:
                                    # print "pin" , bit_pattern[-(j+1)]
                                    if bit_pattern[-(j + 1)] == '0':
                                        # output inverted as board is
                                        # physically active low
                                        sghGC.pinUpdate(pin, 1)
                                    else:
                                        sghGC.pinUpdate(pin, 0)
                                    j += 1

                    if piglow is not None:
                        # do PiGlow stuff but make sure PiGlow physically
                        # detected

                        # check LEDS
                        for i in range(1, 19):
                            if self.vFindValue('led' + str(i)):
                                svalue = int(
                                    self.valueNumeric) if self.valueIsNumeric else 0
                                svalue = min(255, max(svalue, 0))
                                PiGlow_Values[PiGlow_Lookup[i - 1]] = svalue
                                piglow.update_pwm_values(PiGlow_Values)

                        for i in range(1, 4):
                            if self.vFindValue('leg' + str(i)):
                                svalue = int(
                                    self.valueNumeric) if self.valueIsNumeric else 0
                                svalue = min(255, max(svalue, 0))
                                PiGlow_Values[
                                    PiGlow_Lookup[((i - 1) * 6) + 0]] = svalue
                                PiGlow_Values[
                                    PiGlow_Lookup[((i - 1) * 6) + 1]] = svalue
                                PiGlow_Values[
                                    PiGlow_Lookup[((i - 1) * 6) + 2]] = svalue
                                PiGlow_Values[
                                    PiGlow_Lookup[((i - 1) * 6) + 3]] = svalue
                                PiGlow_Values[
                                    PiGlow_Lookup[((i - 1) * 6) + 4]] = svalue
                                PiGlow_Values[
                                    PiGlow_Lookup[((i - 1) * 6) + 5]] = svalue
                                piglow.update_pwm_values(PiGlow_Values)

                            if self.vFindValue('arm' + str(i)):
                                svalue = int(
                                    self.valueNumeric) if self.valueIsNumeric else 0
                                svalue = min(255, max(svalue, 0))
                                PiGlow_Values[
                                    PiGlow_Lookup[((i - 1) * 6) + 0]] = svalue
                                PiGlow_Values[
                                    PiGlow_Lookup[((i - 1) * 6) + 1]] = svalue
                                PiGlow_Values[
                                    PiGlow_Lookup[((i - 1) * 6) + 2]] = svalue
                                PiGlow_Values[
                                    PiGlow_Lookup[((i - 1) * 6) + 3]] = svalue
                                PiGlow_Values[
                                    PiGlow_Lookup[((i - 1) * 6) + 4]] = svalue
                                PiGlow_Values[
                                    PiGlow_Lookup[((i - 1) * 6) + 5]] = svalue
                                piglow.update_pwm_values(PiGlow_Values)

                        pcolours = [
                            'red', 'orange', 'yellow', 'green', 'blue', 'white']
                        for i in range(len(pcolours)):
                            if self.vFindValue(pcolours[i]):
                                svalue = int(
                                    self.valueNumeric) if self.valueIsNumeric else 0
                                svalue = min(255, max(svalue, 0))
                                PiGlow_Values[PiGlow_Lookup[i + 0]] = svalue
                                PiGlow_Values[PiGlow_Lookup[i + 6]] = svalue
                                PiGlow_Values[PiGlow_Lookup[i + 12]] = svalue
                                piglow.update_pwm_values(PiGlow_Values)

                        # Use bit pattern to control leds
                        if self.vFindValue('ledpattern'):
                            # print 'Found ledpattern'
                            num_of_bits = 18
                            bit_pattern = (
                                '00000000000000000000000000' + self.value)[-num_of_bits:]
                            # print 'led_pattern %s' % bit_pattern
                            j = 0
                            for i in range(18):
                                #bit_state = ((2**i) & sensor_value) >> i
                                # print 'dummy pin %d state %d' % (i,
                                # bit_state)
                                if bit_pattern[-(j + 1)] == '0':
                                    PiGlow_Values[PiGlow_Lookup[i]] = 0
                                else:
                                    PiGlow_Values[PiGlow_Lookup[i]] = 1
                                j += 1

                            piglow.update_pwm_values(PiGlow_Values)

                            # Replaced by global bright variable code
                            # if self.vFindValue('bright'):
                            #    svalue = int(self.valueNumeric) if self.valueIsNumeric else 0
                            #    svalue= min(255,max(svalue,0))
                            #    PiGlow_Brightness = svalue

                    if self.vFindValue("x"):
                        self.matrixX = int(
                            self.valueNumeric) if self.valueIsNumeric else 0
                        self.matrixX = min(7, max(self.matrixX, 0))

                    if self.vFindValue("y"):
                        self.matrixY = int(
                            self.valueNumeric) if self.valueIsNumeric else 0
                        self.matrixY = min(7, max(self.matrixY, 0))

                    if self.vFindValue("matrixuse"):
                        self.matrixUse = int(self.valueNumeric) if self.value in [
                            '64', '16', '9', '4'] else 64

                        # if self.vFindValue("matrixuse"):
                        # self.matrixUse= int(self.valueNumeric) if self.valueIsNumeric else 64
                        # if self.matrixUse == 0:
                        # self.matrixUse = 64
                        # else:
                        # self.matrixUse = min(64,max(self.matrixUse,1))

                    if self.vFindValue("matrixrotate"):
                        print "mr"
                        try:
                            AdaMatrix.setRotate(
                                min(3, max(int(self.valueNumeric) if self.valueIsNumeric else 0, 0)))
                        except:
                            pass
                    # print "sensor-update rcvd" , dataraw

                    if "ladder" in ADDON:
                        # do ladderboard stuff

                        # check All LEDS On/Off/High/Low/1/0
                        self.vAllCheck("leds")

                        self.vLEDCheck(ladderOutputs)

                    elif "motorpitx" in ADDON:
                        # do MotorPiTx stuff
                        # check for motor variable commands
                        self.vListCheck(
                            [15, 11, 13, 7], ["output1", "output2", "input1", "input2"])
                        moveServos = False

                        if self.vFindValue('tiltoffset'):
                            tiltoffset = int(
                                self.valueNumeric) if self.valueIsNumeric else 0
                            moveServos = True

                        if self.vFindValue('panoffset'):
                            panoffset = int(
                                self.valueNumeric) if self.valueIsNumeric else 0
                            moveServos = True

                        if self.vFindValue('tilt'):
                            # print "tilt command rcvd"
                            if self.valueIsNumeric:
                                tilt = int(self.valueNumeric)
                                moveServos = True
                                # print "tilt=", tilt
                            elif self.value == "off":
                                os.system(
                                    "echo " + "0" + "=0 > /dev/servoblaster")
                        else:
                            if self.vFindValue('servo1'):
                                # print "tilt command rcvd"
                                if self.valueIsNumeric:
                                    tilt = int(self.valueNumeric)
                                    moveServos = True
                                    # print "tilt=", tilt
                                elif self.value == "off":
                                    sghGC.pinServod(12, "off")

                        if self.vFindValue('pan'):
                            # print "pan command rcvd"
                            if self.valueIsNumeric:
                                pan = int(self.valueNumeric)
                                moveServos = True
                                # print "pan=", pan
                            elif self.value == "off":
                                os.system(
                                    "echo " + "1" + "=0 > /dev/servoblaster")
                        else:
                            if self.vFindValue('servo2'):
                                # print "servob command rcvd"
                                if self.valueIsNumeric:
                                    pan = int(self.valueNumeric)
                                    moveServos = True
                                    # print "servob=", pan
                                elif self.value == "off":
                                    sghGC.pinServod(10, "off")

                        if moveServos:
                            # print "move servos == True"
                            degrees = int(tilt + tiltoffset)
                            degrees = min(90, max(degrees, -90))
                            servodvalue = 50 + ((90 - degrees) * 200 / 180)
                            sghGC.pinServod(12, servodvalue)
                            degrees = int(pan + panoffset)
                            degrees = min(90, max(degrees, -90))
                            servodvalue = 50 + ((90 - degrees) * 200 / 180)
                            # print "Value being sent to pin 10:",servodvalue
                            sghGC.pinServod(10, servodvalue)

                        # check for motor variable commands
                        motorList = [
                            ['motor1', 19, 21, 23], ['motor2', 18, 16, 22]]
                        for listLoop in range(0, 2):
                            if self.vFindValue(motorList[listLoop][0]):
                                svalue = int(
                                    self.valueNumeric) if self.valueIsNumeric else 0
                                # This technique can be used if enabel is held
                                # high by hardware
                                if svalue > 0:
                                    sghGC.pinUpdate(
                                        motorList[listLoop][1], (svalue), "pwmmotor")
                                    sghGC.pinUpdate(motorList[listLoop][2], 0)
                                    # set enable to 1
                                    sghGC.pinUpdate(motorList[listLoop][3], 1)
                                elif svalue < 0:
                                    sghGC.pinUpdate(motorList[listLoop][1], 0)
                                    sghGC.pinUpdate(
                                        motorList[listLoop][2], (svalue), "pwmmotor")
                                    # set enable to 1
                                    sghGC.pinUpdate(motorList[listLoop][3], 1)
                                else:
                                    sghGC.pinUpdate(motorList[listLoop][3], 0)
                                    sghGC.pinUpdate(motorList[listLoop][1], 0)
                                    sghGC.pinUpdate(motorList[listLoop][2], 0)

                    elif ((piglow is not None) and ("piglow" in ADDON)):
                        # do PiGlow stuff but make sure PiGlow physically
                        # detected

                        # check LEDS
                        for i in range(1, 19):
                            if self.vFindValue('led' + str(i)):
                                svalue = int(
                                    self.valueNumeric) if self.valueIsNumeric else 0
                                svalue = min(255, max(svalue, 0))
                                PiGlow_Values[PiGlow_Lookup[i - 1]] = svalue
                                piglow.update_pwm_values(PiGlow_Values)

                        for i in range(1, 4):
                            if self.vFindValue('leg' + str(i)):
                                svalue = int(
                                    self.valueNumeric) if self.valueIsNumeric else 0
                                svalue = min(255, max(svalue, 0))
                                PiGlow_Values[
                                    PiGlow_Lookup[((i - 1) * 6) + 0]] = svalue
                                PiGlow_Values[
                                    PiGlow_Lookup[((i - 1) * 6) + 1]] = svalue
                                PiGlow_Values[
                                    PiGlow_Lookup[((i - 1) * 6) + 2]] = svalue
                                PiGlow_Values[
                                    PiGlow_Lookup[((i - 1) * 6) + 3]] = svalue
                                PiGlow_Values[
                                    PiGlow_Lookup[((i - 1) * 6) + 4]] = svalue
                                PiGlow_Values[
                                    PiGlow_Lookup[((i - 1) * 6) + 5]] = svalue
                                piglow.update_pwm_values(PiGlow_Values)

                            if self.vFindValue('arm' + str(i)):
                                svalue = int(
                                    self.valueNumeric) if self.valueIsNumeric else 0
                                svalue = min(255, max(svalue, 0))
                                PiGlow_Values[
                                    PiGlow_Lookup[((i - 1) * 6) + 0]] = svalue
                                PiGlow_Values[
                                    PiGlow_Lookup[((i - 1) * 6) + 1]] = svalue
                                PiGlow_Values[
                                    PiGlow_Lookup[((i - 1) * 6) + 2]] = svalue
                                PiGlow_Values[
                                    PiGlow_Lookup[((i - 1) * 6) + 3]] = svalue
                                PiGlow_Values[
                                    PiGlow_Lookup[((i - 1) * 6) + 4]] = svalue
                                PiGlow_Values[
                                    PiGlow_Lookup[((i - 1) * 6) + 5]] = svalue
                                piglow.update_pwm_values(PiGlow_Values)

                        pcolours = [
                            'red', 'orange', 'yellow', 'green', 'blue', 'white']
                        for i in range(len(pcolours)):
                            if self.vFindValue(pcolours[i]):
                                svalue = int(
                                    self.valueNumeric) if self.valueIsNumeric else 0
                                svalue = min(255, max(svalue, 0))
                                PiGlow_Values[PiGlow_Lookup[i + 0]] = svalue
                                PiGlow_Values[PiGlow_Lookup[i + 6]] = svalue
                                PiGlow_Values[PiGlow_Lookup[i + 12]] = svalue
                                piglow.update_pwm_values(PiGlow_Values)

                        # Use bit pattern to control leds
                        if self.vFindValue('ledpattern'):
                            # print 'Found ledpattern'
                            num_of_bits = 18
                            bit_pattern = (
                                '00000000000000000000000000' + self.value)[-num_of_bits:]
                            # print 'led_pattern %s' % bit_pattern
                            j = 0
                            for i in range(18):
                                #bit_state = ((2**i) & sensor_value) >> i
                                # print 'dummy pin %d state %d' % (i,
                                # bit_state)
                                if bit_pattern[-(j + 1)] == '0':
                                    PiGlow_Values[PiGlow_Lookup[i]] = 0
                                else:
                                    PiGlow_Values[PiGlow_Lookup[i]] = 1
                                j += 1

                            piglow.update_pwm_values(PiGlow_Values)

                            # Replaced by global bright variable code
                            # if self.vFindValue('bright'):
                            #    svalue = int(self.valueNumeric) if self.valueIsNumeric else 0
                            #    svalue= min(255,max(svalue,0))
                            #    PiGlow_Brightness = svalue

                    elif "gpio" in ADDON:
                        # do gPiO stuff

                        # check Allpins On/Off/High/Low/1/0
                        self.vAllCheck("allpins")

                        # check for any pin On/Off/High/Low/1/0 any PWM
                        # settings using power or motor
                        self.vPinCheck()

                        # check for motor variable commands
                        motorList = [['motora', 11, 12], ['motorb', 13, 15, ]]
                        #motorList = [['motora',21,26],['motorb',19,24]]
                        for listLoop in range(0, 2):
                            if self.vFindValue(motorList[listLoop][0]):
                                svalue = min(
                                    100, max(-100, int(self.valueNumeric))) if self.valueIsNumeric else 0
                                logging.debug(
                                    "motor:%s valuee:%s", motorList[listLoop][0], svalue)
                                sghGC.motorUpdate(
                                    motorList[listLoop][1], motorList[listLoop][2], svalue)

                                # End of gPiO Variable handling

                    elif "berry" in ADDON:
                        # do BerryClip stuff
                        # check All LEDS On/Off/High/Low/1/0
                        self.vAllCheck("leds")

                        # check All LEDS On/Off/High/Low/1/0
                        self.vLEDCheck(berryOutputs)

                        if self.vFindOnOff('buzzer'):
                            self.index_pin_update(24, self.valueNumeric)

                            # End of BerryClip Variable handling

                    elif "pirocon" in ADDON:
                        # do PiRoCon stuff
                        #logging.debug("Processing variables for PiRoCon")
                        # print "panoffset" , panoffset, "tilt",tiltoffset
                        moveServos = False

                        if self.vFindValue('tiltoffset'):
                            tiltoffset = int(
                                self.valueNumeric) if self.valueIsNumeric else 0
                            moveServos = True

                        if self.vFindValue('panoffset'):
                            panoffset = int(
                                self.valueNumeric) if self.valueIsNumeric else 0
                            moveServos = True

                        if self.vFindValue('tilt'):
                            # print "tilt command rcvd"
                            if self.valueIsNumeric:
                                tilt = int(self.valueNumeric)
                                moveServos = True
                                # print "tilt=", tilt
                            elif self.value == "off":
                                os.system(
                                    "echo " + "0" + "=0 > /dev/servoblaster")
                        else:
                            if self.vFindValue('servoa'):
                                # print "tilt command rcvd"
                                if self.valueIsNumeric:
                                    tilt = int(self.valueNumeric)
                                    moveServos = True
                                    # print "tilt=", tilt
                                elif self.value == "off":
                                    os.system(
                                        "echo " + "0" + "=0 > /dev/servoblaster")

                        if self.vFindValue('pan'):
                            # print "pan command rcvd"
                            if self.valueIsNumeric:
                                pan = int(self.valueNumeric)
                                moveServos = True
                                # print "pan=", pan
                            elif self.value == "off":
                                os.system(
                                    "echo " + "1" + "=0 > /dev/servoblaster")
                        else:
                            if self.vFindValue('servob'):
                                # print "pan command rcvd"
                                if self.valueIsNumeric:
                                    pan = int(self.valueNumeric)
                                    moveServos = True
                                    # print "pan=", pan
                                elif self.value == "off":
                                    os.system(
                                        "echo " + "1" + "=0 > /dev/servoblaster")

                        if moveServos:
                            degrees = int(tilt + tiltoffset)
                            degrees = min(80, max(degrees, -60))
                            servodvalue = 50 + ((90 - degrees) * 200 / 180)
                            # print "sending", servodvalue, "to servod"
                            #os.system("echo " + "0" + "=" + str(servodvalue-1) + " > /dev/servoblaster")
                            sghGC.pinServod(18, servodvalue)  # orig =18
                            #os.system("echo " + "0" + "=" + str(servodvalue) + " > /dev/servoblaster")
                            degrees = int(pan + panoffset)
                            degrees = min(90, max(degrees, -90))
                            servodvalue = 50 + ((90 - degrees) * 200 / 180)
                            sghGC.pinServod(22, servodvalue)  # orig =22
                            #os.system("echo " + "1" + "=" + str(servodvalue) + " > /dev/servoblaster")

                        # check for motor variable commands
                        motorList = [['motora', 21, 26, 0], ['motorb', 19, 24]]
                        if "piroconb" in ADDON:
                            logging.debug("PiRoConB Found:%s", ADDON)
                            motorList = [
                                ['motora', 21, 19, 0, False], ['motorb', 26, 24, 0, False]]

                        for listLoop in range(0, 2):
                            if self.vFindValue(motorList[listLoop][0]):
                                svalue = min(
                                    100, max(-100, int(self.valueNumeric))) if self.valueIsNumeric else 0
                                logging.debug(
                                    "motor:%s valuee:%s", motorList[listLoop][0], svalue)
                                sghGC.motorUpdate(
                                    motorList[listLoop][1], motorList[listLoop][2], svalue)

                                # End of PiRoCon Variable handling
                    elif "piringo" in ADDON:
                        # do piringo stuff

                        # check All LEDS On/Off/High/Low/1/0
                        self.vAllCheck("leds")

                        self.vLEDCheck(piringoOutputs)

                    elif "pibrella" in ADDON:  # PiBrella

                        # check All On/Off/High/Low/1/0
                        self.vAllCheck("allpins")

                        self.vListCheck([13, 11, 7, 15, 16, 18, 22],
                                        ["led1", "led2", "led3", "led4", "led5", "led6", "led7"])
                        self.vListCheck([13, 11, 11, 11, 7, 15, 16, 18, 22],
                                        ["red", "amber", "yellow", "orange", "green", "outpute", "outputf", "outputg",
                                         "outputh"])
                        self.vListCheckMotorOnly(
                            [15, 16, 18, 22], ["e", "f", "g", "h"])

                        if self.vFindValue('stepper'):
                            if self.valueIsNumeric:
                                self.stepperUpdate(
                                    [15, 16, 18, 22], self.valueNumeric)
                            else:
                                self.stepperUpdate([15, 16, 18, 22], 0)

                        if self.vFindValue("beep"):
                            try:
                                bn, bd = self.value.split(",")
                            except:
                                bn = "60"
                                bd = "1"
                            beepNote = int(float(bn))
                            beepDuration = (float(bd))
                            svalue = int(
                                self.valueNumeric) if self.valueIsNumeric else 60
                            beepThread = threading.Thread(target=self.beep,
                                                          args=[12, 440 * 2 ** ((beepNote - 69) / 12.0), beepDuration])
                            beepThread.start()

                            # if self.vFindValue("beepnote"):
                            # beepNote = max(12,int(self.valueNumeric)) if self.valueIsNumeric else 60

                            # if self.vFindValue("beepduration"):
                            # beepDuration = max(0.125,int(self.valueNumeric)) if self.valueIsNumeric else 0.5

                    # RGB-LED by Meltwater/rsstab/tim cox
                    elif "rgbled" in ADDON:

                        #print ("rgb-led variable processing")
                        if self.vFindOnOff("all"):
                            for loop in range(0, 5):
                                sghGC.pinUpdate(
                                    rgbOutputs[loop], 1 - self.valueNumeric)
                            for loop in range(5, 8):
                                sghGC.pinUpdate(
                                    rgbOutputs[loop], self.valueNumeric)

                        rgbList = rgbOutputs[0:5]
                        for listLoop in rgbList:
                            if self.vFindOnOff("led" + str(1 + rgbList.index(listLoop))):
                                sghGC.pinUpdate(
                                    rgbOutputs[rgbList.index(listLoop)], 1 - self.valueNumeric)
                            if self.vFindValue("power" + str(1 + rgbList.index(listLoop))):
                                if self.valueIsNumeric:
                                    sghGC.pinUpdate(
                                        rgbOutputs[rgbList.index(listLoop)], 100 - self.valueNumeric, "pwm")
                                else:
                                    sghGC.pinUpdate(
                                        rgbOutputs[rgbList.index(listLoop)], 1)

                        rgbList = ["red", "green", "blue"]
                        for listLoop in rgbList:
                            if self.vFindOnOff(listLoop):
                                print listLoop, "found",
                                sghGC.pinUpdate(
                                    rgbOutputs[5 + rgbList.index(listLoop)], self.valueNumeric)

                    elif "rtkrpimcb" in ADDON:
                        # check for motor variable commands
                        motorList = [['motor1', 11, 12], ['motor2', 15, 16]]
                        for listLoop in range(0, 2):
                            if self.vFindValue(motorList[listLoop][0]):
                                svalue = int(
                                    self.valueNumeric) if self.valueIsNumeric else 0
                                if svalue > 0:
                                    sghGC.pinUpdate(motorList[listLoop][2], 1)
                                    sghGC.pinUpdate(
                                        motorList[listLoop][1], (100 - svalue), "pwmmotor")
                                elif svalue < 0:
                                    sghGC.pinUpdate(motorList[listLoop][2], 0)
                                    sghGC.pinUpdate(
                                        motorList[listLoop][1], (svalue), "pwmmotor")
                                else:
                                    sghGC.pinUpdate(motorList[listLoop][1], 0)
                                    sghGC.pinUpdate(motorList[listLoop][2], 0)

                    elif "pidie" in ADDON:
                        # check All LEDS On/Off/High/Low/1/0
                        self.vAllCheck("leds")
                        self.vListCheck([7, 11, 12, 13, 15, 16, 18, 22, 8],
                                        ["led1", "led2", "led3", "led4", "led5", "led6", "led7", "led8", "led9"])
                        self.vListCheckPowerOnly([7, 11, 12, 13, 15, 16, 18, 22, 8],
                                                 ["1", "2", "3", "4", "5", "6", "7", "8", "9"])

                    elif "fishdish" in ADDON:
                        # do fishdish stuff
                        # check All LEDS On/Off/High/Low/1/0
                        self.vAllCheck("leds")

                        # check All LEDS On/Off/High/Low/1/0
                        self.vLEDCheck(fishOutputs)

                        if self.vFindOnOff('buzzer'):
                            self.index_pin_update(24, self.valueNumeric)

                    elif "traffichat" in ADDON:
                        # do traffichat stuff
                        # check All LEDS On/Off/High/Low/1/0
                        self.vAllCheck("leds")

                        # check All LEDS On/Off/High/Low/1/0
                        self.vLEDCheck(traffichatOutputs)
                        #traffichatOutputs = [15, 16, 18, 29]

                        if self.vFindOnOff('buzzer'):
                            print("buzz")
                            sghGC.pinUpdate(29, self.valueNumeric)

                        if self.vFindOnOff('green'):
                            print (self.valueNumeric)
                            sghGC.pinUpdate(15, self.valueNumeric)

                        if self.vFindOnOff('yellow'):
                            sghGC.pinUpdate(16, self.valueNumeric)

                        if self.vFindOnOff('red'):
                            sghGC.pinUpdate(18, self.valueNumeric)

                    elif "p2g3" in ADDON:
                        # do PiRoCon stuff
                        #logging.debug("Processing variables for P2G3")

                        # check for motor variable commands
                        motorList = [
                            ['motorb', 19, 21, 0, False], ['motora', 26, 24, 0, False]]
                        #logging.debug("ADDON:%s", ADDON)

                        for listLoop in range(0, 2):
                            if self.vFindValue(motorList[listLoop][0]):
                                svalue = min(
                                    100, max(-100, int(self.valueNumeric))) if self.valueIsNumeric else 0
                                logging.debug(
                                    "motor:%s valuee:%s", motorList[listLoop][0], svalue)
                                sghGC.motorUpdate(
                                    motorList[listLoop][1], motorList[listLoop][2], svalue)
                                # for listLoop in range(0,2):
                                # if self.vFindValue(motorList[listLoop][0]):
                                # svalue = int(self.valueNumeric) if self.valueIsNumeric else 0
                                # #logging.debug("svalue %s %s", motorList[listLoop][0],svalue)
                                # if svalue > 0:
                                # sghGC.pinUpdate(motorList[listLoop][2],1)
                                # sghGC.pinUpdate(motorList[listLoop][1],(100-svalue),"pwmmotor")
                                # elif svalue < 0:
                                # sghGC.pinUpdate(motorList[listLoop][2],0)
                                # sghGC.pinUpdate(motorList[listLoop][1],(svalue),"pwmmotor")
                                # else:
                                # sghGC.pinUpdate(motorList[listLoop][1],0)
                                # sghGC.pinUpdate(motorList[listLoop][2],0)

                        if (pcaPWM is not None):
                            ledList = [0, 3, 6, 9, 12]
                            # go thru PowerPWM on PCA Board
                            for i in range(0, 5):
                                if self.vFindValue('blue'):
                                    svalue = int(
                                        self.valueNumeric) if self.valueIsNumeric else 0
                                    svalue = min(
                                        4095, max((((100 - svalue) * 4096) / 100), 0))
                                    pcaPWM.setPWM((i * 3), 0, svalue)
                                if self.vFindValue('green'):
                                    svalue = int(
                                        self.valueNumeric) if self.valueIsNumeric else 0
                                    svalue = min(
                                        4095, max((((100 - svalue) * 4096) / 100), 0))
                                    pcaPWM.setPWM((i * 3) + 1, 0, svalue)
                                if self.vFindValue('red'):
                                    svalue = int(
                                        self.valueNumeric) if self.valueIsNumeric else 0
                                    svalue = min(
                                        4095, max((((100 - svalue) * 4096) / 100), 0))

                                    pcaPWM.setPWM((i * 3) + 2, 0, svalue)
                    elif "pi2golite" in ADDON:
                        #logging.debug("Processing variables for pi2golite")

                        # check for motor variable commands
                        motorList = [
                            ['motorb', 19, 21, 0, False], ['motora', 26, 24, 0, False]]
                        #logging.debug("ADDON:%s", ADDON)

                        for listLoop in range(0, 2):
                            if self.vFindValue(motorList[listLoop][0]):
                                svalue = min(
                                    100, max(-100, int(self.valueNumeric))) if self.valueIsNumeric else 0
                                logging.debug(
                                    "motor:%s valuee:%s", motorList[listLoop][0], svalue)
                                sghGC.motorUpdate(
                                    motorList[listLoop][1], motorList[listLoop][2], svalue)

                        moveServos = False

                        if self.vFindValue('tiltoffset'):
                            tiltoffset = int(
                                self.valueNumeric) if self.valueIsNumeric else 0
                            moveServos = True

                        if self.vFindValue('panoffset'):
                            panoffset = int(
                                self.valueNumeric) if self.valueIsNumeric else 0
                            moveServos = True

                        if self.vFindValue('tilt'):
                            # print "tilt command rcvd"
                            if self.valueIsNumeric:
                                tilt = int(self.valueNumeric)
                                moveServos = True
                                # print "tilt=", tilt
                            elif self.value == "off":
                                os.system(
                                    "echo " + "0" + "=0 > /dev/servoblaster")
                        else:
                            if self.vFindValue('servo18'):
                                # print "tilt command rcvd"
                                if self.valueIsNumeric:
                                    tilt = int(self.valueNumeric)
                                    moveServos = True
                                    # print "tilt=", tilt
                                elif self.value == "off":
                                    os.system(
                                        "echo " + "0" + "=0 > /dev/servoblaster")

                        if self.vFindValue('pan'):
                            # print "pan command rcvd"
                            if self.valueIsNumeric:
                                pan = int(self.valueNumeric)
                                moveServos = True
                                # print "pan=", pan
                            elif self.value == "off":
                                os.system(
                                    "echo " + "1" + "=0 > /dev/servoblaster")
                        else:
                            if self.vFindValue('servo22'):
                                # print "pan command rcvd"
                                if self.valueIsNumeric:
                                    pan = int(self.valueNumeric)
                                    moveServos = True
                                    # print "pan=", pan
                                elif self.value == "off":
                                    os.system(
                                        "echo " + "1" + "=0 > /dev/servoblaster")

                        if moveServos:
                            degrees = int(tilt + tiltoffset)
                            degrees = min(80, max(degrees, -60))
                            servodvalue = 50 + ((90 - degrees) * 200 / 180)
                            # print "sending", servodvalue, "to servod"
                            #os.system("echo " + "0" + "=" + str(servodvalue-1) + " > /dev/servoblaster")
                            sghGC.pinServod(18, servodvalue)  # orig =18
                            #os.system("echo " + "0" + "=" + str(servodvalue) + " > /dev/servoblaster")
                            degrees = int(pan + panoffset)
                            degrees = min(90, max(degrees, -90))
                            servodvalue = 50 + ((90 - degrees) * 200 / 180)
                            sghGC.pinServod(22, servodvalue)  # orig =22
                            #os.system("echo " + "1" + "=" + str(servodvalue) + " > /dev/servoblaster")

                            # End of Pi2gplite Variable handling
                    elif "pi2go" in ADDON:
                        # do PiRoCon stuff
                        #logging.debug("Processing variables for Pi2Go")

                        # check for motor variable commands
                        motorList = [
                            ['motorb', 19, 21, 0, False], ['motora', 26, 24, 0, False]]
                        #logging.debug("ADDON:%s", ADDON)

                        for listLoop in range(0, 2):
                            if self.vFindValue(motorList[listLoop][0]):
                                svalue = min(
                                    100, max(-100, int(self.valueNumeric))) if self.valueIsNumeric else 0
                                logging.debug(
                                    "motor:%s valuee:%s", motorList[listLoop][0], svalue)
                                sghGC.motorUpdate(
                                    motorList[listLoop][1], motorList[listLoop][2], svalue)

                        if (pcaPWM is not None):
                            ledList = [0, 3, 6, 9, 12]
                            # go thru PowerPWM on PCA Board
                            for i in range(0, 5):
                                if self.vFindValue('blue'):
                                    svalue = int(
                                        self.valueNumeric) if self.valueIsNumeric else 0
                                    svalue = min(
                                        4095, max((((svalue) * 4096) / 100), 0))
                                    pcaPWM.setPWM((i * 3), 0, svalue)
                                if self.vFindValue('green'):
                                    svalue = int(
                                        self.valueNumeric) if self.valueIsNumeric else 0
                                    svalue = min(
                                        4095, max((((svalue) * 4096) / 100), 0))
                                    pcaPWM.setPWM((i * 3) + 1, 0, svalue)
                                if self.vFindValue('red'):
                                    svalue = int(
                                        self.valueNumeric) if self.valueIsNumeric else 0
                                    svalue = min(
                                        4095, max((((svalue) * 4096) / 100), 0))

                                    pcaPWM.setPWM((i * 3) + 2, 0, svalue)

                            # go thru servos on PCA Board
                            for i in range(12, 16):
                                if self.vFindValue('servo' + str(i)):
                                    svalue = int(
                                        self.valueNumeric) if self.valueIsNumeric else 0
                                    # print i, svalue
                                    pcaPWM.setPWM(
                                        i, 0, int(min(780, max(120, 450 - (svalue * 3.33333)))))

                            if self.vFindValue('pan'):
                                i = 12
                                svalue = int(
                                    self.valueNumeric) if self.valueIsNumeric else 0
                                # print i, svalue
                                pcaPWM.setPWM(
                                    i, 0, int(min(780, max(120, 450 - (svalue * 3.33333)))))

                            if self.vFindValue('tilt'):
                                i = 13
                                svalue = int(
                                    self.valueNumeric) if self.valueIsNumeric else 0
                                # print i, svalue
                                pcaPWM.setPWM(
                                    i, 0, int(min(780, max(120, 450 - (svalue * 3.33333)))))

                    elif "apb01" in ADDON:
                        #logging.debug("Processing variables for apb01")

                        # check for motor variable commands
                        motorList = [
                            ['motorb', 21, 19, 0, False], ['motora', 24, 26, 0, False]]
                        #logging.debug("ADDON:%s", ADDON)

                        for listLoop in range(0, 2):
                            if self.vFindValue(motorList[listLoop][0]):
                                svalue = min(
                                    100, max(-100, int(self.valueNumeric))) if self.valueIsNumeric else 0
                                logging.debug(
                                    "motor:%s valuee:%s", motorList[listLoop][0], svalue)
                                sghGC.motorUpdate(
                                    motorList[listLoop][1], motorList[listLoop][2], svalue)

                    elif "agobo" in ADDON:
                        #logging.debug("Processing variables for apb01")

                        # check for motor variable commands
                        motorList = [
                            ['motorb', 19, 21, 0, False], ['motora', 26, 24, 0, False]]
                        #logging.debug("ADDON:%s", ADDON)

                        for listLoop in range(0, 2):
                            if self.vFindValue(motorList[listLoop][0]):
                                svalue = min(
                                    100, max(-100, int(self.valueNumeric))) if self.valueIsNumeric else 0
                                logging.debug(
                                    "motor:%s valuee:%s", motorList[listLoop][0], svalue)
                                sghGC.motorUpdate(
                                    motorList[listLoop][1], motorList[listLoop][2], svalue)

                                # End of agobo variable handling
                    elif "piringo" in ADDON:
                        # do piringo stuff

                        # check All LEDS On/Off/High/Low/1/0
                        self.vAllCheck("leds")

                        self.vLEDCheck(piringoOutputs)

                    elif "pibrella" in ADDON:  # PiBrella

                        # check All On/Off/High/Low/1/0
                        self.vAllCheck("allpins")

                        self.vListCheck([13, 11, 7, 15, 16, 18, 22],
                                        ["led1", "led2", "led3", "led4", "led5", "led6", "led7"])
                        self.vListCheck([13, 11, 11, 11, 7, 15, 16, 18, 22],
                                        ["red", "amber", "yellow", "orange", "green", "outpute", "outputf", "outputg",
                                         "outputh"])
                        self.vListCheckMotorOnly(
                            [15, 16, 18, 22], ["e", "f", "g", "h"])

                        if self.vFindValue('stepper'):
                            if self.valueIsNumeric:
                                self.stepperUpdate(
                                    [15, 16, 18, 22], self.valueNumeric)
                            else:
                                self.stepperUpdate([15, 16, 18, 22], 0)

                        if self.vFindValue("beep"):
                            try:
                                bn, bd = self.value.split(",")
                            except:
                                bn = "60"
                                bd = "1"
                            beepNote = int(float(bn))
                            beepDuration = (float(bd))
                            svalue = int(
                                self.valueNumeric) if self.valueIsNumeric else 60
                            beepThread = threading.Thread(target=self.beep,
                                                          args=[12, 440 * 2 ** ((beepNote - 69) / 12.0), beepDuration])
                            beepThread.start()

                            # if self.vFindValue("beepnote"):
                            # beepNote = max(12,int(self.valueNumeric)) if self.valueIsNumeric else 60

                            # if self.vFindValue("beepduration"):
                            # beepDuration = max(0.125,int(self.valueNumeric)) if self.valueIsNumeric else 0.5

                    elif "happi" in ADDON:
                        # do happi stuff
                        logging.debug("Processing variables for HapPi")

                        # check for motor variable commands
                        self.vListHBridge2(
                            [['motor1', 11, 12], ['motor2', 15, 16]])

                    elif "raspibot2" in ADDON:
                        logging.debug("Processing variables for RasPiBot2")
                        # check All LEDS On/Off/High/Low/1/0
                        self.vAllCheck("leds")
                        self.vListCheck([26, 24, 15, 13],
                                        ["led1", "led2", "output1", "output2"])  # Check for LED off/on type broadcasts

                        # check for motor variable commands
                        motorList = [['motorl', 19, 22], ['motorr', 11, 7]]
                        logging.debug("ADDON:%s", ADDON)
                        for listLoop in range(0, 2):
                            if self.vFindValue(motorList[listLoop][0]):
                                svalue = int(
                                    self.valueNumeric) if self.valueIsNumeric else 0
                                logging.debug(
                                    "svalue %s %s", motorList[listLoop][0], svalue)
                                if svalue > 0:
                                    sghGC.pinUpdate(motorList[listLoop][2], 1)
                                    sghGC.pinUpdate(
                                        motorList[listLoop][1], (svalue), "pwmmotor")
                                elif svalue < 0:
                                    sghGC.pinUpdate(motorList[listLoop][2], 0)
                                    sghGC.pinUpdate(
                                        motorList[listLoop][1], (svalue), "pwmmotor")
                                else:
                                    sghGC.pinUpdate(motorList[listLoop][1], 0)
                                    sghGC.pinUpdate(motorList[listLoop][2], 0)

                    elif "pizazz" in ADDON:

                        logging.debug("Processing variables for Pizazz")

                        # Check for LEDs
                        self.vListCheck(
                            [22, 18, 11, 7], ["led1", "led2", "led3", "led4"])

                        # check for motor variable commands
                        motorList = [
                            ['motorr', 19, 21, 0], ['motorl', 24, 26, 0]]
                        #logging.debug("ADDON:%s", ADDON)

                        for listLoop in range(0, 2):
                            if self.vFindValue(motorList[listLoop][0]):
                                svalue = min(
                                    100, max(-100, int(self.valueNumeric))) if self.valueIsNumeric else 0
                                logging.debug(
                                    "motor:%s valuee:%s", motorList[listLoop][0], svalue)
                                sghGC.motorUpdate(
                                    motorList[listLoop][1], motorList[listLoop][2], svalue)

                    elif "simpie" in ADDON:
                        # do BerryClip stuff
                        # check All LEDS On/Off/High/Low/1/0
                        self.vAllCheck("all")
                        # Check for LEDs
                        self.vListCheck([12, 16, 18], ["red", "green", "blue"])

                        if self.vFindOnOff('buzzer'):
                            self.index_pin_update(7, 100 - self.valueNumeric)

                    elif "techtom" in ADDON:
                        # do ladderboard stuff

                        # check All LEDS On/Off/High/Low/1/0
                        self.vAllCheck("leds")

                        self.vLEDCheck(ladderOutputs)

                    elif "ledborg" in ADDON:
                        # check All LEDS On/Off/High/Low/1/0
                        self.vAllCheck("all")
                        # Check for LEDs
                        self.vListCheck([11, 13, 15], ["red", "green", "blue"])

                        # End of BerryClip Variable handling
                    elif "explorer" in ADDON:

                        motorList = [
                            ['motor1', 38, 35, 0, False], ['motor2', 40, 37, 0, False]]
                        #logging.debug("ADDON:%s", ADDON)

                        for listLoop in range(0, 2):
                            if self.vFindValue(motorList[listLoop][0]):
                                svalue = min(
                                    100, max(-100, int(self.valueNumeric))) if self.valueIsNumeric else 0
                                logging.debug(
                                    "motor:%s valuee:%s", motorList[listLoop][0], svalue)
                                sghGC.motorUpdate(
                                    motorList[listLoop][1], motorList[listLoop][2], svalue)

                    else:  # normal variable processing with no add on board

                        # check All On/Off/High/Low/1/0
                        self.vAllCheck("allpins")

                        # check for any pin On/Off/High/Low/1/0 any PWM
                        # settings using power or motor
                        self.vPinCheck()
                        #logging.debug("Steppers in use")
                        if steppersInUse:
                            logging.debug("Steppers in use")
                            stepperList = [
                                ['motora', [11, 12, 13, 15]], ['motorb', [16, 18, 22, 7]]]
                            for listLoop in range(0, 2):
                                if self.vFindValue(stepperList[listLoop][0]):
                                    logging.debug(
                                        "Stepper found %s", stepperList[listLoop][0])
                                    if self.valueIsNumeric:
                                        self.stepperUpdate(
                                            stepperList[listLoop][1], self.valueNumeric)
                                    else:
                                        self.stepperUpdate(
                                            stepperList[listLoop][1], 0)

                            stepperList = [
                                ['positiona', [11, 12, 13, 15]], ['positionb', [16, 18, 22, 7]]]
                            for listLoop in range(0, 2):
                                #print ("look for steppers")
                                if self.vFindValue(stepperList[listLoop][0]):
                                    print (
                                        "Found stepper", stepperList[listLoop][0])
                                    if self.valueIsNumeric:
                                        print ("value =", self.value)
                                        print stepperList[listLoop][1][0]
                                        try:
                                            print (
                                                "Trying to see if turn prev set")
                                            direction = int(
                                                100 * sign(int(self.valueNumeric) - turn[stepperList[listLoop][1][0]]))
                                            steps = abs(
                                                int(self.valueNumeric) - turn[stepperList[listLoop][1][0]])
                                        except:
                                            direction = int(
                                                100 * sign(int(self.valueNumeric)))
                                            steps = abs(int(self.valueNumeric))
                                            turn = [None] * sghGC.numOfPins
                                            pass
                                        print (
                                            "direction and steps", direction, steps)
                                        self.stepperUpdate(
                                            stepperList[listLoop][1], direction, steps)
                                        turn[
                                            stepperList[listLoop][1][0]] = self.valueNumeric
                                        print (
                                            "position set to :", turn[stepperList[listLoop][1][0]])
                                    else:
                                        self.stepperUpdate(
                                            stepperList[listLoop][1], 0)
                                        try:
                                            turn[
                                                stepperList[listLoop][1][0]] = 0
                                        except:
                                            turn = [None] * sghGC.numOfPins
                                            turn[
                                                stepperList[listLoop][1][0]] = 0
                                            pass
                        else:
                            motorList = [['motora', 11], ['motorb', 12]]
                            for listLoop in range(0, 2):
                                if self.vFindValue(motorList[listLoop][0]):
                                    if self.valueIsNumeric:
                                        sghGC.pinUpdate(
                                            motorList[listLoop][1], self.valueNumeric, type="pwmmotor")
                                    else:
                                        sghGC.pinUpdate(
                                            motorList[listLoop][1], 0, type="pwmmotor")

                           # motorList = [['motor21,19', 21, 19, 0], ['motor26,24', 26, 24, 0]]
                            print self.dataraw
                            motorhList = [
                                m.start() for m in re.finditer('motorh', self.dataraw)]
                            print motorhList
                            motorList = []
                            for loop in motorhList:
                                name = self.dataraw[loop:]
                                name = name[0:name.find(' ')]
                                print "name", name
                                pin1 = name[6:name.find(',')]
                                print "pin1", pin1
                                pin2 = name[1 + name.find(','):]
                                print "pin2", pin2
                                motorList.append(
                                    [name, int(pin1), int(pin2), 0])
                            print motorList
                            for listLoop in motorList:
                                print "listloop:", listLoop
                                if self.vFindValue(listLoop[0]):
                                    svalue = min(
                                        100, max(-100, int(self.valueNumeric))) if self.valueIsNumeric else 0
                                    logging.debug(
                                        "motor:%s valuee:%s", listLoop[0], svalue)
                                    sghGC.motorUpdate(
                                        listLoop[1], listLoop[2], svalue)
                        # end of motor checking

                        if self.bFindValue('servo'):
                            print "servo"
                            for pin in sghGC.validPins:
                                if self.vFindValue('servo' + str(pin)):
                                    svalue = int(
                                        self.valueNumeric) if self.valueIsNumeric else -150
                                    svalue = (svalue + 150)
                                    sghGC.pinServod(pin, svalue)

                    # Use bit pattern to control ports
                    if self.vFindValue('pinpattern'):
                        if "pitt" in ADDON:
                            bit_pattern = ('00000000' + self.value)[-8:]
                            print 'bit_pattern %s' % bit_pattern
                            j = 0
                            for pin in [8, 10, 12, 16, 18, 22, 24, 26]:
                                # print "pin" , bit_pattern[-(j+1)]
                                if bit_pattern[-(j + 1)] == '0':
                                    sghGC.pinUpdate(pin, 0)
                                else:
                                    sghGC.pinUpdate(pin, 1)
                                j += 1
                        else:
                            svalue = self.value
                            bit_pattern = (
                                '0000000000000000000000000000000000000000' + svalue)[-sghGC.numOfPins:]
                            j = 0
                            # onSense = '1' if sghGC.INVERT else '0' # change
                            # to look for 0 if invert on
                            onSense = '0'
                            for pin in sghGC.validPins:
                                if (sghGC.pinUse[pin] == sghGC.POUTPUT):
                                    # print "pin" , bit_pattern[-(j+1)]
                                    if bit_pattern[-(j + 1)] == onSense:
                                        sghGC.pinUpdate(pin, 0)
                                    else:
                                        sghGC.pinUpdate(pin, 1)
                                    j += 1

                    checkStr = 'stepdelay'
                    if (checkStr + ' ') in dataraw:
                        # print "MotorA Received"
                        # print "stepper status" , stepperInUse[STEPPERA]
                        tempValue = getValue(checkStr, dataraw)
                        if isNumeric(tempValue):
                            step_delay = int(float(tempValue))
                            print 'step delay changed to', step_delay

                    if pcfSensor is not None:  # if PCF ADC found
                        if self.vFindValue('dac'):
                            svalue = int(
                                self.valueNumeric) if self.valueIsNumeric else 0
                            pcfSensor.writeDAC(max(0, min(255, svalue)))

                    if pcaPWM is not None:
                        for i in range(0, 16):  # go thru servos on PCA Board
                            if self.vFindValue('adaservo' + str(i)):
                                if self.value != "off":
                                    svalue = int(
                                        self.valueNumeric) if self.valueIsNumeric else 0
                                    # print i, svalue
                                    svalue = int(
                                        min(720, max(120, 420 - (svalue * 3.33333))))
                                    pcaPWM.setPWM(i, 0, svalue)
                                    print "pwm sent", i, svalue
                                else:
                                    pcaPWM.setPWM(i, 0, 0)
                                    print "servo", i, "switched off"
                        for i in range(0, 16):  # go thru PowerPWM on PCA Board
                            if self.vFindValue('adapower' + str(i + 1)):
                                svalue = int(
                                    self.valueNumeric) if self.valueIsNumeric else 0
                                svalue = min(
                                    4095, max(((svalue * 4096) / 100), 0))
                                pcaPWM.setPWM(i, 0, svalue)
                                print svalue

                        meArmAction = False
                        oldmeH = meHorizontal
                        oldmeD = meDistance
                        oldmeV = meVertical
                        if self.vFindValue('mehorizontal'):
                            meHorizontal = max(
                                -50, min(50, int(self.valueNumeric))) if self.valueIsNumeric else 0
                            meArmAction = True

                        if self.vFindValue('medistance'):
                            meDistance = max(
                                70, min(150, int(self.valueNumeric))) if self.valueIsNumeric else 100
                            meArmAction = True

                        if self.vFindValue('mevertical'):
                            meVertical = max(-0, min(60, int(self.valueNumeric))
                                             ) if self.valueIsNumeric else 50
                            meArmAction = True

                        if self.vFindValue('megripper'):
                            if self.value == "close":
                                self.arm.closeGripper()
                                print "gripper closed"
                            else:
                                self.arm.openGripper()
                                print "Gripper opened"

                        if meArmAction:
                            s = 5
                            deltaH = (meHorizontal - oldmeH) / s
                            deltaD = (meDistance - oldmeD) / s
                            deltaV = (meVertical - oldmeV) / s
                            for loop in range(s):
                                oldmeH += deltaH
                                oldmeD += deltaD
                                oldmeV += deltaV
                                self.meArmGotoPoint(oldmeH, oldmeD, oldmeV)
                                time.sleep(0.1)

                            time.sleep(0.1)
                            self.meArmGotoPoint(
                                meHorizontal, meDistance, meVertical)

                    if self.vFindValue("minex"):
                        print "minex"
                        sghMC.setxPos(int(self.value))

                    if self.vFindValue("miney"):
                        print "miney"
                        sghMC.setyPos(int(self.value))

                    if self.vFindValue("minez"):
                        print "minez"
                        sghMC.setzPos(int(self.value))

                    if self.vFindValue('ultradelay'):
                        sghGC.ultraFreq = self.valueNumeric if self.valueIsNumeric else 1

                    if ((piglow is not None) and ("piglow" not in ADDON)):
                        # do PiGlow stuff but make sure PiGlow physically
                        # detected

                        # check LEDS
                        for i in range(1, 19):
                            if self.vFindValue('pgled' + str(i)):
                                svalue = int(
                                    self.valueNumeric) if self.valueIsNumeric else 0
                                svalue = min(255, max(svalue, 0))
                                PiGlow_Values[PiGlow_Lookup[i - 1]] = svalue
                                piglow.update_pwm_values(PiGlow_Values)

                        for i in range(1, 4):
                            if self.vFindValue('pgleg' + str(i)):
                                svalue = int(
                                    self.valueNumeric) if self.valueIsNumeric else 0
                                svalue = min(255, max(svalue, 0))
                                PiGlow_Values[
                                    PiGlow_Lookup[((i - 1) * 6) + 0]] = svalue
                                PiGlow_Values[
                                    PiGlow_Lookup[((i - 1) * 6) + 1]] = svalue
                                PiGlow_Values[
                                    PiGlow_Lookup[((i - 1) * 6) + 2]] = svalue
                                PiGlow_Values[
                                    PiGlow_Lookup[((i - 1) * 6) + 3]] = svalue
                                PiGlow_Values[
                                    PiGlow_Lookup[((i - 1) * 6) + 4]] = svalue
                                PiGlow_Values[
                                    PiGlow_Lookup[((i - 1) * 6) + 5]] = svalue
                                piglow.update_pwm_values(PiGlow_Values)

                            if self.vFindValue('pgarm' + str(i)):
                                svalue = int(
                                    self.valueNumeric) if self.valueIsNumeric else 0
                                svalue = min(255, max(svalue, 0))
                                PiGlow_Values[
                                    PiGlow_Lookup[((i - 1) * 6) + 0]] = svalue
                                PiGlow_Values[
                                    PiGlow_Lookup[((i - 1) * 6) + 1]] = svalue
                                PiGlow_Values[
                                    PiGlow_Lookup[((i - 1) * 6) + 2]] = svalue
                                PiGlow_Values[
                                    PiGlow_Lookup[((i - 1) * 6) + 3]] = svalue
                                PiGlow_Values[
                                    PiGlow_Lookup[((i - 1) * 6) + 4]] = svalue
                                PiGlow_Values[
                                    PiGlow_Lookup[((i - 1) * 6) + 5]] = svalue
                                piglow.update_pwm_values(PiGlow_Values)

                        pcolours = [
                            'pgred', 'pgorange', 'pgyellow', 'pggreen', 'pgblue', 'pgwhite']
                        for i in range(len(pcolours)):
                            if self.vFindValue(pcolours[i]):
                                svalue = int(
                                    self.valueNumeric) if self.valueIsNumeric else 0
                                svalue = min(255, max(svalue, 0))
                                PiGlow_Values[PiGlow_Lookup[i + 0]] = svalue
                                PiGlow_Values[PiGlow_Lookup[i + 6]] = svalue
                                PiGlow_Values[PiGlow_Lookup[i + 12]] = svalue
                                piglow.update_pwm_values(PiGlow_Values)

                        # Use bit pattern to control leds
                        if self.vFindValue('pgledpattern'):
                            # print 'Found ledpattern'
                            num_of_bits = 18
                            bit_pattern = (
                                '00000000000000000000000000' + self.value)[-num_of_bits:]
                            # print 'led_pattern %s' % bit_pattern
                            j = 0
                            for i in range(18):
                                #bit_state = ((2**i) & sensor_value) >> i
                                # print 'dummy pin %d state %d' % (i,
                                # bit_state)
                                if bit_pattern[-(j + 1)] == '0':
                                    PiGlow_Values[PiGlow_Lookup[i]] = 0
                                else:
                                    PiGlow_Values[PiGlow_Lookup[i]] = 1
                                j += 1

                            piglow.update_pwm_values(PiGlow_Values)

                    if "mearm" in ADDON:

                        if (pcaPWM is not None):

                            # go thru servos on PCA Board
                            for i in range(0, 16):
                                if self.vFindValue('servo' + str(i)):
                                    svalue = int(
                                        self.valueNumeric) if self.valueIsNumeric else 0
                                    # print i, svalue
                                    pcaPWM.setPWM(
                                        i, 0, int(min(780, max(120, 450 - (svalue * 3.33333)))))

                                    # Check for Broadcast type messages being received
                # print "loggin level",debugLogging
                if (debugLogging == False):
                    logging.getLogger().setLevel(logging.INFO)

                if 'broadcast' in self.dataraw:
                    # print 'broadcast:' , self.dataraw

                    if self.bFindValue("qmsg"):
                        msgQueue.put((5, self.value))
                        # print "queue len", len(msgQueue)

                    # if self.bFindValue("hardrestart"):
                    #    os.execv(__file__, sys.argv)

                    if self.bFindValue("setpins"):
                        logging.debug("SetPins broadcast found")
                        logging.debug("SetPins value len %d", len(self.value))
                        logging.debug("SetPins value %s", self.value)

                        if len(self.value) == 0:
                            with lock:

                                for pin in sghGC.validPins:
                                    sghGC.pinUse[pin] = sghGC.PINPUTDOWN
                                sghGC.pinUse[11] = sghGC.POUTPUT
                                sghGC.pinUse[12] = sghGC.POUTPUT
                                sghGC.pinUse[13] = sghGC.POUTPUT
                                sghGC.pinUse[15] = sghGC.POUTPUT
                                sghGC.pinUse[16] = sghGC.POUTPUT
                                sghGC.pinUse[18] = sghGC.POUTPUT
                                #sghGC.pinUse[22] = sghGC.PINPUT
                                #sghGC.pinUse[7] = sghGC.PINPUT
                                sghGC.setPinMode()
                                anyAddOns = True

                        elif self.value == "low":
                            with lock:
                                print "set pins to input with pulldown low"
                                for pin in sghGC.validPins:
                                    sghGC.pinUse[pin] = sghGC.PINPUTDOWN
                                #sghGC.pinUse[3] = sghGC.PUNUSED
                                #sghGC.pinUse[5] = sghGC.PUNUSED
                                sghGC.setPinMode()
                                anyAddOns = True

                        elif self.value == "high":
                            with lock:
                                print "set pins to input with pull ups"
                                for pin in sghGC.validPins:
                                    sghGC.pinUse[pin] = sghGC.PINPUT
                                #sghGC.pinUse[3] = sghGC.PUNUSED
                                #sghGC.pinUse[5] = sghGC.PUNUSED
                                sghGC.setPinMode()
                                anyAddOns = True

                        elif self.value == "none":
                            with lock:
                                print "set pins to input with no pullups"
                                for pin in sghGC.validPins:
                                    sghGC.pinUse[pin] = sghGC.PINPUTNONE
                                #sghGC.pinUse[3] = sghGC.PUNUSED
                                #sghGC.pinUse[5] = sghGC.PUNUSED
                                sghGC.setPinMode()
                                anyAddOns = True

                    if self.bFindOnOff("sghdebug"):
                        if (self.OnOrOff == True) and (debugLogging == False):
                            logging.getLogger().setLevel(logging.DEBUG)
                            debugLogging = True
                        if (self.OnOrOff == False) and (debugLogging == True):
                            logging.getLogger().setLevel(logging.INFO)
                            debugLogging = False

                    if (debugLogging == False):
                        logging.getLogger().setLevel(logging.INFO)

                    if self.bFindOnOff("eventdetect"):
                        sghGC.pinEventEnabled = self.OnOrOff
                        print "pinEvent Detect: ", sghGC.pinEventEnabled

                    if self.bFindValue("bright"):
                        sghGC.ledDim = int(
                            self.valueNumeric) if self.valueIsNumeric else 100
                        PiGlow_Brightness = sghGC.ledDim
                        print sghGC.ledDim

                    if self.bFindValue("triggerreset"):
                        print "triggerreset detected"
                        # print len(self.value) , ("["+self.value+"]")
                        if self.value == "":
                            print "reset all triggers"
                            sghGC.anyTrigger = 0
                            for pin in sghGC.validPins:
                                sghGC.pinTrigger[pin] = 0
                                # print "delaying 5 secs"
                                # time.sleep(5)
                        else:
                            for pin in sghGC.validPins:
                                if sghGC.pinTriggerName[pin] == self.value:
                                    print "trigger reset found", self.value
                                    sghGC.pinTrigger[pin] = 0
                                    sghGC.anyTrigger = 0

                    if self.bFindValue("touchreset"):
                        sghCT = sghGC.capTouchHelper
                        # print "touchreset detected", sghCT
                        # print len(self.value) , ("["+self.value+"]")
                        if self.value == "":
                            for loop in range(0, 8):
                                # print "loop",sghCT.ctTrigStatus[loop]
                                sghCT.ctTrigStatus[loop][1] = 0
                                sensor_name = "touch" + \
                                    str(sghCT.ctTrigStatus[loop][0])
                                sensor_value = "0"
                                # print sensor_name,sensor_value
                                bcast_str = 'sensor-update "%s" %s' % (
                                    sensor_name, sensor_value)
                                msgQueue.put((5, bcast_str))
                            sghCT.ctTrigStatus[8][1] = 0
                        else:
                            for loop in range(0, 8):
                                if sghCT.ctTrigStatus[loop][0] == int(self.valueNumeric):
                                    sghCT.ctTrigStatus[loop][1] = 0
                                    sensor_name = "touch" + \
                                        str(sghCT.ctTrigStatus[loop][0])
                                    sensor_value = "0"
                                    # print sensor_name,sensor_value
                                    bcast_str = 'sensor-update "%s" %s' % (
                                        sensor_name, sensor_value)
                                    msgQueue.put((5, bcast_str))

                    #msgQueue.put((5,"broadcast Begin"))
                    if self.bFind("stepper"):
                        print ("Stepper declared")
                        steppersInUse = True
                        sghGC.pinUse[11] = sghGC.POUTPUT
                        sghGC.pinUse[12] = sghGC.POUTPUT
                        sghGC.pinUse[13] = sghGC.POUTPUT
                        sghGC.pinUse[15] = sghGC.POUTPUT
                        sghGC.pinUse[16] = sghGC.POUTPUT
                        sghGC.pinUse[18] = sghGC.POUTPUT
                        sghGC.pinUse[22] = sghGC.POUTPUT
                        sghGC.pinUse[7] = sghGC.POUTPUT
                        sghGC.setPinMode()

                    if "ladder" in ADDON:  # Gordon's Ladder Board
                        # do ladderboard stuff
                        #print ("Ladder broadcast processing")
                        # Check for all off/on type broadcasrs
                        self.bCheckAll()
                        # Check for LED off/on type broadcasts
                        self.bLEDCheck(ladderOutputs)

                    elif "motorpitx" in ADDON:  # Boeeerb MotorPiTx
                        self.bCheckAll()
                        self.bListCheck(
                            [15, 11, 13, 7], ["output1", "output2", "input1", "input2"])
                        if ('sonar1') in dataraw:
                            distance = sghGC.pinSonar(13)
                            # print'Distance:',distance,'cm'
                            sensor_name = 'sonar' + str(13)
                            bcast_str = 'sensor-update "%s" %d' % (
                                sensor_name, distance)
                            # print 'sending: %s' % bcast_str
                            msgQueue.put((5, bcast_str))

                        if ('sonar2') in dataraw:
                            distance = sghGC.pinSonar(7)
                            # print'Distance:',distance,'cm'
                            sensor_name = 'sonar' + str(7)
                            bcast_str = 'sensor-update "%s" %d' % (
                                sensor_name, distance)
                            # print 'sending: %s' % bcast_str
                            msgQueue.put((5, bcast_str))

                        if self.bFind('ultra1'):
                            print 'start pinging on', str(13)
                            self.startUltra(13, 0, self.OnOrOff)

                        if self.bFind('ultra2'):
                            print 'start pinging on', str(7)
                            self.startUltra(7, 0, self.OnOrOff)

                    elif ((piglow is not None) and ("piglow" in ADDON)):
                        # print "processing piglow variables"

                        if self.bFindOnOff('all'):
                            # print "found allon/off"
                            for i in range(1, 19):
                                # print i
                                PiGlow_Values[
                                    i - 1] = PiGlow_Brightness * self.OnOrOff
                                # print "Values", PiGlow_Values
                                piglow.update_pwm_values(PiGlow_Values)

                        # check LEDS
                        for i in range(1, 19):
                            #check_broadcast = str(i) + 'on'
                            # print check_broadcast
                            if self.bFindOnOff('led' + str(i)):
                                # print dataraw
                                PiGlow_Values[
                                    PiGlow_Lookup[i - 1]] = PiGlow_Brightness * self.OnOrOff
                                piglow.update_pwm_values(PiGlow_Values)

                            if self.bFindOnOff('light' + str(i)):
                                # print dataraw
                                PiGlow_Values[
                                    PiGlow_Lookup[i - 1]] = PiGlow_Brightness * self.OnOrOff
                                piglow.update_pwm_values(PiGlow_Values)

                        pcolours = [
                            'red', 'orange', 'yellow', 'green', 'blue', 'white']
                        for i in range(len(pcolours)):
                            if self.bFindOnOff(pcolours[i]):
                                # print dataraw
                                PiGlow_Values[
                                    PiGlow_Lookup[i + 0]] = PiGlow_Brightness * self.OnOrOff
                                PiGlow_Values[
                                    PiGlow_Lookup[i + 6]] = PiGlow_Brightness * self.OnOrOff
                                PiGlow_Values[
                                    PiGlow_Lookup[i + 12]] = PiGlow_Brightness * self.OnOrOff
                                piglow.update_pwm_values(PiGlow_Values)

                        for i in range(1, 4):
                            if self.bFindOnOff('leg' + str(i)):
                                # print dataraw
                                PiGlow_Values[
                                    PiGlow_Lookup[((i - 1) * 6) + 0]] = PiGlow_Brightness * self.OnOrOff
                                PiGlow_Values[
                                    PiGlow_Lookup[((i - 1) * 6) + 1]] = PiGlow_Brightness * self.OnOrOff
                                PiGlow_Values[
                                    PiGlow_Lookup[((i - 1) * 6) + 2]] = PiGlow_Brightness * self.OnOrOff
                                PiGlow_Values[
                                    PiGlow_Lookup[((i - 1) * 6) + 3]] = PiGlow_Brightness * self.OnOrOff
                                PiGlow_Values[
                                    PiGlow_Lookup[((i - 1) * 6) + 4]] = PiGlow_Brightness * self.OnOrOff
                                PiGlow_Values[
                                    PiGlow_Lookup[((i - 1) * 6) + 5]] = PiGlow_Brightness * self.OnOrOff
                                piglow.update_pwm_values(PiGlow_Values)

                            if self.bFindOnOff('arm' + str(i)):
                                # print dataraw
                                PiGlow_Values[
                                    PiGlow_Lookup[((i - 1) * 6) + 0]] = PiGlow_Brightness * self.OnOrOff
                                PiGlow_Values[
                                    PiGlow_Lookup[((i - 1) * 6) + 1]] = PiGlow_Brightness * self.OnOrOff
                                PiGlow_Values[
                                    PiGlow_Lookup[((i - 1) * 6) + 2]] = PiGlow_Brightness * self.OnOrOff
                                PiGlow_Values[
                                    PiGlow_Lookup[((i - 1) * 6) + 3]] = PiGlow_Brightness * self.OnOrOff
                                PiGlow_Values[
                                    PiGlow_Lookup[((i - 1) * 6) + 4]] = PiGlow_Brightness * self.OnOrOff
                                PiGlow_Values[
                                    PiGlow_Lookup[((i - 1) * 6) + 5]] = PiGlow_Brightness * self.OnOrOff
                                piglow.update_pwm_values(PiGlow_Values)

                    elif "gpio" in ADDON:  # gPiO
                        #print ("gPiO broadcast processing")
                        # Check for all off/on type broadcasts
                        self.bCheckAll()
                        # Check for pin off/on type broadcasts
                        self.bPinCheck(sghGC.validPins)

                    elif "berry" in ADDON:  # BerryClip

                        #print ("Berry broadcast processing")
                        # Check for all off/on type broadcasts
                        self.bCheckAll()
                        # Check for LED off/on type broadcasts
                        self.bLEDCheck(berryOutputs)
                        if self.bFindOnOff('buzzer'):
                            sghGC.pinUpdate(24, self.OnOrOff)

                    elif "pirocon" in ADDON:  # pirocon

                        # Check for all off/on type broadcasrs
                        self.bCheckAll()
                        # Check for pin off/on type broadcasts
                        self.bPinCheck(sghGC.validPins)

                        # check pins
                        for pin in sghGC.validPins:
                            if self.bFindOnOff('pin' + str(pin)):
                                sghGC.pinUpdate(pin, self.OnOrOff)

                            if self.bFind('sonar' + str(pin)):
                                distance = sghGC.pinSonar(pin)
                                # print'Distance:',distance,'cm'
                                sensor_name = 'sonar' + str(pin)
                                bcast_str = 'sensor-update "%s" %d' % (
                                    sensor_name, distance)
                                # print 'sending: %s' % bcast_str
                                msgQueue.put((5, bcast_str))

                            # Start using ultrasonic sensor on a pin
                            if self.bFind('ultra' + str(pin)):
                                self.startUltra(pin, 0, self.OnOrOff)

                        motorList = [
                            ['turnr', 21, 26, 7], ['turnl', 19, 24, 11]]
                        if "piroconb" in ADDON:
                            logging.debug("PiRoConB Found:%s", ADDON)
                            motorList = [
                                ['turnr', 21, 19, 7], ['turnl', 26, 24, 11]]

                        if self.bFindValue("move"):
                            svalue = int(
                                self.valueNumeric) if self.valueIsNumeric else 0

                            sghGC.countDirection[
                                motorList[0][3]] = -1 if svalue < 0 else 1
                            print "sghdir", sghGC.countDirection[motorList[0][3]]

                            turnDualThread = threading.Thread(target=self.stopTurning,
                                                              args=[motorList, svalue, sghGC.pinCount[motorList[0][3]]])
                            turnDualThread.start()
                            for listLoop in range(0, 2):
                                if svalue > 0:
                                    sghGC.pinUpdate(motorList[listLoop][2], 1)
                                    sghGC.pinUpdate(
                                        motorList[listLoop][1], (100 - self.turnSpeed), "pwmmotor")
                                elif svalue < 0:
                                    sghGC.pinUpdate(motorList[listLoop][2], 0)
                                    sghGC.pinUpdate(
                                        motorList[listLoop][1], (self.turnSpeed), "pwmmotor")

                        if self.bFindValue("turn"):
                            svalue = int(
                                self.valueNumeric) if self.valueIsNumeric else 0
                            turnDualThread = threading.Thread(
                                target=self.stopTurning, args=[motorList, svalue])
                            turnDualThread.start()
                            if svalue > 0:
                                sghGC.pinUpdate(motorList[0][2], 1)
                                sghGC.pinUpdate(
                                    motorList[0][1], (100 - self.turnSpeed), "pwmmotor")
                                sghGC.pinUpdate(motorList[1][2], 0)
                                sghGC.pinUpdate(
                                    motorList[1][1], (self.turnSpeed), "pwmmotor")
                            elif svalue < 0:
                                sghGC.pinUpdate(motorList[0][2], 0)
                                sghGC.pinUpdate(
                                    motorList[0][1], (self.turnSpeed), "pwmmotor")
                                sghGC.pinUpdate(motorList[1][2], 1)
                                sghGC.pinUpdate(
                                    motorList[1][1], (100 - self.turnSpeed), "pwmmotor")

                    elif "piringo" in ADDON:  # piringo
                        # do piringo stuff
                        # Check for all off/on type broadcasrs
                        self.bCheckAll()
                        # Check for LED off/on type broadcasts
                        self.bLEDCheck(piringoOutputs)
                        # Vary LED Brightness
                        self.bLEDPowerCheck(piringoOutputs)

                    elif "pibrella" in ADDON:  # PiBrella
                        #print ("PiBrella broadcast processing")
                        # Check for all off/on type broadcasts
                        self.bCheckAll()

                        self.bListCheck([13, 11, 7, 15, 16, 18, 22],
                                        ["led1", "led2", "led3", "led4", "led5", "led6", "led7"])
                        self.bListCheck([13, 11, 11, 11, 7, 15, 16, 18, 22],
                                        ["red", "amber", "yellow", "orange", "green", "outpute", "outputf", "outputg",
                                         "outputh"])

                        if self.bFindValue("beep"):
                            try:
                                bn, bd = self.value.split(",")
                            except:
                                bd = "0.25"
                                try:
                                    bn = int(self.valueNumeric)
                                except:
                                    bn = "60"
                            beepNote = int(float(bn))
                            beepDuration = (float(bd))
                            svalue = int(
                                self.valueNumeric) if self.valueIsNumeric else 60
                            beepThread = threading.Thread(target=self.beep,
                                                          args=[12, 440 * 2 ** ((beepNote - 69) / 12.0), beepDuration])
                            beepThread.start()

                        if self.bFind('sonare,a'):
                            distance = sghGC.pinSonar2(15, 21)
                            # print'Distance:',distance,'cm'
                            sensor_name = 'sonara'
                            bcast_str = 'sensor-update "%s" %d' % (
                                sensor_name, distance)
                            # print 'sending: %s' % bcast_str
                            msgQueue.put((5, bcast_str))

                    elif "rgbled" in ADDON:  # rgb-led

                        #print ("rgb-led broadcast processing")
                        if self.bFindOnOff("all"):
                            for loop in range(0, 5):
                                sghGC.pinUpdate(
                                    rgbOutputs[loop], 1 - self.OnOrOff)
                            for loop in range(5, 8):
                                sghGC.pinUpdate(rgbOutputs[loop], self.OnOrOff)

                        rgbList = rgbOutputs[0:5]
                        for listLoop in rgbList:
                            if self.bFindOnOff("led" + str(1 + rgbList.index(listLoop))):
                                sghGC.pinUpdate(
                                    rgbOutputs[rgbList.index(listLoop)], 1 - self.OnOrOff)

                        rgbList = ["red", "green", "blue"]
                        for listLoop in rgbList:
                            if self.bFindOnOff(listLoop):
                                print listLoop, "found",
                                sghGC.pinUpdate(
                                    rgbOutputs[5 + rgbList.index(listLoop)], self.OnOrOff)

                    elif "pidie" in ADDON:  # pidie
                        # do piringo stuff
                        # Check for all off/on type broadcasrs
                        self.bCheckAll()

                        # Vary LED Brightness
                        self.bLEDPowerCheck(pidieOutputs)

                        self.bListCheck([7, 11, 12, 13, 15, 16, 18, 22, 8],
                                        ["led1", "led2", "led3", "led4", "led5", "led6", "led7", "led8", "led9"])
                        self.bListCheckPowerOnly([7, 11, 12, 13, 15, 16, 18, 22, 8],
                                                 ["1", "2", "3", "4", "5", "6", "7", "8", "9"])

                    elif "traffichat" in ADDON:  # traffichat
                                                # based off fish dish and piringo
                        # do traffichat stuff
                        # Check for all off/on type broadcasrs
                        self.bCheckAll()
                        # Check for LED off/on type broadcasts
                        self.bLEDCheck(traffichatOutputs)
                        # Vary LED Brightness
                        self.bLEDPowerCheck(traffichatOutputs)

                        traffichatList = ["green", "yellow", "red"]
                        for listLoop in traffichatList:
                            if self.bFindOnOff(listLoop):
                                print listLoop, "found",
                                sghGC.pinUpdate(
                                    traffichatOutputs[traffichatList.index(listLoop)], self.OnOrOff)

                        if self.bFindOnOff('buzzer'):
                            sghGC.pinUpdate(29, self.OnOrOff)

                    elif "fishdish" in ADDON:  # fishdish
                        # do piringo stuff
                        # Check for all off/on type broadcasrs
                        self.bCheckAll()
                        # Check for LED off/on type broadcasts
                        self.bLEDCheck(fishOutputs)
                        self.bLEDPowerCheck(fishOutputs)  # Vary LED Brightness

                        fishList = ["green", "yellow", "red"]
                        for listLoop in fishList:
                            if self.bFindOnOff(listLoop):
                                print listLoop, "found",
                                sghGC.pinUpdate(
                                    fishOutputs[fishList.index(listLoop)], self.OnOrOff)

                        if self.bFindOnOff('buzzer'):
                            sghGC.pinUpdate(24, self.OnOrOff)

                    elif "p2g3" in ADDON:
                        if (pcaPWM is not None):
                            # go thru PowerPWM on PCA Board
                            for i in range(0, 5):
                                if self.bFindValue('blue'):
                                    svalue = int(
                                        self.valueNumeric) if self.valueIsNumeric else 100 if self.value == "on" else 0
                                    svalue = svalue * sghGC.ledDim / 100
                                    svalue = min(
                                        4095, max((((100 - svalue) * 4096) / 100), 0))
                                    pcaPWM.setPWM((i * 3), 0, svalue)
                                if self.bFindValue('green'):
                                    svalue = int(
                                        self.valueNumeric) if self.valueIsNumeric else 100 if self.value == "on" else 0
                                    svalue = svalue * sghGC.ledDim / 100
                                    svalue = min(
                                        4095, max((((100 - svalue) * 4096) / 100), 0))
                                    pcaPWM.setPWM((i * 3) + 1, 0, svalue)
                                if self.bFindValue('red'):
                                    svalue = int(
                                        self.valueNumeric) if self.valueIsNumeric else 100 if self.value == "on" else 0
                                    svalue = svalue * sghGC.ledDim / 100
                                    svalue = min(
                                        4095, max((((100 - svalue) * 4096) / 100), 0))
                                    pcaPWM.setPWM((i * 3) + 2, 0, svalue)
                                if self.bFindOnOff('all'):
                                    svalue = int(
                                        self.valueNumeric) if self.valueIsNumeric else 100 if self.value == "on" else 0
                                    svalue = svalue * sghGC.ledDim / 100
                                    svalue = min(
                                        4095, max((((100 - svalue) * 4096) / 100), 0))
                                    pcaPWM.setPWM((i * 3), 0, svalue)
                                    pcaPWM.setPWM((i * 3) + 1, 0, svalue)
                                    pcaPWM.setPWM((i * 3) + 2, 0, svalue)

                                    # Start using ultrasonic sensor on a pin
                        if self.bFindOnOff('ultra'):
                            self.startUltra(8, 0, self.OnOrOff)

                    elif "pi2golite" in ADDON:  #
                        # do pi2golite stuff
                        self.bCheckAll(False, [15, 16])
                        self.bListCheck([15, 16], ["frontleds", "backleds"])

                        motorList = [
                            ['turnl', 26, 24, 12], ['turnr', 19, 21, 13]]

                        moveFound = False

                        if self.bFindValue("movea"):
                            while sghGC.encoderInUse > 0:
                                time.sleep(0.1)
                            sghGC.encoderInUse = 1

                            # set turning sensor to turning
                            msgQueue.put(
                                (5, 'sensor-update "motors" "turning"'))
                            time.sleep(0.2)
                            moveFound = True
                            print " "
                            svalue = int(
                                self.valueNumeric) if self.valueIsNumeric else 0
                            #svalue = int(float(svalue * 4) / 10.0)
                            print "movea", svalue
                            sghGC.countDirection[
                                motorList[0][3]] = -1 if svalue < 0 else 1
                            print "direction", sghGC.countDirection[motorList[0][3]]
                            # sghGC.pinRead(motorList[0][3])
                            sghGC.pinLastState[motorList[0][3]] = -1
                            print "encoder state before turning starts", motorList[0][3], sghGC.pinLastState[
                                motorList[0][3]]
                            moveMotorAThread = threading.Thread(target=self.moveMotor,
                                                                args=[motorList[0], svalue, motorList[0][3]])
                            moveMotorAThread.start()

                        if self.bFindValue("moveb"):
                            while sghGC.encoderInUse > 0:
                                time.sleep(0.1)
                            sghGC.encoderInUse = 1

                            # set turning sensor to turning
                            msgQueue.put(
                                (5, 'sensor-update "motors" "turning"'))
                            time.sleep(0.2)
                            moveFound = True
                            print " "
                            svalue = int(
                                self.valueNumeric) if self.valueIsNumeric else 0
                            #svalue = int(float(svalue * 4) / 10.0)
                            print "movea", svalue
                            sghGC.countDirection[
                                motorList[1][3]] = -1 if svalue < 0 else 1
                            print "direction", sghGC.countDirection[motorList[1][3]]
                            # sghGC.pinRead(motorList[0][3])
                            sghGC.pinLastState[motorList[1][3]] = -1
                            print "encoder state before turning starts", motorList[1][3], sghGC.pinLastState[
                                motorList[1][3]]
                            moveMotorBThread = threading.Thread(target=self.moveMotor,
                                                                args=[motorList[1], svalue, motorList[1][3]])
                            moveMotorBThread.start()

                        if self.bFindValue("move") and moveFound == False:
                            while sghGC.encoderInUse > 0:
                                time.sleep(0.1)
                            sghGC.encoderInUse = 2
                            # set turning sensor to turning
                            msgQueue.put(
                                (5, 'sensor-update "motors" "turning"'))
                            time.sleep(0.2)
                            print " "
                            svalue = int(
                                self.valueNumeric) if self.valueIsNumeric else 0
                            #svalue = int(float(svalue * 4) / 10.0)
                            print "move ", svalue
                            sghGC.countDirection[
                                motorList[0][3]] = -1 if svalue < 0 else 1
                            sghGC.countDirection[
                                motorList[1][3]] = -1 if svalue < 0 else 1
                            print "sghdir", sghGC.countDirection[motorList[0][3]]
                            moveMotorAThread = threading.Thread(target=self.moveMotor,
                                                                args=[motorList[0], svalue, motorList[0][3]])
                            moveMotorAThread.start()
                            moveMotorBThread = threading.Thread(target=self.moveMotor,
                                                                args=[motorList[1], svalue, motorList[1][3]])
                            moveMotorBThread.start()

                        if self.bFindValue("turn"):
                            while sghGC.encoderInUse > 0:
                                time.sleep(0.1)
                            sghGC.encoderInUse = 2
                            # set turning sensor to turning
                            msgQueue.put(
                                (5, 'sensor-update "motors" "turning"'))
                            time.sleep(0.2)
                            print " "
                            svalue = int(
                                self.valueNumeric) if self.valueIsNumeric else 0
                            #svalue = int(float(svalue) / 22.5)
                            print "turn ", svalue
                            sghGC.countDirection[
                                motorList[0][3]] = -1 if svalue < 0 else 1
                            sghGC.countDirection[
                                motorList[1][3]] = 1 if svalue < 0 else -1
                            moveMotorAThread = threading.Thread(target=self.moveMotor,
                                                                args=[motorList[0], svalue, motorList[0][3]])
                            moveMotorAThread.start()
                            moveMotorBThread = threading.Thread(target=self.moveMotor,
                                                                args=[motorList[1], -svalue, motorList[1][3]])
                            moveMotorBThread.start()

                    elif "pi2go" in ADDON:
                        if (pcaPWM is not None):
                            # go thru PowerPWM on PCA Board
                            for i in range(0, 4):
                                if self.bFindValue('blue'):
                                    svalue = int(
                                        self.valueNumeric) if self.valueIsNumeric else 100 if self.value == "on" else 0
                                    svalue = svalue * sghGC.ledDim / 100
                                    svalue = min(
                                        4095, max((((svalue) * 4096) / 100), 0))
                                    pcaPWM.setPWM((i * 3), 0, svalue)
                                if self.bFindValue('green'):
                                    svalue = int(
                                        self.valueNumeric) if self.valueIsNumeric else 100 if self.value == "on" else 0
                                    svalue = svalue * sghGC.ledDim / 100
                                    svalue = min(
                                        4095, max((((svalue) * 4096) / 100), 0))
                                    pcaPWM.setPWM((i * 3) + 1, 0, svalue)
                                if self.bFindValue('red'):
                                    svalue = int(
                                        self.valueNumeric) if self.valueIsNumeric else 100 if self.value == "on" else 0
                                    svalue = svalue * sghGC.ledDim / 100
                                    svalue = min(
                                        4095, max((((svalue) * 4096) / 100), 0))
                                    pcaPWM.setPWM((i * 3) + 2, 0, svalue)
                                if self.bFindOnOff('all'):
                                    svalue = int(
                                        self.valueNumeric) if self.valueIsNumeric else 100 if self.value == "on" else 0
                                    svalue = svalue * sghGC.ledDim / 100
                                    svalue = min(
                                        4095, max((((svalue) * 4096) / 100), 0))
                                    pcaPWM.setPWM((i * 3), 0, svalue)
                                    pcaPWM.setPWM((i * 3) + 1, 0, svalue)
                                    pcaPWM.setPWM((i * 3) + 2, 0, svalue)

                                    # Start using ultrasonic sensor on a pin
                        if self.bFindOnOff('ultra'):
                            self.startUltra(8, 0, self.OnOrOff)
                        if "startlightinfo" in dataraw:
                            sghGC.lightInfo = True

                        def set_neopixel(led, red, green, blue):
                            pcaPWM.setPWM(
                                (led * 3) + 2, 0, min(4095, max((((red) * 4096) / 255), 0)))
                            pcaPWM.setPWM(
                                (led * 3) + 1, 0, min(4095, max((((green) * 4096) / 255), 0)))
                            pcaPWM.setPWM(
                                (led * 3), 0, min(4095, max((((blue) * 4096) / 255), 0)))

                        def pi2go_mapName(name):
                            print name
                            try:
                                return ['left', 'back', 'right', 'front'].index(name) + 1
                            except:
                                return 0

                        lettercolours = [
                            'r', 'g', 'b', 'c', 'm', 'y', 'w', '0', '1', 'z']
                        ledcolours = ['red', 'green', 'blue', 'cyan', 'magenta', 'yellow', 'white', 'off', 'on',
                                      'invert', 'random']

                        # only define dictionary on first pass
                        if tcolours is None:
                            tcolours = {'red': (255, 0, 0), 'green': (0, 255, 0), 'blue': (0, 0, 255),
                                        'cyan': (0, 255, 255), 'magenta': (255, 0, 255), 'yellow': (255, 255, 0),
                                        'white': (255, 255, 255), 'off': (0, 0, 0), 'on': (255, 255, 255),
                                        'invert': (0, 0, 0)}

                        self.matrixUse = 4

                        if self.bFind("allon"):
                            for index in range(0, self.matrixUse):
                                set_neopixel(
                                    index, self.matrixRed, self.matrixGreen, self.matrixBlue)

                        if self.bFind("alloff"):
                            for index in range(0, self.matrixUse):
                                set_neopixel(index, 0, 0, 0)

                        if self.bFind("sweep"):
                            print "sweep"
                            for index in range(0, self.matrixUse):
                                self.matrixRed, self.matrixGreen, self.matrixBlue = tcolours.get(
                                    ledcolours[random.randint(0, 6)], (0, 0, 0))
                                set_neopixel(
                                    index, self.matrixRed, self.matrixGreen, self.matrixBlue)
                                time.sleep(0.05)

                        if self.bFindValue("red"):
                            self.matrixRed = int(
                                self.valueNumeric) if self.valueIsNumeric else 0
                            if self.value == "on":
                                self.matrixRed = 255
                            if self.value == "off":
                                self.matrixRed = 0

                        if self.bFindValue("green"):
                            self.matrixGreen = int(
                                self.valueNumeric) if self.valueIsNumeric else 0
                            if self.value == "on":
                                self.matrixGreen = 255
                            if self.value == "off":
                                self.matrixGreen = 0

                        if self.bFindValue("blue"):
                            self.matrixBlue = int(
                                self.valueNumeric) if self.valueIsNumeric else 0
                            if self.value == "on":
                                self.matrixBlue = 255
                            if self.value == "off":
                                self.matrixBlue = 0

                        if self.bFindValue("colour"):
                            # print "colour" ,self.value
                            if self.value == "invert":
                                tcolours[
                                    "invert"] = 255 - self.matrixRed, 255 - self.matrixGreen, 255 - self.matrixBlue
                            if self.valueIsNumeric:
                                colourIndex = max(
                                    1, min(8, int(self.value))) - 1
                                self.matrixRed, self.matrixGreen, self.matrixBlue = tcolours.get(
                                    ledcolours[colourIndex], (128, 128, 128))
                            else:
                                if self.value[0] == "#":
                                    try:
                                        self.value = (
                                            self.value + "00000000")[0:7]
                                        self.matrixRed = int(
                                            self.value[1:3], 16)
                                        self.matrixGreen = int(
                                            self.value[3:5], 16)
                                        self.matrixBlue = int(
                                            self.value[5:], 16)
                                        # print "matrxired", self.matrixRed
                                    except:
                                        pass
                                else:
                                    ledcolour = self.value
                                    self.matrixRed, self.matrixGreen, self.matrixBlue = tcolours.get(self.value, (
                                        128, 128, 128))
                                    if ledcolour == 'random':
                                        self.matrixRed, self.matrixGreen, self.matrixBlue = tcolours.get(
                                            ledcolours[random.randint(0, 6)], (128, 128, 128))

                            tcolours[
                                "on"] = self.matrixRed, self.matrixGreen, self.matrixBlue

                        if self.bFind("pixel"):
                            print "pixel detected"
                            pixelProcessed = False
                            for led in range(0, self.matrixUse):
                                if (self.bFindValue("pixel") and pi2go_mapName(self.value) == str(led + 1)):
                                    set_neopixel(
                                        led, self.matrixRed, self.matrixGreen, self.matrixBlue)
                                    pixelProcessed = True

                            if not pixelProcessed:
                                for led in range(0, self.matrixUse):
                                    for ledcolour in ledcolours:
                                        if (self.bFindValue("pixel", ledcolour)) and pi2go_mapName(self.value) == str(led + 1):
                                            self.matrixRed, self.matrixGreen, self.matrixBlue = tcolours.get(
                                                ledcolour, (self.matrixRed, self.matrixGreen, self.matrixBlue))
                                            if ledcolour == 'random':
                                                self.matrixRed, self.matrixGreen, self.matrixBlue = tcolours.get(
                                                    ledcolours[random.randint(0, 6)], (64, 64, 64))
                                            # print
                                            # "pixel",self.matrixRed,self.matrixGreen,self.matrixBlue
                                            if self.valueIsNumeric:
                                                if (ledcolour != "invert"):
                                                    set_neopixel(
                                                        led, self.matrixRed, self.matrixGreen, self.matrixBlue)
                                                pixelProcessed = True

                            if not pixelProcessed:
                                # print "#", self.value[-7:-7]
                                fullvalue = self.value
                                if ("xxxxxxx" + fullvalue)[-7] == "#":
                                    for led in range(0, self.matrixUse):
                                        if (self.bFindValue("pixel", fullvalue[-7:]) and self.value == str(
                                                led + 1)):
                                            try:
                                                c = (
                                                    fullvalue[-7:] + "00000000")[0:7]
                                                # print "full", c
                                                r = int(c[1:3], 16)
                                                g = int(c[3:5], 16)
                                                b = int(c[5:], 16)
                                                set_neopixel(led, r, g, b)
                                                pixelProcessed = True
                                            except:
                                                pass
                                # print "0x" , fullvalue, ("........" +
                                # fullvalue)[-8:-6]
                                if ("........" + fullvalue)[-8:-6] == "0x":
                                    for led in range(0, self.matrixUse):
                                        if (self.bFindValue("pixel", fullvalue[-8:]) and self.value == str(
                                                led + 1)):
                                            try:
                                                c = (
                                                    fullvalue[-7:] + "00000000")[0:7]
                                                # print "full", c
                                                r = int(c[1:3], 16)
                                                g = int(c[3:5], 16)
                                                b = int(c[5:], 16)
                                                set_neopixel(led, r, g, b)
                                                pixelProcessed = True
                                            except:
                                                pass

                            if self.bFindValue("bright"):
                                sghGC.ledDim = int(
                                    self.valueNumeric) if self.valueIsNumeric else 20
                                sensor_name = 'bright'
                                bcast_str = 'sensor-update "%s" %d' % (
                                    sensor_name, sghGC.ledDim)
                                # print 'sending: %s' % bcast_str
                                msgQueue.put((5, bcast_str))

                    elif "raspibot2" in ADDON:
                        # Check for all off/on type broadcasrs
                        self.bCheckAll()
                        self.bListCheck([26, 24, 15, 13],
                                        ["led1", "led2", "output1", "output2"])  # Check for LED off/on type broadcasts
                        if self.bFind('sonar'):
                            distance = sghGC.pinSonar2(12, 16)
                            # print'Distance:',distance,'cm'
                            sensor_name = 'sonar'
                            bcast_str = 'sensor-update "%s" %d' % (
                                sensor_name, distance)
                            # print 'sending: %s' % bcast_str
                            msgQueue.put((5, bcast_str))

                    elif "pizazz" in ADDON:
                        # Check for all off/on type broadcasrs
                        self.bCheckAll()
                        # Check for LEDs
                        self.bListCheck(
                            [22, 18, 11, 7], ["led1", "led2", "led3", "led4"])

                        if self.bFind('sonar'):
                            distance = sghGC.pinSonar(8)
                            # print'Distance:',distance,'cm'
                            sensor_name = 'sonar'
                            bcast_str = 'sensor-update "%s" %d' % (
                                sensor_name, distance)
                            # print 'sending: %s' % bcast_str
                            msgQueue.put((5, bcast_str))

                        # Start using ultrasonic sensor on a pin
                        if self.bFindOnOff('ultra'):
                            self.startUltra(8, 0, self.OnOrOff)

                    elif "simpie" in ADDON:
                        # do BerryClip stuff
                        if self.bFindOnOff('all'):
                            for pin in [12, 16, 18]:
                                sghGC.pinUpdate(pin, self.OnOrOff)
                            sghGC.pinUpdate(7, self.OnOrOff)
                        # Check for LEDs
                        self.bListCheck([12, 16, 18], ["red", "green", "blue"])

                        if self.bFindOnOff('buzzer'):
                            sghGC.pinUpdate(7, 1 - self.OnOrOff)

                    elif "techtom" in ADDON:  # Gordon's Ladder Board
                        # do techtom stuff
                        #print ("TechTom broadcast processing")
                        # Check for all off/on type broadcasrs
                        self.bCheckAll()
                        # Check for pin on off
                        self.bPinCheck([8, 10, 12, 16, 18, 22, 24, 26])

                    elif "ledborg" in ADDON:  # LedBorg

                        if self.bFindOnOff('all'):
                            for pin in [11, 13, 15]:
                                sghGC.pinUpdate(pin, self.OnOrOff)
                        # Check for LEDs
                        self.bListCheck([11, 13, 15], ["red", "green", "blue"])

                    # Matrix connected
                    elif ("unicorn") in ADDON or ("neopixels" in ADDON) or ("playhat" in ADDON):
                        oldADDON = ADDON
                        if "playhat" in ADDON:
                            ADDON = ADDON + " neopixels9"
                        if UH is None:
                            # try:
                            import sgh_unicornhat as UH
                            print "UnicornHat imported OK"
                            # except:
                            # print "UnicornHat software not installed"
                            # break
                        # print "addon", ADDON
                        lettercolours = [
                            'r', 'g', 'b', 'c', 'm', 'y', 'w', '0', '1', 'z']
                        ledcolours = ['red', 'green', 'blue', 'cyan', 'magenta', 'yellow', 'white', 'off', 'on',
                                      'invert', 'random']

                        # only define dictionary on first pass
                        if tcolours is None:
                            tcolours = {'red': (255, 0, 0), 'green': (0, 255, 0), 'blue': (0, 0, 255),
                                        'cyan': (0, 255, 255), 'magenta': (255, 0, 255), 'yellow': (255, 255, 0),
                                        'white': (255, 255, 255), 'off': (0, 0, 0), 'on': (255, 255, 255),
                                        'invert': (0, 0, 0)}

                        if ("neopixels" in ADDON):
                            self.matrixUse = int(
                                rtnNumeric(ADDON[ADDON.index('neopixels'):], 64))
                            # print "neopixels",self.matrixUse

                        # print
                        origdataraw = self.dataraw
                        self.dataraw = self.dataraw[self.dataraw.find(
                            "broadcast") + 10:]  # split dataraw so that operations are sequential
                        # print "inside unicorn" , self.dataraw

                        # print "data before split" ,self.dataraw
                        broadcastList = ((self.dataraw).strip()).split(' ')
                        # print "broadcastList" , broadcastList

                        for broadcastListLoop in broadcastList:
                            self.dataraw = " " + str(broadcastListLoop) + " "
                            # print "inside inner loop", self.dataraw

                            # print "self.matrixuse", self.matrixUse

                            if self.bFindValue("matrixuse"):
                                # print "mu" , self.value
                                if self.value == '4':
                                    self.matrixUse = 4
                                    self.matrixMult = 4
                                    self.matrixLimit = 4
                                    self.matrixRangemax = 2
                                    # print
                                    # self.matrixMult,self.matrixLimit,self.matrixRangemax
                                elif self.value == '9':
                                    self.matrixUse = 9
                                    self.matrixMult = 3
                                    self.matrixLimit = 2
                                    self.matrixRangemax = 3
                                    # print
                                    # self.matrixMult,self.matrixLimit,self.matrixRangemax
                                elif self.value == '16':
                                    self.matrixUse = 16
                                    self.matrixMult = 2
                                    self.matrixLimit = 2
                                    self.matrixRangemax = 4
                                    # print
                                    # self.matrixMult,self.matrixLimit,self.matrixRangemax
                                else:
                                    self.matrixUse = 64
                                    self.matrixMult = 1
                                    self.matrixLimit = 1
                                    self.matrixRangemax = 8
                                    # print
                                    # self.matrixMult,self.matrixLimit,self.matrixRangemax

                            # print "outside",
                            # self.matrixMult,self.matrixLimit,self.matrixRangemax

                            if "unicorn" in ADDON:
                                if self.bFind("allon"):
                                    for y in range(0, 8):
                                        for x in range(0, 8):
                                            UH.set_pixel(
                                                x, y, self.matrixRed, self.matrixGreen, self.matrixBlue)
                                    UH.show()

                                if self.bFind("alloff"):
                                    for y in range(0, 8):
                                        for x in range(0, 8):
                                            UH.set_pixel(x, y, 0, 0, 0)
                                    UH.show()

                                if self.bFind("sweep"):
                                    print "sweep"

                                    for ym in range(0, self.matrixRangemax):
                                        for xm in range(0, self.matrixRangemax):
                                            self.matrixRed, self.matrixGreen, self.matrixBlue = tcolours.get(
                                                ledcolours[random.randint(0, 6)], (0, 0, 0))
                                            for yy in range(0, self.matrixLimit):
                                                for xx in range(0, self.matrixLimit):
                                                    UH.set_pixel((xm * self.matrixMult) + xx,
                                                                 7 -
                                                                 ((ym * self.matrixMult) +
                                                                  yy), self.matrixRed,
                                                                 self.matrixGreen, self.matrixBlue)
                                            UH.show()
                                            time.sleep(0.05)
                            else:
                                if self.bFind("allon"):
                                    for index in range(0, self.matrixUse):
                                        UH.set_neopixel(
                                            index, self.matrixRed, self.matrixGreen, self.matrixBlue)
                                    UH.show()

                                if self.bFind("alloff"):
                                    for index in range(0, self.matrixUse):
                                        UH.set_neopixel(index, 0, 0, 0)
                                    UH.show()

                                if self.bFind("sweep"):
                                    print "sweep"
                                    for index in range(0, self.matrixUse):
                                        self.matrixRed, self.matrixGreen, self.matrixBlue = tcolours.get(
                                            ledcolours[random.randint(0, 6)], (0, 0, 0))
                                        UH.set_neopixel(
                                            index, self.matrixRed, self.matrixGreen, self.matrixBlue)
                                        UH.show()
                                        time.sleep(0.05)

                            if self.bFindValue("red"):
                                self.matrixRed = int(
                                    self.valueNumeric) if self.valueIsNumeric else 0
                                if self.value == "on":
                                    self.matrixRed = 255
                                if self.value == "off":
                                    self.matrixRed = 0

                            if self.bFindValue("green"):
                                self.matrixGreen = int(
                                    self.valueNumeric) if self.valueIsNumeric else 0
                                if self.value == "on":
                                    self.matrixGreen = 255
                                if self.value == "off":
                                    self.matrixGreen = 0

                            if self.bFindValue("blue"):
                                self.matrixBlue = int(
                                    self.valueNumeric) if self.valueIsNumeric else 0
                                if self.value == "on":
                                    self.matrixBlue = 255
                                if self.value == "off":
                                    self.matrixBlue = 0

                            if self.bFindValue("colour"):
                                # print "colour" ,self.value
                                if self.value == "invert":
                                    tcolours[
                                        "invert"] = 255 - self.matrixRed, 255 - self.matrixGreen, 255 - self.matrixBlue
                                if self.valueIsNumeric:
                                    colourIndex = max(
                                        1, min(8, int(self.value))) - 1
                                    self.matrixRed, self.matrixGreen, self.matrixBlue = tcolours.get(
                                        ledcolours[colourIndex], (128, 128, 128))
                                else:
                                    if self.value[0] == "#":
                                        try:
                                            self.value = (
                                                self.value + "00000000")[0:7]
                                            self.matrixRed = int(
                                                self.value[1:3], 16)
                                            self.matrixGreen = int(
                                                self.value[3:5], 16)
                                            self.matrixBlue = int(
                                                self.value[5:], 16)
                                            # print "matrxired", self.matrixRed
                                        except:
                                            pass
                                    else:
                                        ledcolour = self.value
                                        self.matrixRed, self.matrixGreen, self.matrixBlue = tcolours.get(self.value, (
                                            128, 128, 128))
                                        if ledcolour == 'random':
                                            self.matrixRed, self.matrixGreen, self.matrixBlue = tcolours.get(
                                                ledcolours[random.randint(0, 6)], (128, 128, 128))
                                tcolours[
                                    "on"] = self.matrixRed, self.matrixGreen, self.matrixBlue

                                # print "rgb", self.matrixRed,self.matrixGreen,self.matrixBlue
                                # print tcolours

                            if "unicorn" in ADDON:
                                if self.bFind("pixel"):
                                    pixelProcessed = False
                                    for ym in range(0, self.matrixRangemax):
                                        for xm in range(0, self.matrixRangemax):
                                            for ledcolour in ledcolours:
                                                if (self.bFindValue("pixel", ledcolour) and (
                                                        self.value == (str(xm + 1) + "," + str(ym + 1)))):
                                                    print "1st catch,xm,ym", xm, ym
                                                    self.matrixRed, self.matrixGreen, self.matrixBlue = tcolours.get(
                                                        ledcolour, (self.matrixRed, self.matrixGreen, self.matrixBlue))
                                                    if ledcolour == 'random':
                                                        self.matrixRed, self.matrixGreen, self.matrixBlue = tcolours.get(
                                                            ledcolours[random.randint(0, 6)], (64, 64, 64))
                                                    # print
                                                    # "pixel",self.matrixRed,self.matrixGreen,self.matrixBlue
                                                    for yy in range(0, self.matrixLimit):
                                                        for xx in range(0, self.matrixLimit):
                                                            # print "led no"
                                                            # ,led
                                                            if (ledcolour != "invert"):
                                                                UH.set_pixel((xm * self.matrixMult) + xx,
                                                                             7 -
                                                                             (
                                                                                 (ym * self.matrixMult) + yy),
                                                                             self.matrixRed, self.matrixGreen,
                                                                             self.matrixBlue)
                                                            else:
                                                                gnp = UH.get_pixel((xm * self.matrixMult) + xx,
                                                                                   7 - ((ym * self.matrixMult) + yy))
                                                                # print
                                                                # "before" ,gnp
                                                                gnpi = map(
                                                                    lambda a: (255 - a), gnp)
                                                                # print
                                                                # "after", gnpi
                                                                r, g, b = gnpi
                                                                # print "rgb",
                                                                # r,g,b
                                                                UH.set_pixel((xm * self.matrixMult) + xx,
                                                                             7 - ((ym * self.matrixMult) + yy), r, g, b)
                                                    UH.show()
                                                    pixelProcessed = True

                                    if not pixelProcessed:
                                        # print "#", self.value[-7:]
                                        fullvalue = self.value
                                        if ("xxxxxxx" + fullvalue)[-7] == "#":
                                            for ym in range(0, self.matrixRangemax):
                                                for xm in range(0, self.matrixRangemax):
                                                    if (self.bFindValue("pixel", fullvalue[-7:]) and (
                                                            self.value == (str(xm + 1) + "," + str(ym + 1)))):
                                                        # print
                                                        # "led,self.value",led,self.value
                                                        try:
                                                            c = (
                                                                fullvalue[-7:] + "00000000")[0:7]
                                                            # print "full", c
                                                            r = int(c[1:3], 16)
                                                            g = int(c[3:5], 16)
                                                            b = int(c[5:], 16)
                                                            for yy in range(0, self.matrixLimit):
                                                                for xx in range(0, self.matrixLimit):
                                                                    UH.set_pixel((xm * self.matrixMult) + xx,
                                                                                 7 -
                                                                                 (
                                                                                     (ym * self.matrixMult) + yy), r,
                                                                                 g, b)
                                                            UH.show()
                                                            pixelProcessed = True
                                                        except:
                                                            pass

                                    if not pixelProcessed:
                                        for ym in range(0, self.matrixRangemax):
                                            for xm in range(0, self.matrixRangemax):
                                                if self.bFindValue("pixel" + str(xm + 1) + "," + str(ym + 1)):
                                                    ledcolour = self.value
                                                    self.matrixRed, self.matrixGreen, self.matrixBlue = tcolours.get(
                                                        ledcolour, (self.matrixRed, self.matrixGreen, self.matrixBlue))
                                                    if ledcolour == 'random':
                                                        self.matrixRed, self.matrixGreen, self.matrixBlue = tcolours.get(
                                                            ledcolours[random.randint(0, 6)], (32, 32, 32))
                                                    print "3rd catch xm,ym ", xm, ym
                                                    for yy in range(0, self.matrixLimit):
                                                        for xx in range(0, self.matrixLimit):
                                                            UH.set_pixel((xm * self.matrixMult) + xx,
                                                                         7 -
                                                                         (
                                                                             (ym * self.matrixMult) + yy),
                                                                         self.matrixRed, self.matrixGreen,
                                                                         self.matrixBlue)
                                                    UH.show()
                                                    pixelProcessed = True

                                    if not pixelProcessed:
                                        for led in range(0, self.matrixUse):
                                            if (self.bFindValue("pixel") and self.value == str(led + 1)):
                                                ym = int(
                                                    int(led) / self.matrixRangemax)
                                                xm = led % self.matrixRangemax
                                                # print "xm,ym" ,xm,ym
                                                # print self.matrixRed,self.matrixGreen,self.matrixBlue
                                                # print
                                                # self.matrixMult,self.matrixLimit,self.matrixRangemax,led,
                                                # ym, ym
                                                for yy in range(0, self.matrixLimit):
                                                    for xx in range(0, self.matrixLimit):
                                                        UH.set_pixel((xm * self.matrixMult) + xx,
                                                                     7 -
                                                                     ((ym * self.matrixMult) +
                                                                      yy), self.matrixRed,
                                                                     self.matrixGreen, self.matrixBlue)
                                                UH.show()
                                                pixelProcessed = True

                                    if not pixelProcessed:
                                        for led in range(0, self.matrixUse):
                                            for ledcolour in ledcolours:
                                                if (self.bFindValue("pixel", ledcolour) and self.value == str(led + 1)):
                                                    ym = int(
                                                        int(led) / self.matrixRangemax)
                                                    xm = led % self.matrixRangemax
                                                    # print "xm,ym" ,xm,ym
                                                    self.matrixRed, self.matrixGreen, self.matrixBlue = tcolours.get(
                                                        ledcolour, (self.matrixRed, self.matrixGreen, self.matrixBlue))
                                                    if ledcolour == 'random':
                                                        self.matrixRed, self.matrixGreen, self.matrixBlue = tcolours.get(
                                                            ledcolours[random.randint(0, 6)], (64, 64, 64))
                                                    # print
                                                    # "pixel",self.matrixRed,self.matrixGreen,self.matrixBlue
                                                    for yy in range(0, self.matrixLimit):
                                                        for xx in range(0, self.matrixLimit):
                                                            # print "led no"
                                                            # ,led
                                                            if (ledcolour != "invert"):
                                                                UH.set_pixel((xm * self.matrixMult) + xx,
                                                                             7 -
                                                                             (
                                                                                 (ym * self.matrixMult) + yy),
                                                                             self.matrixRed, self.matrixGreen,
                                                                             self.matrixBlue)
                                                            else:
                                                                gnp = UH.get_pixel((xm * self.matrixMult) + xx,
                                                                                   7 - ((ym * self.matrixMult) + yy))
                                                                # print
                                                                # "before" ,gnp
                                                                gnpi = map(
                                                                    lambda a: (255 - a), gnp)
                                                                # print
                                                                # "after", gnpi
                                                                r, g, b = gnpi
                                                                # print "rgb",
                                                                # r,g,b
                                                                UH.set_pixel((xm * self.matrixMult) + xx,
                                                                             7 - ((ym * self.matrixMult) + yy), r, g, b)
                                                    UH.show()
                                                    pixelProcessed = True

                                    if not pixelProcessed:
                                        # print "#", self.value[-7:]
                                        fullvalue = self.value
                                        if ("xxxxxxx" + fullvalue)[-7] == "#":
                                            for led in range(0, self.matrixUse):
                                                if (self.bFindValue("pixel", fullvalue[-7:]) and self.value == str(
                                                        led + 1)):
                                                    ym = int(
                                                        int(led) / self.matrixRangemax)
                                                    xm = led % self.matrixRangemax
                                                    # print
                                                    # "led,self.value",led,self.value
                                                    try:
                                                        c = (
                                                            fullvalue[-7:] + "00000000")[0:7]
                                                        # print "full", c
                                                        r = int(c[1:3], 16)
                                                        g = int(c[3:5], 16)
                                                        b = int(c[5:], 16)
                                                        for yy in range(0, self.matrixLimit):
                                                            for xx in range(0, self.matrixLimit):
                                                                UH.set_pixel((xm * self.matrixMult) + xx,
                                                                             7 - ((ym * self.matrixMult) + yy), r, g, b)
                                                        UH.show()
                                                        pixelProcessed = True
                                                    except:
                                                        pass

                                if self.bFind("getpixel"):
                                    for ym in range(0, self.matrixRangemax):
                                        for xm in range(0, self.matrixRangemax):
                                            if self.bFindValue("getpixel" + str(xm + 1) + "," + str(ym + 1)):
                                                gnp = UH.get_pixel(
                                                    xm * self.matrixMult, 7 - (ym * self.matrixMult))
                                                # print "getpixel
                                                # led,xm,ymgnp",led,xm,ym,gnp
                                                r, g, b = gnp
                                                bcolourname = "#" + ("000000" + (str(
                                                    hex(b + (g * 256) + (r * 256 * 256))))[2:])[-6:]
                                                bcolour = str(r).zfill(
                                                    3) + str(g).zfill(3) + str(b).zfill(3)
                                                #ledcolours = ['red','green','blue','cyan','magenta','yellow','white','off','on','invert','random']
                                                #bcolourname = "black"
                                                try:
                                                    bcolourname = ledcolours[
                                                        ["255000000", "000255000", "000000255", "000255255",
                                                         "255000255", "255255000", "255255255"].index(bcolour)]
                                                except ValueError:
                                                    pass
                                                # print "col lookup",
                                                # bcolourname
                                                sensor_name = 'colour'
                                                bcast_str = 'sensor-update "%s" %s' % (
                                                    sensor_name, bcolourname)
                                                # print 'sending: %s' %
                                                # bcast_str
                                                msgQueue.put((5, bcast_str))

                                    for led in range(0, self.matrixUse):
                                        if (self.bFindValue("getpixel") and self.value == str(led + 1)):
                                            ym = int(
                                                int(led) / self.matrixRangemax)
                                            xm = led % self.matrixRangemax
                                            gnp = UH.get_pixel(
                                                xm * self.matrixMult, 7 - (ym * self.matrixMult))
                                            # print "getpixel
                                            # led,xm,ymgnp",led,xm,ym,gnp
                                            r, g, b = gnp
                                            bcolourname = "#" + ("000000" + (str(hex(b + (g * 256) + (r * 256 * 256))))[
                                                2:])[-6:]
                                            bcolour = str(r).zfill(
                                                3) + str(g).zfill(3) + str(b).zfill(3)
                                            #ledcolours = ['red','green','blue','cyan','magenta','yellow','white','off','on','invert','random']
                                            #bcolourname = "black"
                                            try:
                                                bcolourname = ledcolours[
                                                    ["255000000", "000255000", "000000255", "000255255", "255000255",
                                                     "255255000", "255255255"].index(bcolour)]
                                            except ValueError:
                                                pass
                                            # print "col lookup", bcolourname
                                            sensor_name = 'colour'
                                            bcast_str = 'sensor-update "%s" %s' % (
                                                sensor_name, bcolourname)
                                            # print 'sending: %s' % bcast_str
                                            msgQueue.put((5, bcast_str))

                            else:  # Neopixel processing
                                if self.bFind("pixel"):
                                    pixelProcessed = False
                                    for led in range(0, self.matrixUse):
                                        if (self.bFindValue("pixel") and self.value == str(led + 1)):
                                            UH.set_neopixel(
                                                led, self.matrixRed, self.matrixGreen, self.matrixBlue)
                                            UH.show()
                                            pixelProcessed = True

                                    if not pixelProcessed:
                                        for led in range(0, self.matrixUse):
                                            for ledcolour in ledcolours:
                                                if (self.bFindValue("pixel", ledcolour) and self.value == str(led + 1)):
                                                    self.matrixRed, self.matrixGreen, self.matrixBlue = tcolours.get(
                                                        ledcolour, (self.matrixRed, self.matrixGreen, self.matrixBlue))
                                                    if ledcolour == 'random':
                                                        self.matrixRed, self.matrixGreen, self.matrixBlue = tcolours.get(
                                                            ledcolours[random.randint(0, 6)], (64, 64, 64))
                                                    # print
                                                    # "pixel",self.matrixRed,self.matrixGreen,self.matrixBlue
                                                    if self.valueIsNumeric:
                                                        if (ledcolour != "invert"):
                                                            UH.set_neopixel(led, self.matrixRed, self.matrixGreen,
                                                                            self.matrixBlue)
                                                        else:
                                                            gnp = UH.get_neopixel(
                                                                led)
                                                            # print "before"
                                                            # ,gnp
                                                            gnpi = map(
                                                                lambda a: (255 - a), gnp)
                                                            # print "after",
                                                            # gnpi
                                                            r, g, b = gnpi
                                                            # print "rgb",
                                                            # r,g,b
                                                            UH.set_neopixel(
                                                                led, r, g, b)
                                                        UH.show()
                                                        pixelProcessed = True

                                    if not pixelProcessed:
                                        # print "#", self.value[-7:-7]
                                        fullvalue = self.value
                                        if ("xxxxxxx" + fullvalue)[-7] == "#":
                                            for led in range(0, self.matrixUse):
                                                if (self.bFindValue("pixel", fullvalue[-7:]) and self.value == str(
                                                        led + 1)):
                                                    try:
                                                        c = (
                                                            fullvalue[-7:] + "00000000")[0:7]
                                                        # print "full", c
                                                        r = int(c[1:3], 16)
                                                        g = int(c[3:5], 16)
                                                        b = int(c[5:], 16)
                                                        UH.set_neopixel(
                                                            led, r, g, b)
                                                        UH.show()
                                                        pixelProcessed = True
                                                    except:
                                                        pass
                                        # print "0x" , fullvalue, ("........" +
                                        # fullvalue)[-8:-6]
                                        if ("........" + fullvalue)[-8:-6] == "0x":
                                            for led in range(0, self.matrixUse):
                                                if (self.bFindValue("pixel", fullvalue[-8:]) and self.value == str(
                                                        led + 1)):
                                                    try:
                                                        c = (
                                                            fullvalue[-7:] + "00000000")[0:7]
                                                        # print "full", c
                                                        r = int(c[1:3], 16)
                                                        g = int(c[3:5], 16)
                                                        b = int(c[5:], 16)
                                                        UH.set_neopixel(
                                                            led, r, g, b)
                                                        UH.show()
                                                        pixelProcessed = True
                                                    except:
                                                        pass

                                if self.bFind("getpixel"):
                                    for led in range(0, self.matrixUse):
                                        if (self.bFindValue("getpixel") and self.value == str(led + 1)):
                                            gnp = UH.get_neopixel(int(led))
                                            # print "led,gnp",led,gnp
                                            r, g, b = gnp
                                            bcolourname = "#" + ("000000" + (str(hex(b + (g * 256) + (r * 256 * 256))))[
                                                2:])[-6:]
                                            # #print "bcolour", bcolour
                                            # sensor_name = 'colour#'
                                            # bcast_str = 'sensor-update "%s" %s' % (sensor_name, bcolour)
                                            # #print 'sending: %s' % bcast_str
                                            # msgQueue.put((5,bcast_str))

                                            # bcolour = "["  + str(r).zfill(3) + ","+ str(g).zfill(3)+ "," + str(b).zfill(3) + "]"
                                            # print "rgb bcolour", bcolour
                                            # sensor_name = 'rgbcolour'
                                            # bcast_str = 'sensor-update "%s" %s' % (sensor_name, bcolour)
                                            # #print 'sending: %s' % bcast_str
                                            # msgQueue.put((5,bcast_str))

                                            bcolour = str(r).zfill(
                                                3) + str(g).zfill(3) + str(b).zfill(3)
                                            #ledcolours = ['red','green','blue','cyan','magenta','yellow','white','off','on','invert','random']
                                            #bcolourname = "black"
                                            try:
                                                bcolourname = ledcolours[
                                                    ["255000000", "000255000", "000000255", "000255255", "255000255",
                                                     "255255000", "255255255"].index(bcolour)]
                                            except ValueError:
                                                pass
                                            # print "col lookup", bcolourname
                                            sensor_name = 'colour'
                                            bcast_str = 'sensor-update "%s" %s' % (
                                                sensor_name, bcolourname)
                                            # print 'sending: %s' % bcast_str
                                            msgQueue.put((5, bcast_str))

                            if self.bFindValue("bright"):
                                sghGC.ledDim = int(
                                    self.valueNumeric) if self.valueIsNumeric else 20
                                try:
                                    UH.brightness(
                                        max(0, min(1, float(float(sghGC.ledDim) / 100))))
                                    UH.show()
                                    sensor_name = 'bright'
                                    bcast_str = 'sensor-update "%s" %d' % (
                                        sensor_name, sghGC.ledDim)
                                    # print 'sending: %s' % bcast_str
                                    msgQueue.put((5, bcast_str))
                                except:
                                    pass

                            if self.bFindValue('matrixpattern'):
                                bit_pattern = (
                                    self.value + 'xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx')[
                                    0:64]
                                # print 'bit_pattern %s' % bit_pattern
                                for j in range(0, 64):
                                    ym = j // 8
                                    xm = j - (8 * ym)
                                    bp = bit_pattern[j]
                                    if bp in lettercolours:
                                        self.matrixRed, self.matrixGreen, self.matrixBlue = tcolours.get(
                                            ledcolours[
                                                lettercolours.index(bp)],
                                            (self.matrixRed, self.matrixGreen, self.matrixBlue))
                                        UH.set_pixel(
                                            xm, 7 - ym, self.matrixRed, self.matrixGreen, self.matrixBlue)
                                UH.show()
                            if "unicorn" in ADDON:
                                if self.bFind("moveleft"):
                                    for y in range(0, self.matrixRangemax):
                                        for x in range(0, self.matrixRangemax - 1):
                                            oldr, oldg, oldb = UH.get_pixel(
                                                x + 1, y)
                                            # print "oldpixel" , oldpixel
                                            UH.set_pixel(
                                                x, y, oldr, oldg, oldb)
                                        UH.set_pixel(7, y, 0, 0, 0)
                                    UH.show()

                                if self.bFind("moveright"):
                                    for y in range(0, self.matrixRangemax):
                                        for x in range(self.matrixRangemax - 1, 0, -1):
                                            # print "y,x",y,x
                                            oldr, oldg, oldb = UH.get_pixel(
                                                x - 1, y)
                                            # print "oldpixel" , oldpixel
                                            UH.set_pixel(
                                                x, y, oldr, oldg, oldb)
                                        UH.set_pixel(0, y, 0, 0, 0)
                                    UH.show()

                                if self.bFind("moveup"):
                                    for x in range(0, self.matrixRangemax):
                                        for y in range(0, self.matrixRangemax - 1):
                                            oldr, oldg, oldb = UH.get_pixel(
                                                x, y + 1)
                                            # print "oldpixel" , oldpixel
                                            UH.set_pixel(
                                                x, y, oldr, oldg, oldb)
                                        UH.set_pixel(x, 7, 0, 0, 0)
                                    UH.show()

                                if self.bFind("movedown"):
                                    for x in range(0, self.matrixRangemax):
                                        for y in range(self.matrixRangemax - 1, 0, -1):
                                            # print "y,x",y,x
                                            oldr, oldg, oldb = UH.get_pixel(
                                                x, y - 1)
                                            # print "oldpixel" , oldpixel
                                            UH.set_pixel(
                                                x, y, oldr, oldg, oldb)
                                        UH.set_pixel(x, 0, 0, 0, 0)
                                    UH.show()
                            else:
                                if self.bFindValue("shift"):
                                    if self.value != "down":
                                        for index in range(self.matrixUse, 0, - 1):
                                            oldr, oldg, oldb = UH.get_neopixel(
                                                index - 1)
                                            print "oldpixel", index, oldr, oldg, oldb
                                            UH.set_neopixel(
                                                index, oldr, oldg, oldb)
                                        #UH.set_neopixel(1, 0, 0, 0)
                                        UH.show()
                                if self.bFind("rotate"):
                                    lr, lg, lb = UH.get_neopixel(0)
                                    for index in range(0, self.matrixUse - 1):
                                        oldr, oldg, oldb = UH.get_neopixel(
                                            index + 1)
                                        # print "oldpixel" , oldpixel
                                        UH.set_neopixel(
                                            index, oldr, oldg, oldb)
                                    UH.set_neopixel(
                                        self.matrixUse - 1, lr, lg, lb)
                                    UH.show()

                            if self.bFind("invert"):
                                for index in range(0, self.matrixUse):
                                    oldr, oldg, oldb = UH.get_neopixel(index)
                                    # print "oldpixel" , oldpixel
                                    UH.set_neopixel(
                                        index, 255 - oldr, 255 - oldg, 255 - oldb)
                                UH.show()

                            if self.bFindValue("level"):
                                if self.valueIsNumeric:
                                    for index in range(0, self.matrixUse):
                                        oldr, oldg, oldb = tuple(
                                            [max(0, min(255, int(elim * (self.valueNumeric / 100.0)))) for elim in
                                             UH.get_neopixel(index)])
                                        # print "old" , oldr,oldg,oldb
                                        # print "oldpixel" , oldpixel
                                        UH.set_neopixel(
                                            index, oldr, oldg, oldb)
                                    UH.show()

                            if "unicorn" in ADDON:
                                rowList = [
                                    'a', 'b', 'c', 'd', 'e', 'f', 'g', 'h']
                                for i in range(0, 8):
                                    if self.bFindValue('row' + rowList[i]):
                                        bit_pattern = (
                                            self.value + "xxxxxxxx")[0:8]
                                        # print 'bit_pattern %s' % bit_pattern
                                        for j in range(0, 8):
                                            ym = i
                                            xm = j
                                            bp = bit_pattern[j]
                                            if bp in lettercolours:
                                                self.matrixRed, self.matrixGreen, self.matrixBlue = tcolours.get(
                                                    ledcolours[
                                                        lettercolours.index(bp)],
                                                    (self.matrixRed, self.matrixGreen, self.matrixBlue))
                                                UH.set_pixel(xm, 7 - ym, self.matrixRed, self.matrixGreen,
                                                             self.matrixBlue)
                                        UH.show()

                                for i in range(0, 8):
                                    if self.bFindValue('row' + str(i + 1)):
                                        bit_pattern = (
                                            self.value + "xxxxxxxx")[0:8]
                                        # print 'bit_pattern %s' % bit_pattern
                                        for j in range(0, 8):
                                            ym = i
                                            xm = j
                                            bp = bit_pattern[j]
                                            if bp in lettercolours:
                                                self.matrixRed, self.matrixGreen, self.matrixBlue = tcolours.get(
                                                    ledcolours[
                                                        lettercolours.index(bp)],
                                                    (self.matrixRed, self.matrixGreen, self.matrixBlue))
                                                UH.set_pixel(xm, 7 - ym, self.matrixRed, self.matrixGreen,
                                                             self.matrixBlue)
                                        UH.show()

                                colList = [
                                    'a', 'b', 'c', 'd', 'e', 'f', 'g', 'h']
                                for i in range(0, 8):
                                    if self.bFindValue('col' + colList[i]):
                                        # print self.value
                                        bit_pattern = (
                                            self.value + "xxxxxxxx")[0:8]
                                        for j in range(0, 8):
                                            ym = j
                                            xm = i
                                            bp = bit_pattern[j]
                                            if bp in lettercolours:
                                                self.matrixRed, self.matrixGreen, self.matrixBlue = tcolours.get(
                                                    ledcolours[
                                                        lettercolours.index(bp)],
                                                    (self.matrixRed, self.matrixGreen, self.matrixBlue))
                                                UH.set_pixel(xm, 7 - ym, self.matrixRed, self.matrixGreen,
                                                             self.matrixBlue)
                                        UH.show()

                                for i in range(0, 8):
                                    if self.bFindValue('col' + str(i + 1)):
                                        # print tcolours
                                        # print self.matrixRed,self.matrixGreen,self.matrixBlue
                                        # print self.value
                                        bit_pattern = (
                                            self.value + "xxxxxxxx")[0:8]
                                        for j in range(0, 8):
                                            ym = j
                                            xm = i
                                            bp = bit_pattern[j]
                                            if bp in lettercolours:
                                                self.matrixRed, self.matrixGreen, self.matrixBlue = tcolours.get(
                                                    ledcolours[
                                                        lettercolours.index(bp)],
                                                    (self.matrixRed, self.matrixGreen, self.matrixBlue))

                                                UH.set_pixel(xm, 7 - ym, self.matrixRed, self.matrixGreen,
                                                             self.matrixBlue)
                                        UH.show()

                                if self.bFindValue('loadimage'):
                                    try:
                                        sb = subprocess.Popen(
                                            ['convert', '-scale', '8x8!', '+matte', self.value, 'sghimage.bmp']).wait()
                                    except:
                                        pass
                                    # time.sleep(1)
                                    # When reading a binary file, always add a
                                    # 'b' to the file open mode
                                    with open('sghimage.bmp', 'rb') as f:
                                        # with open(self.value + '.bmp', 'rb') as f:
                                        # BMP files store their width and height statring at byte 18 (12h), so seek
                                        # to that position
                                        f.seek(10)

                                        # The width and height are 4 bytes
                                        # each, so read 8 bytes to get both of
                                        # them
                                        bytes = f.read(4)

                                        # Here, we decode the byte array from the last step. The width and height
                                        # are each unsigned, little endian, 4 byte integers, so they have the format
                                        # code '<II'. See
                                        # http://docs.python.org/3/library/struct.html
                                        # for more info
                                        bmpdata = int(
                                            struct.unpack('<I', bytes)[0])
                                        # print bmpdata

                                        # Print the width and height of the
                                        # image
                                        print(
                                            'Data starts at:  ' + str(bmpdata))
                                        f.seek(bmpdata)

                                        # move to start of pixel data
                                        bytes = f.read(192)
                                        # get 64 pixels * 3 for BGR
                                        pixel = struct.unpack('192B', bytes)
                                        # print "pixel",pixel
                                        for i in range(0, 64):
                                            UH.set_pixel(i % 8, 7 - (i // 8), pixel[(i * 3) + 2], pixel[(i * 3) + 1],
                                                         pixel[(i * 3) + 0])

                                        UH.show()

                                if self.bFindValue('saveimage'):
                                    # try:
                                    # sb = subprocess.Popen(['convert', '-scale', '8x8!', '+matte', self.value , 'sghimage.bmp']).wait()
                                    # except:
                                    # pass
                                    # time.sleep(1)
                                    # When reading a binary file, always add a
                                    # 'b' to the file open mode
                                    with open('sghimage.bmp', 'wb') as f:
                                        header = [0x42, 0x4D, 0xF6, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x36,
                                                  0x00, 0x00, 0x00, 0x28, 0x00, 0x00, 0x00, 0x08, 0x00, 0x00, 0x00,
                                                  0x08, 0x00, 0x00, 0x00, 0x01, 0x00, 0x18, 0x00, 0x00, 0x00, 0x00,
                                                  0x00, 0x00, 0x00, 0x00, 0x00, 0x13, 0x0B, 0x00, 0x00, 0x13, 0x0B,
                                                  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00]
                                        for i in header:
                                            f.write(chr(i))
                                        for i in range(0, 64):
                                            r, g, b = UH.get_pixel(
                                                i % 8, 7 - (i // 8))
                                            # print "rgb",r,g,b
                                            f.write(chr(b))
                                            f.write(chr(g))
                                            f.write(chr(r))
                                    sb = subprocess.Popen(
                                        ['cp', 'sghimage.bmp', self.value + '.bmp']).wait()

                                if self.bFindValue('load2image'):
                                    try:
                                        sb = subprocess.Popen(['convert', '-scale', '16x16!', '+matte', self.value,
                                                               'sghimage.bmp']).wait()
                                    except:
                                        pass
                                    # time.sleep(1)
                                    # When reading a binary file, always add a
                                    # 'b' to the file open mode
                                    with open('sghimage.bmp', 'rb') as f:
                                        # with open(self.value + '.bmp', 'rb') as f:
                                        # BMP files store their width and height statring at byte 18 (12h), so seek
                                        # to that position
                                        f.seek(10)

                                        # The width and height are 4 bytes
                                        # each, so read 8 bytes to get both of
                                        # them
                                        bytes = f.read(4)

                                        # Here, we decode the byte array from the last step. The width and height
                                        # are each unsigned, little endian, 4 byte integers, so they have the format
                                        # code '<II'. See
                                        # http://docs.python.org/3/library/struct.html
                                        # for more info
                                        bmpdata = int(
                                            struct.unpack('<I', bytes)[0])

                                        # Print the width and height of the
                                        # image
                                        print(
                                            'Data starts at:  ' + str(bmpdata))
                                        f.seek(bmpdata)

                                        # move to start of pixel data
                                        bytes = f.read(768)
                                        # get 64 pixels * 3 for BGR
                                        pixel = struct.unpack('768B', bytes)
                                        # print "pixel",pixel

                                        j = -18
                                        for i in range(0, 64):
                                            if i % 8 == 0:
                                                j += 18
                                            else:
                                                j += 2
                                            # print "i,j",i,j
                                            UH.set_pixel(i % 8, 7 - (i // 8), pixel[(j * 3) + 2], pixel[(j * 3) + 1],
                                                         pixel[(j * 3) + 0])
                                        UH.show()

                        self.dataraw = origdataraw

                        ADDON = oldADDON  # restore after possible parthat use

                    if "piandbash" in ADDON:
                        if self.bFindOnOff('all'):
                            mcp.output(8, self.OnOrOff)
                            mcp.output(12, self.OnOrOff)
                            mcp.output(10, self.OnOrOff)
                            for loop in range(0, 8):
                                if sghGC.piAndBash[loop] == sghGC.POUTPUT:
                                    mcp.output(loop, self.OnOrOff)

                        if self.bFindValue('clearlcd'):
                            if self.valueIsNumeric:
                                if self.valueNumeric == 1:
                                    pnblcd.lcd_byte(
                                        pnblcd.LCD_LINE_1, pnblcd.LCD_CMD)
                                    pnblcd.lcd_string('')
                                if self.valueNumeric == 2:
                                    pnblcd.lcd_byte(
                                        pnblcd.LCD_LINE_2, pnblcd.LCD_CMD)
                                    pnblcd.lcd_string('')
                            else:
                                pnblcd.lcd_byte(
                                    pnblcd.LCD_LINE_1, pnblcd.LCD_CMD)
                                pnblcd.lcd_string('')
                                pnblcd.lcd_byte(
                                    pnblcd.LCD_LINE_2, pnblcd.LCD_CMD)
                                pnblcd.lcd_string('')

                        if self.bFindOnOff('green'):
                            mcp.output(8, self.OnOrOff)
                        if self.bFindOnOff('red'):
                            mcp.output(12, self.OnOrOff)
                        if self.bFindOnOff('yellow'):
                            mcp.output(10, self.OnOrOff)
                        if self.bFindOnOff('amber'):
                            mcp.output(10, self.OnOrOff)

                        for loop in range(0, 8):
                            if self.bFindOnOff("digital" + str(loop)):
                                with lock:
                                    sghGC.piAndBash[loop] = sghGC.POUTPUT
                                    mcp.config(loop, mcp.OUTPUT)
                                mcp.output(loop, self.OnOrOff)

                        if self.bFindValue('line1'):
                            if self.value == 'clear':
                                self.value = ''
                            pnblcd.lcd_byte(pnblcd.LCD_LINE_1, pnblcd.LCD_CMD)
                            pnblcd.lcd_string(self.value)
                        if self.bFindValue('line2'):
                            if self.value == 'clear':
                                self.value = ''
                            pnblcd.lcd_byte(pnblcd.LCD_LINE_2, pnblcd.LCD_CMD)
                            pnblcd.lcd_string(self.value)

                    if "agobo" in ADDON:
                        if self.bFindOnOff('all'):
                            sghGC.pinUpdate(15, self.OnOrOff)
                            sghGC.pinUpdate(13, self.OnOrOff)

                        if self.bFindOnOff('leftled'):
                            sghGC.pinUpdate(15, self.OnOrOff)
                        if self.bFindOnOff('rightled'):
                            sghGC.pinUpdate(13, self.OnOrOff)

                    else:  # Plain GPIO Broadcast processing

                        # Check for all off/on type broadcasrs
                        self.bCheckAll()
                        # self.bPinCheck(sghGC.validPins) # Check for pin
                        # off/on type broadcasts

                        # check pins
                        for pin in sghGC.validPins:
                            if self.bFindOnOff('pin' + str(pin)):
                                sghGC.pinUpdate(pin, self.OnOrOff)
                            if self.bFindOnOff('gpio' + str(sghGC.gpioLookup[pin])):
                                sghGC.pinUpdate(pin, self.OnOrOff)
                            if self.bFindValue('power' + str(pin) + ","):
                                #logging.debug("bPowerPin:%s",pin )
                                if self.valueIsNumeric:
                                    sghGC.pinUpdate(
                                        pin, self.valueNumeric, type="pwm")
                                else:
                                    sghGC.pinUpdate(pin, 0, type="pwm")

                            if self.bFindValue('motor' + str(pin) + ","):
                                #logging.debug("bPowerPin:%s",pin )
                                if self.valueIsNumeric:
                                    sghGC.pinUpdate(
                                        pin, self.valueNumeric, type="pwmmotor")
                                else:
                                    sghGC.pinUpdate(pin, 0, type="pwmmotor")

                            if self.bFind('sonar' + str(pin)):
                                distance = sghGC.pinSonar(pin)
                                # print'Distance:',distance,'cm'
                                sensor_name = 'sonar' + str(pin)
                                bcast_str = 'sensor-update "%s" %d' % (
                                    sensor_name, distance)
                                # print 'sending: %s' % bcast_str
                                msgQueue.put((5, bcast_str))

                            if self.bFind('rctime' + str(pin)):
                                RCtime = sghGC.pinRCTime(pin)
                                # print'Distance:',distance,'cm'
                                sensor_name = 'RCtime' + str(pin)
                                bcast_str = 'sensor-update "%s" %d' % (
                                    sensor_name, RCtime)
                                # print 'sending: %s' % bcast_str
                                msgQueue.put((5, bcast_str))

                                # Start using ultrasonic sensor on a pin
                            if self.bFind('ultra' + str(pin)):
                                print 'start pinging on', str(pin)
                                self.startUltra(pin, 0, self.OnOrOff)

                                # end of normal pin checking

                    stepperList = [
                        ['positiona', [11, 12, 13, 15]], ['positionb', [16, 18, 22, 7]]]
                    for listLoop in range(0, 2):
                        #print ("loop" , listLoop)
                        if self.bFindValue(stepperList[listLoop][0]):
                            if self.valueIsNumeric:
                                self.stepperUpdate(
                                    stepperList[listLoop][1], 10, self.valueNumeric)
                            else:
                                self.stepperUpdate(stepperList[listLoop][1], 0)

                                # if steppersInUse == True:
                                # #logging.debug("Steppers in use")
                                # stepperList = [['motora',[11,12,13,15]],['motorb',[16,18,22,7]]]
                                # for listLoop in range(0,2):
                                # if self.vFindValue(stepperList[listLoop][0]):
                                # logging.debug("Stepper found %s",stepperList[listLoop][0])
                                # if self.valueIsNumeric:
                                # self.stepperUpdate(stepperList[listLoop][1],self.valueNumeric)
                                # else:
                                # self.stepperUpdate(stepperList[listLoop][1],0)

                                # stepperList = [['positiona',[11,12,13,15]],['positionb',[16,18,22,7]]]
                                # for listLoop in range(0,2):
                                # #print ("look for steppers")
                                # if self.vFindValue(stepperList[listLoop][0]):
                                # print ("Found stepper",stepperList[listLoop][0])
                                # if self.valueIsNumeric:
                                # print ("value =",self.value)
                                # print stepperList[listLoop][1][0]
                                # try:
                                # print ("Trying to see if turn prev set")
                                # direction = int(100 * sign(int(self.valueNumeric) - turn[stepperList[listLoop][1][0]]))
                                # steps = abs(int(self.valueNumeric) - turn[stepperList[listLoop][1][0]])
                                # except:
                                # direction = int(100 * sign(int(self.valueNumeric)))
                                # steps = abs(int(self.valueNumeric))
                                # turn = [None] * sghGC.numOfPins
                                # pass
                                # print ("direction and steps",direction,steps)
                                # self.stepperUpdate(stepperList[listLoop][1],direction,steps)
                                # turn[stepperList[listLoop][1][0]] = self.valueNumeric
                                # print ("position set to :",turn[stepperList[listLoop][1][0]])
                                # else:
                                # self.stepperUpdate(stepperList[listLoop][1],0)
                                # try:
                                # turn[stepperList[listLoop][1][0]] = 0
                                # except:
                                # turn = [None] * sghGC.numOfPins
                                # turn[stepperList[listLoop][1][0]] = 0
                                # pass
                                # else:
                                # motorList = [['motora',11],['motorb',12]]
                                # for listLoop in range(0,2):
                                # if self.vFindValue(motorList[listLoop][0]):
                                # if self.valueIsNumeric:
                                # sghGC.pinUpdate(motorList[listLoop][1],self.valueNumeric,type="pwm")
                                # else:
                                # sghGC.pinUpdate(motorList[listLoop][1],0,type="pwm")

                    if self.bFindValue('pinpattern'):
                        if "pitt" in ADDON:
                            sensor_value = self.value
                            bit_pattern = ('00000000' + self.value)[-8:]
                            print 'bit_pattern %s' % bit_pattern
                            j = 0
                            for pin in [8, 10, 12, 16, 18, 22, 24, 26]:
                                # print "pin" , bit_pattern[-(j+1)]
                                if bit_pattern[-(j + 1)] == '0':
                                    # output inverted as board is active low
                                    sghGC.pinUpdate(pin, 1)
                                else:
                                    sghGC.pinUpdate(pin, 0)
                                j += 1
                        else:
                            # print 'Found pinpattern broadcast'
                            # print dataraw
                            #num_of_bits = PINS
                            outputall_pos = self.dataraw.find('pinpattern')
                            sensor_value = self.dataraw[
                                (outputall_pos + 10):].split()
                            # print sensor_value
                            #sensor_value[0] = sensor_value[0][:-1]
                            # print sensor_value[0]
                            bit_pattern = (
                                '00000000000000000000000000' + sensor_value[0])[-sghGC.numOfPins:]
                            # print 'bit_pattern %s' % bit_pattern
                            j = 0
                            for pin in sghGC.validPins:
                                if (sghGC.pinUse[pin] == sghGC.POUTPUT):
                                    # print "pin" , bit_pattern[-(j+1)]
                                    if bit_pattern[-(j + 1)] == '0':
                                        sghGC.pinUpdate(pin, 0)
                                    else:
                                        sghGC.pinUpdate(pin, 1)
                                    j += 1

                    if pcfSensor is not None:  # if PCF ADC found
                        for channel in range(1, 5):  # loop thru all 4 inputs
                            if self.bFind('adc' + str(channel)):
                                # get each value
                                adc = pcfSensor.readADC(channel - 1)
                                # print'Distance:',distance,'cm'
                                sensor_name = 'adc' + str(channel)
                                bcast_str = 'sensor-update "%s" %d' % (
                                    sensor_name, adc)
                                # print 'sending: %s' % bcast_str
                                msgQueue.put((5, bcast_str))

                    if ((piglow is not None) and ("piglow" not in ADDON)):
                        # print "processing piglow variables"

                        if self.bFindOnOff('all'):
                            # print "found allon/off"
                            for i in range(1, 19):
                                # print i
                                PiGlow_Values[
                                    i - 1] = PiGlow_Brightness * self.OnOrOff
                                # print "Values", PiGlow_Values
                                piglow.update_pwm_values(PiGlow_Values)

                        # check LEDS
                        for i in range(1, 19):
                            #check_broadcast = str(i) + 'on'
                            # print check_broadcast
                            if self.bFindOnOff('pgled' + str(i)):
                                # print dataraw
                                PiGlow_Values[
                                    PiGlow_Lookup[i - 1]] = PiGlow_Brightness * self.OnOrOff
                                piglow.update_pwm_values(PiGlow_Values)

                            if self.bFindOnOff('pglight' + str(i)):
                                # print dataraw
                                PiGlow_Values[
                                    PiGlow_Lookup[i - 1]] = PiGlow_Brightness * self.OnOrOff
                                piglow.update_pwm_values(PiGlow_Values)

                        pcolours = [
                            'pgred', 'pgorange', 'pgyellow', 'pggreen', 'pgblue', 'pgwhite']
                        for i in range(len(pcolours)):
                            if self.bFindOnOff(pcolours[i]):
                                # print dataraw
                                PiGlow_Values[
                                    PiGlow_Lookup[i + 0]] = PiGlow_Brightness * self.OnOrOff
                                PiGlow_Values[
                                    PiGlow_Lookup[i + 6]] = PiGlow_Brightness * self.OnOrOff
                                PiGlow_Values[
                                    PiGlow_Lookup[i + 12]] = PiGlow_Brightness * self.OnOrOff
                                piglow.update_pwm_values(PiGlow_Values)

                        for i in range(1, 4):
                            if self.bFindOnOff('pgleg' + str(i)):
                                # print dataraw
                                PiGlow_Values[
                                    PiGlow_Lookup[((i - 1) * 6) + 0]] = PiGlow_Brightness * self.OnOrOff
                                PiGlow_Values[
                                    PiGlow_Lookup[((i - 1) * 6) + 1]] = PiGlow_Brightness * self.OnOrOff
                                PiGlow_Values[
                                    PiGlow_Lookup[((i - 1) * 6) + 2]] = PiGlow_Brightness * self.OnOrOff
                                PiGlow_Values[
                                    PiGlow_Lookup[((i - 1) * 6) + 3]] = PiGlow_Brightness * self.OnOrOff
                                PiGlow_Values[
                                    PiGlow_Lookup[((i - 1) * 6) + 4]] = PiGlow_Brightness * self.OnOrOff
                                PiGlow_Values[
                                    PiGlow_Lookup[((i - 1) * 6) + 5]] = PiGlow_Brightness * self.OnOrOff
                                piglow.update_pwm_values(PiGlow_Values)

                            if self.bFindOnOff('pgarm' + str(i)):
                                # print dataraw
                                PiGlow_Values[
                                    PiGlow_Lookup[((i - 1) * 6) + 0]] = PiGlow_Brightness * self.OnOrOff
                                PiGlow_Values[
                                    PiGlow_Lookup[((i - 1) * 6) + 1]] = PiGlow_Brightness * self.OnOrOff
                                PiGlow_Values[
                                    PiGlow_Lookup[((i - 1) * 6) + 2]] = PiGlow_Brightness * self.OnOrOff
                                PiGlow_Values[
                                    PiGlow_Lookup[((i - 1) * 6) + 3]] = PiGlow_Brightness * self.OnOrOff
                                PiGlow_Values[
                                    PiGlow_Lookup[((i - 1) * 6) + 4]] = PiGlow_Brightness * self.OnOrOff
                                PiGlow_Values[
                                    PiGlow_Lookup[((i - 1) * 6) + 5]] = PiGlow_Brightness * self.OnOrOff
                                piglow.update_pwm_values(PiGlow_Values)

                    origdataraw = self.dataraw

                    if AdaMatrix is not None:  # Matrix connected
                        # print self.dataraw
                        # print
                        self.dataraw = self.dataraw[
                            self.dataraw.find("broadcast") + 10:]
                        # print self.dataraw
                        # print
                        # print self.dataraw.split('broadcast')
                        broadcastList = self.dataraw.split(' ')
                        for broadcastListLoop in broadcastList:
                            self.dataraw = " " + str(broadcastListLoop)
                            # print "matrix looping tthru",self.dataraw
                            if self.bFind("alloff"):
                                AdaMatrix.clear()
                            if self.bFind("sweep"):
                                # print "sweep found"
                                for y in range(0, 8):
                                    for x in range(0, 8):
                                        AdaMatrix.setPixel((x), y)
                                        time.sleep(0.01)
                                for y in range(0, 8):
                                    for x in range(0, 8):
                                        AdaMatrix.setPixel((x), y, 2)
                                        time.sleep(0.01)
                                for y in range(0, 8):
                                    for x in range(0, 8):
                                        AdaMatrix.setPixel((x), y, 3)
                                time.sleep(0.01)

                            self.matrixMult = 1
                            self.matrixLimit = 1
                            if self.matrixUse == 4:
                                self.matrixMult = 4
                                self.matrixLimit = 4
                            if self.matrixUse == 9:
                                self.matrixMult = 3
                                self.matrixLimit = 2
                            if self.matrixUse == 16:
                                self.matrixMult = 2
                                self.matrixLimit = 2

                            colours = ["off", "green", "red", "yellow"]

                            for led in range(0, self.matrixUse):
                                # if self.bFind("led"+str(led + 1)+"on"):
                                # ym = int(led / math.sqrt(self.matrixUse))
                                # xm = led - int((math.sqrt(self.matrixUse) * ym))
                                # for yy in range(0,self.matrixLimit):
                                # for xx in range(0,self.matrixLimit):
                                # AdaMatrix.setPixel((7 - (xm * self.matrixMult)-xx),(ym * self.matrixMult)+yy,1)

                                if self.bFind("led" + str(led + 1)):
                                    for colour in colours:
                                        if self.bFind("led" + str(led + 1) + colour):
                                            ym = int(
                                                led / math.sqrt(self.matrixUse))
                                            xm = led - \
                                                int((
                                                    math.sqrt(self.matrixUse) * ym))

                                            print colour
                                            print "led found", xm, ym
                                            for yy in range(0, self.matrixLimit):
                                                for xx in range(0, self.matrixLimit):
                                                    AdaMatrix.setPixel(((xm * self.matrixMult) + xx),
                                                                       (ym * self.matrixMult) +
                                                                       yy,
                                                                       colours.index(colour))

                            for colour in range(0, 3):
                                if self.bFindValue(["green", "red", "yellow"][colour] + "on"):
                                    if self.value == "":
                                        xm = self.matrixX
                                        ym = self.matrixY
                                        # print xm,ym
                                        for yy in range(0, self.matrixLimit):
                                            for xx in range(0, self.matrixLimit):
                                                AdaMatrix.setPixel(((xm * self.matrixMult) + xx),
                                                                   (ym * self.matrixMult) + yy, colour + 1)

                            for ym in range(0, 8):
                                for xm in range(0, 8):

                                    if self.bFind("matrixon" + str(xm) + "x" + str(ym) + "y"):
                                        AdaMatrix.setPixel((xm), ym)

                                    if self.bFind("matrixoff" + str(xm) + "x" + str(ym) + "y"):
                                        AdaMatrix.clearPixel((xm), ym)

                            # ["green","red","yellow"]:
                            for colour in range(0, 3):
                                if self.bFindValue(["green", "red", "yellow"][colour] + "on"):
                                    if self.value != "":
                                        xm = 0
                                        ym = 0

                                        # print "gon found"
                                        # print self.value
                                        if len(self.value) == 4:
                                            xm = int(float(self.value[1]))
                                            ym = int(float(self.value[3]))
                                        if len(self.value) == 2:
                                            xm = int(float(self.value[0]))
                                            ym = int(float(self.value[1]))
                                        for yy in range(0, self.matrixLimit):
                                            for xx in range(0, self.matrixLimit):
                                                AdaMatrix.setPixel(((xm * self.matrixMult) + xx),
                                                                   (ym * self.matrixMult) + yy, colour + 1)

                            if self.bFindValue("brightness"):
                                if self.valueIsNumeric:
                                    AdaMatrix.setBrightness(
                                        max(0, min(15, self.valueNumeric)))
                                else:
                                    AdaMatrix.setBrightness(15)

                            if self.bFindValue('matrixpattern'):
                                bit_pattern = (
                                    self.value + '00000000000000000000000000000000000000000000000000000000000000000')[
                                    0:64]
                                # print 'bit_pattern %s' % bit_pattern
                                for j in range(0, 64):
                                    ym = j // 8
                                    xm = j - (8 * ym)
                                    if bit_pattern[j] == '0':
                                        AdaMatrix.clearPixel((xm), ym)
                                    else:
                                        AdaMatrix.setPixel((xm), ym)
                                    j += 1

                            rowList = ['a', 'b', 'c', 'd', 'e', 'f', 'g', 'h']
                            for i in range(0, 8):
                                if self.bFindValue('row' + rowList[i]):
                                    bit_pattern = (
                                        self.value + "00000000")[0:8]
                                    # print 'bit_pattern %s' % bit_pattern
                                    for j in range(0, 8):
                                        ym = i
                                        xm = j
                                        if bit_pattern[(j)] == '0':
                                            AdaMatrix.clearPixel((xm), ym)
                                        else:
                                            AdaMatrix.setPixel((xm), ym, 1)

                            colList = ['a', 'b', 'c', 'd', 'e', 'f', 'g', 'h']
                            for i in range(0, 8):
                                if self.bFindValue('col' + rowList[i]):
                                    # print self.value
                                    bit_pattern = (
                                        self.value + "00000000")[0:8]
                                    for j in range(0, 8):
                                        ym = j
                                        xm = i

                                        if bit_pattern[(j)] == '0':
                                            AdaMatrix.clearPixel((xm), ym)
                                        else:
                                            AdaMatrix.setPixel((xm), ym, 1)
                                            # time.sleep(2)

                            if self.bFindValue('scrollleft'):
                                AdaMatrix.scroll("left")
                            if self.bFindValue('scrollright'):
                                print "scrollr"
                                AdaMatrix.scroll("right")

                            if self.bFindValue("getmatrix"):
                                print "gm found"
                                if len(self.value) == 4:
                                    xm = int(float(self.value[1]))
                                    ym = int(float(self.value[3]))
                                if len(self.value) == 3:
                                    xm = int(float(self.value[0]))
                                    ym = int(float(self.value[2]))
                                if len(self.value) == 2:
                                    xm = int(float(self.value[0]))
                                    ym = int(float(self.value[1]))
                                mValue = AdaMatrix.getPixel(
                                    xm, ym)  # get  value
                                # print'Distance:',distance,'cm'
                                sensor_name = 'matrixvalue'
                                bcast_str = 'sensor-update "%s" %d' % (
                                    sensor_name, mValue)
                                # print 'sending: %s' % bcast_str
                                msgQueue.put((5, bcast_str))

                    self.dataraw = origdataraw  # restore oringal sell.dataraw
                    if PiMatrix is not None:  # Matrix connected
                        # print self.dataraw
                        # print
                        self.dataraw = self.dataraw[self.dataraw.find(
                            "broadcast") + 10:]  # split dataraw so that operations are sequential
                        # print self.dataraw
                        # print
                        # print self.dataraw.split('broadcast')
                        broadcastList = self.dataraw.split(' ')
                        for broadcastListLoop in broadcastList:
                            self.dataraw = " " + str(broadcastListLoop)
                            # print self.dataraw

                            if self.bFindOnOff("all"):
                                PiMatrix.clear(self.OnOrOff)

                            if self.bFindOnOff("sweep"):
                                for y in range(0, 8):
                                    for x in range(0, 8):
                                        PiMatrix.setPixel(x, y, self.OnOrOff)
                                        time.sleep(0.05)

                            if self.bFindValue("matrixo"):
                                for ym in range(0, 8):
                                    for xm in range(0, 8):
                                        if self.bFindValue("matrixonx" + str(xm) + "y" + str(ym)):
                                            PiMatrix.setPixel(xm, ym)
                                        if self.bFindValue("matrixoffx" + str(xm) + "y" + str(ym)):
                                            PiMatrix.clearPixel(xm, ym)

                                            # if self.bFindValue("brightness"):
                                            # if self.valueIsNumeric:
                                            # PiMatrix.setBrightness(max(0,min(15,self.valueNumeric)))
                                            # else:
                                            # PiMatrix.setBrightness(15)

                            if self.bFindValue('matrixpattern'):
                                bit_pattern = (
                                    self.value + '00000000000000000000000000000000000000000000000000000000000000000')[
                                    0:64]
                                # print 'bit_pattern %s' % bit_pattern
                                for j in range(0, 64):
                                    ym = j // 8
                                    xm = j - (8 * ym)
                                    PiMatrix.setPixel(
                                        xm, ym, int(float(bit_pattern[j])))
                                    j += 1

                            rowList = ['a', 'b', 'c', 'd', 'e', 'f', 'g', 'h']
                            for i in range(0, 8):
                                if self.bFindValue('row' + rowList[i]):
                                    bit_pattern = (
                                        self.value + "00000000")[0:8]
                                    # print 'bit_pattern %s' % bit_pattern
                                    for j in range(0, 8):
                                        ym = i
                                        xm = j
                                        PiMatrix.setPixel(
                                            xm, ym, int(float(bit_pattern[j])))

                            colList = ['a', 'b', 'c', 'd', 'e', 'f', 'g', 'h']
                            for i in range(0, 8):
                                if self.bFindValue('col' + rowList[i]):
                                    # print self.value
                                    bit_pattern = (
                                        self.value + "00000000")[0:8]
                                    for j in range(0, 8):
                                        ym = j
                                        xm = i
                                        PiMatrix.setPixel(
                                            xm, ym, int(float(bit_pattern[j])))

                            if self.bFindValue('scrollleft'):
                                PiMatrix.scroll("left")
                            if self.bFindValue('scrollright'):
                                PiMatrix.scroll("right")

                    self.dataraw = origdataraw

                    if self.bFind('gettime'):
                        now = dt.datetime.now()

                        TimeAndDate = now.strftime('%c')
                        # print "tandD" , TimeAndDate
                        sensor_name = 'timeanddate'
                        bcast_str = 'sensor-update "%s" "''%s''"' % (
                            sensor_name, TimeAndDate)
                        # print 'sending: %s' % bcast_str
                        # msgQueue.put((5,bcast_str))
                        msgQueue.put((5, bcast_str))

                        fulldatetime = now.strftime('%Y%m%d%H%M%S')
                        sensor_name = 'fulldatetime'
                        bcast_str = 'sensor-update "%s" %s' % (
                            sensor_name, fulldatetime)
                        # print 'sending: %s' % bcast_str
                        msgQueue.put((5, bcast_str))
                        hrs = fulldatetime[-6:-4]
                        sensor_name = 'hours'
                        bcast_str = 'sensor-update "%s" "''%s''"' % (
                            sensor_name, hrs)
                        # print 'sending: %s' % bcast_str
                        msgQueue.put((5, bcast_str))
                        minutes = fulldatetime[-4:-2]
                        sensor_name = 'minutes'
                        bcast_str = 'sensor-update "%s" "''%s''"' % (
                            sensor_name, minutes)
                        # print 'sending: %s' % bcast_str
                        msgQueue.put((5, bcast_str))

                    if self.bFind("readcount"):  # update pin count values
                        for pin in sghGC.validPins:  # loop thru all pins
                            if sghGC.pinUse[pin] == sghGC.PCOUNT:
                                if self.bFind('readcount' + str(pin)):
                                    #print ('readcount'+str(pin))
                                    #print (sghGC.pinCount[pin])
                                    # print'Distance:',distance,'cm'
                                    sensor_name = 'count' + str(pin)
                                    bcast_str = 'sensor-update "%s" %d' % (
                                        sensor_name, sghGC.pinCount[pin])
                                    # print 'sending: %s' % bcast_str
                                    msgQueue.put((5, bcast_str))

                    if self.bFind("resetcount"):  # update pin count values
                        for pin in sghGC.validPins:  # loop thru all pins
                            if sghGC.pinUse[pin] == sghGC.PCOUNT:
                                if self.bFind('resetcount' + str(pin)):
                                    sghGC.pinCount[pin] = 0
                        print "diff reset"
                        self.encoderDiff = 0

                    if self.bFind("getip"):  # find ip address
                        logging.debug("Finding IP")
                        arg = 'ip route list'
                        p = subprocess.Popen(
                            arg, shell=True, stdout=subprocess.PIPE)
                        ipdata = p.communicate()
                        split_data = ipdata[0].split()
                        ipaddr = split_data[split_data.index('src') + 1]
                        logging.debug("IP:%s", ipaddr)
                        sensor_name = 'ipaddress'
                        bcast_str = 'sensor-update "%s" %s' % (
                            sensor_name, "ip" + ipaddr)
                        msgQueue.put((5, bcast_str))

                    if self.bFind("gettemp"):  # find temp address
                        if sghGC.dsSensorId == "":
                            print "checking for DS18B"
                            sghGC.findDS180()
                            time.sleep(1)
                        if sghGC.dsSensorId != "":
                            print "ds:", sghGC.dsSensorId
                            print "getting temp"
                            # sghGC.dsSensorId)
                            temperature = sghGC.getDS180Temp()
                            sensor_name = 'temperature'
                            bcast_str = 'sensor-update "%s" %s' % (
                                sensor_name, str(temperature))
                            msgQueue.put((5, bcast_str))

                    if self.bFind("getcputemp"):  # find cputemp
                        logging.debug("Finding CPUTemp")
                        cmd = '/opt/vc/bin/vcgencmd measure_temp'
                        line = os.popen(cmd).readline().strip()
                        temp = line.split('=')[1].split("'")[0]
                        #arg = 'ip route list'
                        # p=subprocess.Popen(arg,shell=True,stdout=subprocess.PIPE)
                        #ipdata = p.communicate()
                        #split_data = ipdata[0].split()
                        #ipaddr = split_data[split_data.index('src')+1]
                        #logging.debug("IP:%s", ipaddr)
                        sensor_name = 'cputemp'
                        bcast_str = 'sensor-update "%s" %s' % (
                            sensor_name, temp)
                        msgQueue.put((5, bcast_str))

                    if self.bFindValue('savedata'):
                        with open('data.txt', 'w') as f:
                            f.write(self.value)
                            print "data saved"

                    if self.bFind("pidisp"):  # display IP
                        print "PiDisp"
                        try:
                            os.system("sudo python ipd03.py")
                        except:
                            print "error from PiDisp"
                            pass

                    if self.bFind('photo'):
                        RasPiCamera.take_photo()

                        # if self.bFindValue('displayphoto'):
                        # os.environ['SDL_VIDEO_WINDOW_POS'] = "%d,%d" % (600,100)
                        # pygame.init()
                        # screen = pygame.display.set_mode((320, 240))
                        # search_dir = "/home/pi/photos/"
                        # os.chdir(search_dir)
                        # files = filter(os.path.isfile, os.listdir(search_dir))
                        # files = [os.path.join(search_dir, f) for f in files] # add path to each file
                        # files.sort(key=lambda x: os.path.getmtime(x))
                        # print files
                        # #os.system('gpicview '+ files[-1])
                        # image1 = pygame.image.load(files[-1])#"/home/pi/photos/0.jpg")
                        # image2 = pygame.transform.scale(image1, (320,240))
                        # screen.fill((255,255,255))
                        # screen.blit(image2,(0,0))
                        # pygame.display.flip()
                        # time.sleep(3)
                        # pygame.display.quit()

                    if self.bFindValue('minecraft'):
                        if self.value == "start":
                            mc = minecraft.Minecraft.create()
                            #mc.setBlocks(-100, 0, -100, 100, 63, 100, 0, 0)
                            #mc.setBlocks(-100, -63, -100, 100, -2, 100, 1, 0)
                            #mc.setBlocks(-100, -1, -100, 100, -1, 100, 2, 0)
                            #mc.player.setPos(0, 0, 0)
                            # mc.camera.setFixed()
                            # mc.camera.setFollow(1)
                            # mc.camera.setPos(0,0,0)
                            mc.postToChat(
                                "ScratchGPIO connected to Minecraft Pi.")

                        if self.value == "camnormal":
                            entityId = mc.getPlayerEntityIds()
                            mc.camera.setNormal(entityId)
                            mc.postToChat("camnormal")

                        if self.value == "camfollow":
                            entityId = mc.getPlayerEntityIds()
                            mc.camera.setFollow(entityId)
                            mc.postToChat("camfollow")

                        if self.value == "camfixed":
                            #entityId = mc.getPlayerEntityIds()
                            mc.camera.setFixed()
                            mc.postToChat("camfixed")

                        if self.value == "move":
                            x, y, z = mc.player.getTilePos()

                            print "old pos", x, y, z
                            print "moving to", sghMC.getxPos(), sghMC.getyPos(), sghMC.getzPos()

                            mc.player.setTilePos(
                                sghMC.getxPos(), sghMC.getyPos(), sghMC.getzPos())
                            mc.postToChat("moved")

                        if self.value == "cammove":
                            x, y, z = mc.player.getTilePos()
                            mc.camera.setPos(
                                sghMC.getxPos(), sghMC.getyPos(), sghMC.getzPos())
                            mc.postToChat("cammoved")

                        if self.value == "setblock":
                            mc.setBlock(
                                sghMC.getxPos(), sghMC.getyPos(), sghMC.getzPos(), 1)

                        if self.value == "clearblock":
                            mc.setBlock(
                                sghMC.getxPos(), sghMC.getyPos(), sghMC.getzPos(), 0)

                        if self.value == "getpos":
                            x, y, z = mc.player.getTilePos()
                            print "pos", x, y, z
                            mc.postToChat(str(x) + " " + str(y) + " " + str(z))
                            sensor_name = 'playerx'
                            bcast_str = 'sensor-update "%s" %s' % (
                                sensor_name, str(x))
                            msgQueue.put((5, bcast_str))
                            sensor_name = 'playery'
                            bcast_str = 'sensor-update "%s" %s' % (
                                sensor_name, str(y))
                            msgQueue.put((5, bcast_str))
                            sensor_name = 'playerz'
                            bcast_str = 'sensor-update "%s" %s' % (
                                sensor_name, str(z))
                            msgQueue.put((5, bcast_str))

                        if self.value == "getblock":
                            blockType = mc.getBlock(
                                sghMC.getxPos(), sghMC.getyPos(), sghMC.getzPos())
                            sensor_name = 'blocktype'
                            bcast_str = 'sensor-update "%s" %s' % (
                                sensor_name, str(blockType))
                            msgQueue.put((5, bcast_str))

                        if self.value == "movex-":
                            x, y, z = mc.player.getTilePos()
                            print x, y, z
                            mc.player.setTilePos(x + 1, y, z)
                            mc.postToChat("moved")
                        if self.value == "movex+":
                            x, y, z = mc.player.getTilePos()
                            print x, y, z
                            mc.player.setTilePos(x - 1, y, z)
                            mc.postToChat("moved")
                        if self.value == "movez-":
                            x, y, z = mc.player.getTilePos()
                            print x, y, z
                            mc.player.setTilePos(x, y, z + 1)
                            mc.postToChat("moved")
                        if self.value == "movez+":
                            x, y, z = mc.player.getTilePos()
                            print x, y, z
                            mc.player.setTilePos(x, y, z - 1)
                            mc.postToChat("moved")
                        if self.value == "movey-":
                            x, y, z = mc.player.getTilePos()
                            print x, y, z
                            mc.player.setTilePos(x, y + 1, z)
                            mc.postToChat("moved")
                        if self.value == "movey+":
                            x, y, z = mc.player.getTilePos()
                            print x, y, z
                            mc.player.setTilePos(x, y - 1, z)
                            mc.postToChat("moved")

                    if self.bFindValue('link'):
                        try:
                            self.scratch_socket2 = socket.socket(
                                socket.AF_INET, socket.SOCK_STREAM)
                            self.scratch_socket2.connect((self.value, 42001))
                            print self.scratch_socket2
                            print "Connected to ", self.value
                        except:
                            print "Failed to connect to ", self.value
                            pass

                    if self.bFindValue('send'):
                        if self.scratch_socket2 is not None:
                            print self.dataraw
                            print self.dataraw.count('send')
                            # print [match.start() for match in
                            # re.finditer(re.escape('send'), self.dataraw)]
                            totalcmd = ''
                            for qwe in self.dataraw.split(" "):
                                # print qwe[0:4]
                                if qwe[0:4] == 'send':
                                    # print qwe
                                    #cmd = qwe[4:]
                                    cmd = 'broadcast "' + qwe[4:] + '"'
                                    # print "sneding:",cmd
                                    n = len(cmd)
                                    b = (chr((n >> 24) & 0xFF)) + (chr((n >> 16) & 0xFF)) + (chr((n >> 8) & 0xFF)) + (
                                        chr(n & 0xFF))
                                    totalcmd = totalcmd + b + cmd
                            # print "Sending to Alt:",totalcmd
                            self.scratch_socket2.send(totalcmd)

                    if self.bFindValue('connect'):
                        cycle_trace = 'disconnected'
                        host = self.value
                        print "cycle_trace has changed to", cycle_trace
                        break

                    if '1coil' in dataraw:
                        print "1coil broadcast"
                        stepType = 0
                        print "step mode", stepMode[stepType]
                        step_delay = 0.0025

                    if '2coil' in dataraw:
                        print "2coil broadcast"
                        stepType = 1
                        print "step mode", stepMode[stepType]
                        step_delay = 0.0025

                    if 'halfstep' in dataraw:
                        print "halfstep broadcast"
                        stepType = 2
                        print "step mode", stepMode[stepType]
                        step_delay = 0.0013

                    if "version" in dataraw:
                        bcast_str = 'sensor-update "%s" %s' % (
                            "Version", Version)
                        # print 'sending: %s' % bcast_str
                        msgQueue.put((5, bcast_str))

                    if self.bFindValue('ultradelay'):
                        sghGC.ultraFreq = self.valueNumeric if self.valueIsNumeric else 1

                    if self.bFindValue("getir"):
                        # print "ir found"

                        value = 0b11111 & MCP23008.readU8(0x09)  # get  val
                        sensor_name = 'irsensor'
                        bcast_str = 'sensor-update "%s" %d' % (
                            sensor_name, value)
                        # print 'sending: %s' % bcast_str
                        msgQueue.put((5, bcast_str))
                        for led in range(0, 5):
                            sensor_name = 'irsensor' + str(led)
                            bcast_str = 'sensor-update "%s" %d' % (
                                sensor_name, int(value & 2 ** led) >> led)
                            # print 'sending: %s' % bcast_str
                            msgQueue.put((5, bcast_str))

                    if self.bFind("setwait"):
                        print "wait"
                        bcast_str = 'sensor-update "%s" %s' % (
                            "carryon", "false")
                        msgQueue.put((1, bcast_str))
                        self.carryOn = False
                        self.carryOnInUse = True

                    if self.bFindValue("getcheerlights"):
                        # print self.value
                        lookupColour = min(
                            10, max(1, int(self.valueNumeric))) if self.valueIsNumeric else 1
                        # print(lookupColour)
                        if (cheerList is None) or (lookupColour == 1):
                            #print("Fetching colour from internet")
                            try:
                                cheerList = cheerlights.get_colours()
                            except:
                                print "cheerlight error"
                                cheerList = ['white']
                                pass
                        # print cheerList
                        cheerColour = cheerList[lookupColour - 1]
                        print "new colour", cheerColour
                        bcast_str = 'sensor-update "%s" %s' % (
                            "cheerlights", cheerColour)
                        msgQueue.put((5, bcast_str))
                        if self.carryOnInUse == True:
                            bcast_str = 'sensor-update "%s" %s' % (
                                "carryon", "true")
                            msgQueue.put((1, bcast_str))
                        # print "data valid", time.time()

                    if "playhat" in ADDON:

                        if self.bFindOnOff('buzzer'):
                            sghGC.pinUpdate(16, self.OnOrOff)

                    if "explorerhat" in ADDON:
                        if self.bFindOnOff("green"):
                            # print "red:",self.OnOrOff
                            sghGC.pinUpdate(7, self.OnOrOff)
                        if self.bFindOnOff("red"):
                            # print "red:",self.OnOrOff
                            sghGC.pinUpdate(11, self.OnOrOff)
                        if self.bFindOnOff("yellow"):
                            # print "red:",self.OnOrOff
                            sghGC.pinUpdate(13, self.OnOrOff)
                        if self.bFindOnOff("blue"):
                            # print "red:",self.OnOrOff
                            sghGC.pinUpdate(29, self.OnOrOff)

                        if self.bFindOnOff("output1"):
                            # print "red:",self.OnOrOff
                            sghGC.pinUpdate(31, self.OnOrOff)
                        if self.bFindOnOff("output2"):
                            # print "red:",self.OnOrOff
                            sghGC.pinUpdate(32, self.OnOrOff)
                        if self.bFindOnOff("output3"):
                            # print "red:",self.OnOrOff
                            sghGC.pinUpdate(33, self.OnOrOff)
                        if self.bFindOnOff("output4"):
                            # print "red:",self.OnOrOff
                            sghGC.pinUpdate(36, self.OnOrOff)

                    # end of broadcast check

                    if self.bFind('shutdownpi'):
                        os.system('sudo shutdown -h "now"')

                    # print "encoderinUse state" ,sghGC.encoderInUse
                    if sghGC.encoderInUse == 0:
                        # inform Scratch that turning is finished
                        msgQueue.put((5, 'sensor-update "encoder" "stopped"'))

                if 'stop handler' in dataraw:
                    print "stop handler msg setn from Scratch"
                    cleanup_threads((listener, sender))
                    sys.exit()
                if self.carryOnInUse == True:
                    bcast_str = 'sensor-update "%s" %s' % ("carryon", "false")
                    msgQueue.put((2, bcast_str))

                self.carryOn = False

        print "Listener Stopped"
        # else:
        # print 'received something: %s' % dataraw


# End of  ScratchListner Class


# Messages to Scratch using a Queue
class SendMsgsToScratch(threading.Thread):

    def __init__(self, socket, msgQueue):
        threading.Thread.__init__(self)
        self.msgQueue = msgQueue
        self.scratch_socket = socket
        self.scratch_socket2 = None
        self._stop = threading.Event()
        print "Send Msgs Init"

    def stop(self):
        self._stop.set()
        print "SendMsgsToScratch Stop Set"

    def stopped(self):
        return self._stop.isSet()

    def run(self):
        print "msgs runnning"
        while not self.stopped():
            # print self.msgQueue.get()
            priority, cmd = self.msgQueue.get()
            if cmd == "STOPSENDING":
                print "STOPSENDING msg retreived from queue"
                self.stop()
                while not self.msgQueue.empty():
                    dummy = self.msgQueue.get()
                break
            # if priority == 1:
            #     print "deque P1 at", time.time()
            n = len(cmd)
            b = (chr((n >> 24) & 0xFF)) + (chr((n >> 16) & 0xFF)) + \
                (chr((n >> 8) & 0xFF)) + (chr(n & 0xFF))
            try:
                self.scratch_socket.send(b + cmd)
            except:
                print "failed to send this message to Scratch", cmd
                pass
            # print "message sent:" ,cmd

        print "SendMsgsToScratch stopped"


def create_socket(host, port):
    while True:
        try:
            print 'Trying'
            scratch_sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            scratch_sock.connect((host, port))
            break
        except socket.error:
            print "There was an error connecting to Scratch!"
            print "I couldn't find a Mesh session at host: %s, port: %s" % (host, port)
            time.sleep(3)
            # sys.exit(1)

    return scratch_sock


def cleanup_threads(threads):
    print "CLEANUP IN PROGRESS"
    print "Threads told to stop"
    for thread in threads:
        thread.stop()
    print "STOPSENDING msg put in queue"
    msgQueue.put((10, "STOPSENDING"))

    print "Waiting for join on main threads to complete"
    for thread in threads:
        thread.join()

    print "All main threads stopped"

    for pin in sghGC.validPins:
        try:
            print "Stopping ", pin
            sghGC.pinRef[pin].stop()
            sghGC.pinRef[pin] = None
            sghGC.pinUse[pin] = sghGC.PUNUSED
            sghGC.pinUpdate(pin, 0)
            print "Stopped ", pin
        except:
            pass

        try:
            if sghGC.pinUse[pin] == sghGC.PSONAR:
                print "Attempting sonar stop on pin:", pin
                sghGC.pinUltraRef[pin].stop()
                sghGC.pinUse[pin] == sghGC.PUNUSED
                print "Sonar stopped on pin:", pin
        except:
            pass

        # print "pin use", sghGC.pinUse[pin]
        if sghGC.pinUse[pin] in [sghGC.POUTPUT]:
            sghGC.pinUpdate(pin, 0)
            # print "pin:" ,pin , " set to 0"

    try:
        print "Stopping Matrix"
        PiMatrix.stop()
        print "Stopped "
    except:
        pass

    print ("cleanup threads finished")


# Main Program Here


# Set some constants and initialise lists

sghGC = sgh_GPIOController.GPIOController(True)

print "pi Revision", sghGC.getPiRevision()

ADDON = ""
# default DEBUG - quiwr = INFO
logging.basicConfig(stream=sys.stderr, level=logging.DEBUG)

logging.basicConfig(
    format='%(asctime)s %(message)s', datefmt='%m/%d/%Y %I:%M:%S %p')

PORT = 42001
DEFAULT_HOST = '127.0.0.1'
BUFFER_SIZE = 8192  # used to be 100
SOCKET_TIMEOUT = 2
firstRun = True
lock = threading.Lock()
cheerlights = CheerLights()

piglow = None
try:
    piglow = sgh_PiGlow.PiGlow(sghGC.i2cbus)
    print ("PiGlow:", piglow)
    print ("Update PWM value on PiGLow attempted")
    piglow.update_pwm_values()  # PiGlow_Values)
except:
    print "No PiGlow Detected"


# See if Compass connected
compass = None
try:
    compass = Compass(gauss=4.7, declination=(-0, 0))
    print "compass detected"
except:
    print "No Compass Detected"

pcaPWM = None
try:
    pcaPWM = PWM(0x40, sghGC.i2cbus, debug=False)
    print pcaPWM
    print pcaPWM.setPWMFreq(60)  # Set frequency to 60 Hz
    print "AdaFruit PWM/Servo Board PCA9685 detected"
except:
    print "No PWM/Servo Board PCA9685 detected"

pcfSensor = None
try:
    pcfSensor = sgh_PCF8591P(sghGC.i2cbus)  # i2c, 0x48)
    print pcfSensor
    print "ADC/DAC PCF8591P Detected"
except:
    print "No ADC/DAC PCF8591 Detected"

AdaMatrix = None
try:
    AdaMatrix = sgh_EightByEight(address=0x70)
    AdaMatrix = ColorEightByEight(address=0x70)
    print AdaMatrix
    print "AdaMatrix Detected"
except:
    print "No AdaMatrix Detected"

PiMatrix = None
#PiMatrix = sgh_PiMatrix.sgh_PiMatrix(0x20,0)
try:
    PiMatrix = sgh_PiMatrix(0x20, sghGC.i2cbus)
    print PiMatrix
    print "PiMatrix Detected"
    PiMatrix.start()
except:
    print "No PiMatrix Detected"
# PiMatrix.start()
# time.sleep(5)

# MCP23008 = None
# #PiMatrix = sgh_PiMatrix.sgh_PiMatrix(0x20,0)
# try:
# if sghGC.getPiRevision() == 1:
# print "Rev1 Board"
# MCP23008 = sgh_MCP23008(0x20,0)
# else:
# MCP23008 = sgh_MCP23008(0x20,1)
# print MCP23008

# print "MCP23008 Detected"
# except:
# print "No MCP23008 Detected"

mcp = None
#mcp = Adafruit_MCP230XX(address = 0x20, num_gpios = 16,busnum = 1)
# print mcp
try:
    mcp = Adafruit_MCP230XX(
        address=0x20, num_gpios=16, busnum=sghGC.i2cbus)  # MCP23017
    print mcp
    print "MCP23017 Detected"
except:
    print "No MCP23017 Detected"

    # Open SPI bus
spi = None

wii = None
try:
    print "looking for nunchuck"
    wii = nunchuck()
    print nunchuck()
    print "NunChuck Detected"
except:
    print "No NunChuck Detected"

RasPiCamera = None
try:
    RasPiCamera = sgh_RasPiCamera.RasPiCamera()
    print RasPiCamera
except:
    print "No Camera Detected"

sghMC = sgh_Minecraft.Minecraft()

if __name__ == '__main__':
    SCRIPTPATH = os.path.split(os.path.realpath(__file__))[0]
    print "PATH:", SCRIPTPATH
    if len(sys.argv) > 1:
        host = sys.argv[1]
    else:
        host = DEFAULT_HOST
    host = host.replace("'", "")

    GPIOPlus = True
    if len(sys.argv) > 2:
        if sys.argv[2] == "standard":
            GPIOPlus = False

cycle_trace = 'start'
msgQueue = Queue.PriorityQueue()


# sghGC.setPinMode()

while True:

    if (cycle_trace == 'disconnected'):
        print "Scratch disconnected"
        cleanup_threads((listener, sender))
        print "Thread cleanup done after disconnect"
        INVERT = False
        sghGC.resetPinMode()
        print ("Pin Reset Done")
        print
        print "-------------------------------------------------------------------------------"
        print
        print
        print
        time.sleep(1)
        cycle_trace = 'start'

    if (cycle_trace == 'start'):
        ADDON = ""
        INVERT = False
        # open the socket
        print 'Starting to connect...',
        the_socket = create_socket(host, PORT)
        print 'Connected!'
        # removed 3dec13 to see what happens
        the_socket.settimeout(SOCKET_TIMEOUT)
        listener = ScratchListener(the_socket)
        #        steppera = StepperControl(11,12,13,15,step_delay)
        #        stepperb = StepperControl(16,18,22,7,step_delay)
        #        stepperc = StepperControl(24,26,19,21,step_delay)

        ##        data = the_socket.recv(BUFFER_SIZE)
        # print "Discard 1st data buffer" , data[4:].lower()
        sender = ScratchSender(the_socket)
        sendMsgs = SendMsgsToScratch(the_socket, msgQueue)
        cycle_trace = 'running'
        print "Running...."
        listener.start()
        sender.start()
        sendMsgs.start()

    # stepperb.start()

    # wait for ctrl+c
    try:
        #        val = values.pop(0)
        #        values.append(val)
        #        # update the piglow with current values
        #        piglow.update_pwm_values(values)

        time.sleep(0.1)
    except KeyboardInterrupt:
        print ("Keyboard Interrupt")
        cleanup_threads((listener, sender))
        print "Thread cleanup done after disconnect"
        # time.sleep(5)
        #sghGC.INVERT = False
        sghGC.resetPinMode()
        print ("Pin Reset Done")
        try:
            print "trying uh clean"
            UH.clean_shutdown()
            print "uhclean done"
        except:
            pass
        sys.exit()
        print "CleanUp complete"

# End of main program
