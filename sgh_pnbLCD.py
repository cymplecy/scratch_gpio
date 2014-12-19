#!/usr/bin/python

# Copyright 2012 Daniel Berlin (with some changes by Adafruit Industries/Limor Fried)
#
# Permission is hereby granted, free of charge, to any person obtaining a copy of
# this software and associated documentation files (the "Software"), to deal  MCP230XX_GPIO(1, 0xin
# the Software without restriction, including without limitation the rights to
# use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies
# of the Software, and to permit persons to whom the Software is furnished to do
# so, subject to the following conditions:

# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.

# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.


import time
import RPi.GPIO as GPIO


#!/usr/bin/python
#
# HD44780 LCD Test Script for
# Raspberry Pi
#
# Author : Matt Hawkins
# Site   : http://www.raspberrypi-spy.co.uk
#
# Date   : 26/07/2012
#
 
# The wiring for the LCD is as follows:
# 1 : GND
# 2 : 5V
# 3 : Contrast (0-5V)*
# 4 : RS (Register Select)
# 5 : R/W (Read Write)       - GROUND THIS PIN
# 6 : Enable or Strobe
# 7 : Data Bit 0             - NOT USED
# 8 : Data Bit 1             - NOT USED
# 9 : Data Bit 2             - NOT USED
# 10: Data Bit 3             - NOT USED
# 11: Data Bit 4
# 12: Data Bit 5
# 13: Data Bit 6
# 14: Data Bit 7
# 15: LCD Backlight +5V**
# 16: LCD Backlight GND
 
#GPIO.setwarnings(False)
# Define GPIO to LCD mapping


GPIO.setmode(GPIO.BOARD)       # Use BOARD GPIO numbers

class sgh_pnbLCD(object):
    def __init__(self):
        self.LCD_RS = 11
        self.LCD_E  = 12
        self.LCD_D4 = 15
        self.LCD_D5 = 16
        self.LCD_D6 = 18
        self.LCD_D7 = 22

        # Define some device constants
        self.LCD_WIDTH = 16    # Maximum characters per line
        self.LCD_CHR = True
        self.LCD_CMD = False

        self.LCD_LINE_1 = 0x80 # LCD RAM address for the 1st line
        self.LCD_LINE_2 = 0xC0 # LCD RAM address for the 2nd line 

        # Timing constants
        self.E_PULSE = 0.0001
        self.E_DELAY = 0.00005
        GPIO.setup(self.LCD_E, GPIO.OUT)  # E
        GPIO.setup(self.LCD_RS, GPIO.OUT) # RS
        GPIO.setup(self.LCD_D4, GPIO.OUT) # DB4
        GPIO.setup(self.LCD_D5, GPIO.OUT) # DB5
        GPIO.setup(self.LCD_D6, GPIO.OUT) # DB6
        GPIO.setup(self.LCD_D7, GPIO.OUT) # DB7
        # Initialise display
        self.lcd_byte(0x33,self.LCD_CMD)
        self.lcd_byte(0x32,self.LCD_CMD)
        self.lcd_byte(0x28,self.LCD_CMD)
        self.lcd_byte(0x0C,self.LCD_CMD)
        self.lcd_byte(0x06,self.LCD_CMD)
        self.lcd_byte(0x01,self.LCD_CMD)  
     
    def lcd_string(self,message):
      # Send string to display
     
      message = message.ljust(self.LCD_WIDTH," ")  
     
      for i in range(self.LCD_WIDTH):
        self.lcd_byte(ord(message[i]),self.LCD_CHR)
     
    def lcd_byte(self,bits, mode):
      # Send byte to data pins
      # bits = data
      # mode = True  for character
      #        False for command
     
      GPIO.output(self.LCD_RS, mode) # RS
     
      # High bits
      GPIO.output(self.LCD_D4, False)
      GPIO.output(self.LCD_D5, False)
      GPIO.output(self.LCD_D6, False)
      GPIO.output(self.LCD_D7, False)
      if bits&0x10==0x10:
        GPIO.output(self.LCD_D4, True)
      if bits&0x20==0x20:
        GPIO.output(self.LCD_D5, True)
      if bits&0x40==0x40:
        GPIO.output(self.LCD_D6, True)
      if bits&0x80==0x80:
        GPIO.output(self.LCD_D7, True)
     
      # Toggle 'Enable' pin
      time.sleep(self.E_DELAY)
      GPIO.output(self.LCD_E, True)
      time.sleep(self.E_PULSE)
      GPIO.output(self.LCD_E, False)
      time.sleep(self.E_DELAY)      
     
      # Low bits
      GPIO.output(self.LCD_D4, False)
      GPIO.output(self.LCD_D5, False)
      GPIO.output(self.LCD_D6, False)
      GPIO.output(self.LCD_D7, False)
      if bits&0x01==0x01:
        GPIO.output(self.LCD_D4, True)
      if bits&0x02==0x02:
        GPIO.output(self.LCD_D5, True)
      if bits&0x04==0x04:
        GPIO.output(self.LCD_D6, True)
      if bits&0x08==0x08:
        GPIO.output(self.LCD_D7, True)
     
      # Toggle 'Enable' pin
      time.sleep(self.E_DELAY)
      GPIO.output(self.LCD_E, True)
      time.sleep(self.E_PULSE)
      GPIO.output(self.LCD_E, False)
      time.sleep(self.E_DELAY) 
      #extra delay added as was getting currpitions
      time.sleep(0.0005)
