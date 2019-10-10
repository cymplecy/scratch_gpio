#! /usr/bin/python

# GNU GPL V3 

import smbus, time, subprocess, RPi.GPIO as GPIO

# Define list for digits 0..9, space, dash and DP in 7-segment (active High)
digits = [0b00111111, 0b00000110, 0b01011011, 0b01001111, 0b01100110, 0b01101101, 0b01111101, 0b00000111, 0b01111111, 0b01101111, 0b00000000, 0b01000000, 0b10000000]

if GPIO.RPI_REVISION > 1:
   bus = smbus.SMBus(1) # For revision 1 Raspberry Pi, change to bus = smbus.SMBus(1) for revision 2.
else:
   bus = smbus.SMBus(0) # For revision 1 Raspberry Pi, change to bus = smbus.SMBus(1) for revision 2.
   

addr = 0x20 # I2C address of MCP23017
bus.write_byte_data(addr, 0x00, 0x00) # Set all of bank 0 to outputs 
bus.write_byte_data(addr, 0x01, 0x00) # Set all of bank 1 to outputs 
bus.write_byte_data(addr, 0x13, 0xff) # Set all of bank 1 to High (Off) 

def sendDigit(digit, pos):
   t = (1<<pos) ^ 255
   bus.write_byte_data(addr, 0x13, t) # Set bank 1 Pos to Low 
   bus.write_byte_data(addr, 0x12, digit) # Set bank 0 to digit 

speed = 0.005

def iDisplay(val):
   val1 = val/10
   val2 = val1/10
   count1 = 0
   while count1 < 5:
      count2 = 0
      while count2 < 20:
         sendDigit(digits[val - val1*10], 0) # '1'
         time.sleep(speed)
         sendDigit(0, 0)
         sendDigit(digits[val1 - val2*10], 1) # '2'
         time.sleep(speed)
         sendDigit(0, 1)
         sendDigit(digits[val2], 2) # '3'
         time.sleep(speed)
         sendDigit(0, 2)
         sendDigit(digits[0], 3) # '0'
         time.sleep(speed)
         sendDigit(0, 3)
         count2 += 1
      time.sleep(0.5)
      count1 += 1

def scroll(str1, count):
   string = '   ' + str1 + '   '
   for j in range(count):
      for i in range(len(string)-3):
         str2 = string[i:(i+4)]
         sDisplay2(str2, count)
      
def sDisplay2(safeStr, count):
   d1 = val(safeStr[3])
   d2 = val(safeStr[2])
   d3 = val(safeStr[1])
   d4 = val(safeStr[0])
   count2 = 0
   while count2 < 10:
      sendDigit(digits[d1], 0) # '1'
      time.sleep(speed)
      sendDigit(0, 0)
      sendDigit(digits[d2], 1) # '2'
      time.sleep(speed)
      sendDigit(0, 1)
      sendDigit(digits[d3], 2) # '3'
      time.sleep(speed)
      sendDigit(0, 2)
      sendDigit(digits[d4], 3) # '0'
      time.sleep(speed)
      sendDigit(0, 3)
      count2 += 1

def sDisplay(string):
   safeStr = '    ' + string
   l = len(safeStr)
   d1 = val(safeStr[l-1])
   d2 = val(safeStr[l-2])
   d3 = val(safeStr[l-3])
   d4 = val(safeStr[l-4])
   count1 = 0
   while count1 < 5:
      count2 = 0
      while count2 < 20:
         sendDigit(digits[d1], 0) # '1'
         time.sleep(speed)
         sendDigit(0, 0)
         sendDigit(digits[d2], 1) # '2'
         time.sleep(speed)
         sendDigit(0, 1)
         sendDigit(digits[d3], 2) # '3'
         time.sleep(speed)
         sendDigit(0, 2)
         sendDigit(digits[d4], 3) # '0'
         time.sleep(speed)
         sendDigit(0, 3)
         count2 += 1
      time.sleep(0.5)
      count1 += 1

def val(digit):
   if (ord(digit) >= 48 and ord(digit) <= 57):
      return ord(digit) - 48
   elif ord(digit) == 32: #space
      return 10  # 10 is a blank
   elif ord(digit) == 46: #period
      return 11  # 11 is a dash

   
arg = 'ip route list'
waiting = True
while waiting:
   p = subprocess.Popen(arg, shell = True, stdout = subprocess.PIPE)
   data = p.communicate()
   split_data = data[0].split()
   length = len(split_data)
   print 'Length', length
   if length > 8:
      waiting = False
   else:
      scroll("0.0.0.0", 2)
ipaddr = split_data[split_data.index('src')+1]
print ipaddr
#parts = ipaddr.split('.')
#print parts[0]
#print parts[1]
#print parts[2]
#print parts[3]
#print int(parts[3])
#iDisplay(int(parts[3]))
#sDisplay(parts[3])
scroll(ipaddr, 10)
bus.write_byte_data(addr, 0x13, 0xff) # Set all of bank 1 to High (Off) 
