#!/usr/bin/python

import time
import datetime
from sgh_Adafruit_LEDBackpack import sgh_LEDBackpack

# ===========================================================================
# 8x8 Pixel Display
# Modified by Simon Walters as interface to ScratchGPIO
# ===========================================================================

class sgh_EightByEight:
  disp = None

  # Constructor
  def __init__(self, address=0x70, debug=False):
        if (debug):
            print "Initializing a new instance of LEDBackpack at 0x%02X" % address
        self.disp = sgh_LEDBackpack(address=address, debug=debug)
        self.matrix = [0] * 64
        self.rotate = 0
    
  def checkRotate(self, x , y):
      if self.rotate == 0:
          return x, y
      elif self.rotate == 1:
          return y, 7-x    
      elif self.rotate == 2:
          return 7-x, 7-y    
      elif self.rotate == 3:
          return 7-y, x    

  def setRotate(self, rotate):
      self.rotate = rotate
      print self.rotate
      
  def getPixel(self, x , y):
        return self.matrix[(x)+(y*8)]


  def writeRowRaw(self, charNumber, value):
    "Sets a row of pixels using a raw 16-bit value"
    if (charNumber > 7):
      return
    # Set the appropriate row
    self.disp.setBufferRow(charNumber, value)

  def clearPixel(self, x, y):
    "A wrapper function to clear pixels (purely cosmetic)"
    self.setPixel(x, y, 0)

  def setPixel(self, x, y, color=1):
    "Sets a single pixel"
    x , y = self.checkRotate(x,y)
    self.matrix[(x)+(y*8)] = color
    if (x >= 8):
      return
    if (y >= 8):
      return   
    x = 7 - x      
    x += 7   # ATTN: This might be a bug?  On the color matrix, this causes x=0 to draw on the last line instead of the first.
    x %= 8
    # Set the appropriate pixel
    buffer = self.disp.getBuffer()
    if (color):
      self.disp.setBufferRow(y, buffer[y] | 1 << x)
    else:
      self.disp.setBufferRow(y, buffer[y] & ~(1 << x))
    #print self.matrix

  def clear(self):
    "Clears the entire display"
    self.disp.clear()
    self.matrix = [0] * 64
    
  def setBrightness(self, brightness):
    "Clears the entire display"
    self.disp.setBrightness(brightness)

  def scroll(self, direction = "left"):
    #print direction
    if direction == "left":
        for y in range(0,8):
            for x in range (0,7):
                #print (x+1+(y*8))
                self.setPixel(x,y,self.matrix[x+1+(y*8)])
        x=7
        for y in range(0,8):
            self.setPixel(x,y,0)
        
    if direction == "right":
        print "right"
        for y in range(0,8):
            for x in range (7,0,-1):
                #print (x+1+(y*8))
                self.setPixel(x,y,self.matrix[x-1+(y*8)])   
        x=0
        for y in range(0,8):
            self.setPixel(x,y,0)            




                
class ColorEightByEight(sgh_EightByEight):
  def setPixel(self, x, y, color=1):
    "Sets a single pixel"
    print x , y
    x , y = self.checkRotate(x,y)
    print x , y
    self.matrix[(x)+(y*8)] = color    
    if (x >= 8):
      return
    if (y >= 8):
      return
    
    x = 7 - x

    x %= 8

    # Set the appropriate pixel
    buffer = self.disp.getBuffer()

    # TODO : Named color constants?
    # ATNN : This code was mostly taken from the arduino code, but with the addition of clearing the other bit when setting red or green.
    #        The arduino code does not do that, and might have the bug where if you draw red or green, then the other color, it actually draws yellow.
    #        The bug doesn't show up in the examples because it's always clearing.

    if (color == 1):
      self.disp.setBufferRow(y, (buffer[y] | (1 << x)) & ~(1 << (x+8)) )
    elif (color == 2):
      self.disp.setBufferRow(y, (buffer[y] | 1 << (x+8)) & ~(1 << x) )
    elif (color == 3):
      self.disp.setBufferRow(y, buffer[y] | (1 << (x+8)) | (1 << x) )
    else:
      self.disp.setBufferRow(y, buffer[y] & ~(1 << x) & ~(1 << (x+8)) )

