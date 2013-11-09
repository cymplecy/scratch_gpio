#!/usr/bin/env python
# sghGCTest.py
#Copyright (C) 2013 by Simon Walters 

#This program is free software; you can redistribute it and/or
#modify it under the terms of the GNU General Public License
#as published by the Free Software Foundation; either version 2
#of the License, or (at your option) any later version.

#This program is distributed in the hope that it will be useful,
#but WITHOUT ANY WARRANTY; without even the implied warranty of
#MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#GNU General Public License for more details.

#You should have received a copy of the GNU General Public License
#along with this program; if not, write to the Free Software
#Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.


Version =  '0.0.1' # 08Nov13

#from sgh_GPIOController import GPIOController 
import  sgh_GPIOController 
import time
import sys

try:
    sghGC = sgh_GPIOController.GPIOController(True)
    print sghGC.getPiRevision()
    sghGC.pinUse[21] = sghGC.POUTPUT
    sghGC.pinUse[26] = sghGC.POUTPUT
    sghGC.setPinMode()
    sghGC.pin_update(21,0)
    sghGC.pin_update(26,1)
    print (sghGC.pinUse)
    
    while True:
        time.sleep(1)

except KeyboardInterrupt:
    print ("Cleaning up")
    sghGC.cleanup()
    sys.exit()
    print "CleanUp complete"

#### End of main program



