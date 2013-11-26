#!/usr/bin/env python
# sgh_Stepper - control 5 pin stepper motors from ScratchGPIO.
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

Version =  '0.1.0' # 26Nov13

import time
import threading
import datetime as dt


#----------------------------- STEPPER CONTROL --------------
class sghStepper(threading.Thread):
    def __init__(self,sghGC,pins,step_delay=0.003,lstepMode = "2coil"):
        self.BigNum = 2123456789
        self.step_delay = step_delay
        self.lstepMode = lstepMode
        self.sghGC = sghGC
        self.stepperSpeed = 0 #stepp speed dset to 0 when thread created
        self.steps = self.BigNum # default to psuedo infinte number of turns
        self.terminated = False
        self.toTerminate = False
        threading.Thread.__init__(self)
        self._stop = threading.Event()
        self.pins = pins
        self.slow_start = self.steps
        self.steps_start = self.steps
        self.paused = False
        self.pause_start_time = dt.datetime.now()

    def start(self):
        self.thread = threading.Thread(None, self.run, None, (), {})
        self.thread.start()

    def stop(self):
        #print "Stop Stepper command given"
        self.toTerminate = True
        while self.terminated == False:
        # Just wait
            time.sleep(0.01)
        print "Stepper stopped"

    def changeSpeed(self, stepperSpeed,steps):
        self.stepperSpeed = int(stepperSpeed)
        self.steps = int(steps)
        self.steps_start = self.steps
        self.slow_start = self.steps - int(min(64,max(1,int(float(self.steps)*0.8))))
        if self.steps > (self.BigNum / 2):
            self.slow_start = self.steps - 64       
            
    def step_coarse(self,a,b,c,d,delay):
        #print self.lstepMode
        if self.lstepMode == '1coil':
            self.sghGC.pinUpdate(d,0)
            self.sghGC.pinUpdate(a,1)
            time.sleep(delay)
            
            self.sghGC.pinUpdate(b,1)
            self.sghGC.pinUpdate(a,0)
            time.sleep(delay)
            
            self.sghGC.pinUpdate(c,1)
            self.sghGC.pinUpdate(b,0)
            time.sleep(delay)
            
            self.sghGC.pinUpdate(d,1)
            self.sghGC.pinUpdate(c,0)
            time.sleep(delay)
            
        elif self.lstepMode == '2coil':
            self.sghGC.pinUpdate(d,0)
            self.sghGC.pinUpdate(c,0)
            self.sghGC.pinUpdate(a,1)
            self.sghGC.pinUpdate(b,1)

            time.sleep(delay)

            self.sghGC.pinUpdate(a,0)
            self.sghGC.pinUpdate(c,1)
            time.sleep(delay)
            
            self.sghGC.pinUpdate(b,0)
            self.sghGC.pinUpdate(d,1)
            time.sleep(delay)
            
            self.sghGC.pinUpdate(c,0)
            self.sghGC.pinUpdate(a,1)
            time.sleep(delay)
            
        elif self.lstepMode == 'halfstep':
            self.sghGC.pinUpdate(d,0) 
            self.sghGC.pinUpdate(a,1)
            time.sleep(delay)

            self.sghGC.pinUpdate(b,1)
            time.sleep(delay)

            self.sghGC.pinUpdate(a,0)
            time.sleep(delay)
            
            self.sghGC.pinUpdate(c,1)
            time.sleep(delay)

            self.sghGC.pinUpdate(b,0)
            time.sleep(delay)

            self.sghGC.pinUpdate(d,1)
            time.sleep(delay)

            self.sghGC.pinUpdate(c,0)
            time.sleep(delay)
            
            self.sghGC.pinUpdate(a,1)
            time.sleep(delay)

    def pause(self):
        self.sghGC.pinUpdate(self.pins[0],0)
        self.sghGC.pinUpdate(self.pins[1],0)
        self.sghGC.pinUpdate(self.pins[2],0)
        self.sghGC.pinUpdate(self.pins[3],0)
        self.paused = True
        print self.pins[0], "pause method run"

    def run(self):
        #time.sleep(2) # just wait till board likely to be up and running
        self.pause_start_time = dt.datetime.now()
        while self.toTerminate == False:
            #print self.pins[0],self.pins[1],self.pins[2],self.pins[3]

            if (self.steps > 0):
                self.steps = self.steps - 1
                self.local_stepper_value=self.stepperSpeed # get stepper value in case its changed during this thread
                if self.local_stepper_value != 0: #if stepper_value non-zero
                    self.currentStepDelay = self.step_delay * 100 / abs(self.local_stepper_value)
                    if self.steps < (self.steps_start - self.slow_start) :
                        self.currentStepDelay = self.currentStepDelay *  (3.0 - (((self.steps) / float(self.steps_start - self.slow_start)))*2.0)
                        #print 2.0 - ((self.steps) / float(self.steps_start - self.slow_start))
                    if (self.slow_start < self.steps):
                        #print 2.0 - ((self.steps_start - self.steps) / float(self.steps_start - self.slow_start))
                        self.currentStepDelay = self.currentStepDelay * (3.0 - (((self.steps_start - self.steps) / float(self.steps_start - self.slow_start)))*2.0)
                    #print self.steps, self.currentStepDelay
                    if self.local_stepper_value > 0: # if positive value
                        self.step_coarse(self.pins[0],self.pins[1],self.pins[2],self.pins[3],self.currentStepDelay) #step forward
                        #print dt.datetime.now()
                    else:
                        self.step_coarse(self.pins[3],self.pins[2],self.pins[1],self.pins[0],self.currentStepDelay) #step forward
##                    if abs(local_stepper_value) != 100: # Only introduce delay if motor not full speed
##                        time.sleep(10*self.step_delay*((100/abs(local_stepper_value))-1))
                    self.pause_start_time = dt.datetime.now()
                    self.paused = False
                    #print PIN_NUM[self.pins[0]],self.pause_start_time
                else:
                    if ((dt.datetime.now() - self.pause_start_time).seconds > 10) and (self.paused == False):
                        self.pause()
                        #print PIN_NUM[self.pins[0]], "paused inner"
                        #print PIN_NUM[self.pins[0]], self.paused
                    #else:
                        #if self.paused == False:
                            #print PIN_NUM[self.pins[0]], "inner" ,(dt.datetime.now() - self.pause_start_time).seconds
                    time.sleep(0.1) # sleep if stepper value is zero
            else:
                if ((dt.datetime.now() - self.pause_start_time).seconds > 10) and (self.paused == False):
                    self.pause()
                    #print PIN_NUM[self.pins[0]], "paused outer"
                    #print PIN_NUM[self.pins[0]], self.paused
                #else:
                    #if self.paused == False:
                        #print PIN_NUM[self.pins[0]], "outer" ,(dt.datetime.now() - self.pause_start_time).seconds
                time.sleep(0.1) # sleep if stepper value is zero

        self.terminated = True
    ####### end of Stepper Class






