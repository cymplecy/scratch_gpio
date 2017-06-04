# meArm.py - York Hack Space May 2014
# A motion control library for Phenoptix meArm using Adafruit 16-channel PWM servo driver

from sgh_Adafruit_PWM_Servo_Driver import PWM
import kinematics
import time
from math import pi

class meArm():
    def __init__(self, sweepMinBase = -206, sweepMaxBase = 206, angleMinBase = -pi / 2.0 , angleMaxBase = pi / 2.0,
    			 sweepMinShoulder = 103, sweepMaxShoulder = -103, angleMinShoulder = pi / 4.0, angleMaxShoulder = 3 * pi / 4.0,
    			 sweepMinElbow = 0, sweepMaxElbow = 103, angleMinElbow = 0, angleMaxElbow = -2 * pi / 4.0,
    			 sweepMinGripper = 75, sweepMaxGripper = 115, angleMinGripper = pi / 2.0, angleMaxGripper = 0):
        """Constructor for meArm - can use as default arm=meArm(), or supply calibration data for servos."""
        self.servoInfo = {}
        self.servoInfo["base"] = self.setupServo(sweepMinBase, sweepMaxBase, angleMinBase, angleMaxBase)
        self.servoInfo["shoulder"] = self.setupServo(sweepMinShoulder, sweepMaxShoulder, angleMinShoulder, angleMaxShoulder)
        self.servoInfo["elbow"] = self.setupServo(sweepMinElbow, sweepMaxElbow, angleMinElbow, angleMaxElbow)
        self.servoInfo["gripper"] = self.setupServo(sweepMinGripper, sweepMaxGripper, angleMinGripper, angleMaxGripper)
        print "servoinfo" , self.servoInfo
        self.radBase = 0.0
        self.radShoulder = pi / 2.0
        self.radElbow = 0.0
        self.BasePos = 0
        self.ShoulderPos = 0
        self.ElbowPos = 0
        
    # Adafruit servo driver has four 'blocks' of four servo connectors, 0, 1, 2 or 3.
    def begin(self, block = 0, address = 0x40):
        """Call begin() before any other meArm calls.  Optional parameters to select a different block of servo connectors or different I2C address."""
        self.pwm = 0#PWM(address) # Address of Adafruit PWM servo driver
        self.base = block * 4
        self.shoulder = block * 4 + 1
        self.elbow = block * 4 + 2
        self.gripper = block * 4 + 3
        #self.pwm.setPWMFreq(60)
        self.openGripper()
        self.goDirectlyTo(0, 148, 80)
        
    def setupServo(self, n_min, n_max, a_min, a_max):
        """Calculate servo calibration record to place in self.servoInfo"""
        rec = {}
        n_range = (n_max - n_min)
        a_range = (a_max - a_min)
        if a_range == 0: return
        gain = n_range / a_range
        zero = n_min - gain * a_min
        rec["gain"] = gain
        rec["zero"] = zero
        rec["min"] = n_min
        rec["max"] = n_max
        return rec
    
    def angle2pwm(self, servo, angle):
        """Work out pulse length to use to achieve a given requested angle taking into account stored calibration data"""
        ret = int((self.servoInfo[servo]["zero"] + self.servoInfo[servo]["gain"] * angle) / 1.0 )
        print "servo",servo,angle
        return ret
        
    def goDirectlyTo(self, x, y, z):
        """Set servo angles so as to place the gripper at a given Cartesian point as quickly as possible, without caring what path it takes to get there"""
        angles = [0,0,0]
        if kinematics.solve(x, y, z, angles):
            print "angles", angles
            print "degs", [int(x * 180.0) / pi for x in angles] 

            self.radBase = angles[0]
            self.radShoulder = angles[1] 
            self.radElbow = angles[2]
            self.BasePos = self.angle2pwm("base", self.radBase)
            self.ShoulderPos = self.angle2pwm("shoulder", self.radShoulder)
            self.ElbowPos = self.angle2pwm("elbow", self.radElbow)

            self.x = x
            self.y = y
            self.z = z
            print "goto %s" % ([x,y,z])
            
    def gotoPoint(self, x, y, z):
        """Travel in a straight line from current position to a requested position"""
        x0 = self.x
        y0 = self.y
        z0 = self.z
        dist = kinematics.distance(x0, y0, z0, x, y, z)
        step = 10
        i = 0
        while i < dist:
            self.goDirectlyTo(x0 + (x - x0) * i / dist, y0 + (y - y0) * i / dist, z0 + (z - z0) * i / dist)
            time.sleep(0.05)
            i += step
        self.goDirectlyTo(x, y, z)
        time.sleep(0.05)
        
    def openGripper(self):
        """Open the gripper, dropping whatever is being carried"""
        #self.pwm.setPWM(self.gripper, 0, self.angle2pwm("gripper", pi/4.0))
        time.sleep(0.3)
        
    def closeGripper(self):
        """Close the gripper, grabbing onto anything that might be there"""
        #self.pwm.setPWM(self.gripper, 0, self.angle2pwm("gripper", -pi/4.0))
        time.sleep(0.3)
    
    def isReachable(self, x, y, z):
        """Returns True if the point is (theoretically) reachable by the gripper"""
        radBase = 0
        radShoulder = 0
        radElbow = 0
        return kinematics.solve(x, y, z, [radBase, radShoulder, radElbow])
    
    def getPos(self):
        """Returns the current position of the gripper"""
        return [self.x, self.y, self.z]
