#!/usr/bin/env python

import roslib; roslib.load_manifest('BioTacPushDemo')
import rospy
import time
import sys
from hubomsg.msg import *
from biotac_sensors.msg import *

# Constants determining how much pressure is considered not touching or pushing
PRESSURE_TOUCHING_LOW = 1810
PRESSURE_TOUCHING_HIGH = 1840
PRESSURE_PUSHING_LOW = 1950
PRESSURE_PUSHING_HIGH = 1980

# Constants for joint limits
RSP_OUT = -1.38
RSP_IN = 0
RSP_RANGE = RSP_IN - RSP_OUT
REP_OUT = 0
REP_IN = -1.34
REP_RANGE = REP_OUT - REP_IN

RSP_INCREMENT = .06 # Amount RSP changes. REP is a function of RSP so they stay synchronized
NUM = 20 # Number of messages to receive before publishing a position command

class Demo:
    def __init__(self):
        rospy.init_node("BioTacDemo")
        
        self.pub = rospy.Publisher("Maestro/Control", PythonMessage) # Sends position commands
        
        self.count = 1
        self.RSP = RSP_IN
        self.REP = REP_IN
        self.pressure = 0.0
        self.last = -1
        time.sleep(1)
        
        # Command arm to go to start position
        print "Moving arm to start position"
        self.pub.publish("RSP REP RF2", "position position position", str(RSP_IN) + " " + str(REP_IN) + ".8", "")
        time.sleep(3)
        print "Starting demo"

        rospy.Subscriber("biotac_pub", BioTacHand, self.sense)

        rospy.spin()

    def sense(self, data):
        btdata = data.bt_data
        self.pressure += btdata[0].pdc_data
        
        if self.count == NUM:
            ave = self.pressure/NUM
            print ave
            
            # Sensor is not being touched. Extend hand outward
            if ave < PRESSURE_TOUCHING_LOW:
                self.last = -1
                self.moveArm(-1)
            # Between touching and not touching
            elif ave < PRESSURE_TOUCHING_HIGH:
                if self.last == -1:
                    self.moveArm(self.last)
                else:
                    self.last = 0
            # Sensor is being touched but not pushed. Do not move hand
            elif(ave < PRESSURE_PUSHING_LOW):
                self.last = 0
            # Between touching and pushing
            elif(ave < PRESSURE_PUSHING_HIGH):
                if self.last == 1:
                    self.moveArm(self.last)
                else:
                    self.last = 0
            # Sensor is being pushed. Retract hand
            else:
                self.last = 1
                self.moveArm(1)
            
            self.count = 1
            self.pressure = 0
        else:
            self.count += 1

    def moveArm(self, direction):
        rspNew = self.RSP + RSP_INCREMENT*direction
        repNew = -REP_RANGE/RSP_RANGE*(rspNew - RSP_IN) + REP_IN
        print "RSP: " + str(rspNew)
        print "REP: " + str(repNew)
        if(rspNew > RSP_OUT and rspNew < RSP_IN and repNew < REP_OUT and repNew > REP_IN): # Make sure command positions are within arm in/out bounds
            self.pub.publish("RSP REP", "position position", str(rspNew) + " " + str(repNew), "")
            self.RSP = rspNew
            self.REP = repNew

    def exit():
        print "Returning arm to home position"
        self.pub.publish("RSP REP RF2", "position position position", "0 0 0", "")
        time.sleep(3)

        print "Exiting"
        sys.exit()


# This magic, courtesy of Eric Rock, magically listens for input in the roslaunching terminal
def input_available():
    rlist, wlist, elist = select.select([sys.stdin], [], [], 0)
    if rlist:
        return True
    return False

if __name__ == '__main__':
    demo = Demo()
    while not rospy.is_shutdown():
        if input_available():
            demo.exit()
        time.sleep(.05)
