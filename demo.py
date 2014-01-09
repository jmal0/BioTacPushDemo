#!/usr/bin/env python

import roslib; roslib.load_manifest('BiotacPushDemo')
import rospy
import time
from hubomsg.msg import *
from biotac_sensors.msg import *

# Constants determining how much pressure is considered not touching or pushing
PRESSURE_TOUCHING = 2100 # This number is untested and not at all final
PRESSURE_PUSHING = 3000 # This number is untested and not at all final

# Constants for joint limits and position increment
RSP_OUT = -1.0
RSP_IN = -.6
RSP_RANGE = RSP_IN - RSP_OUT
REP_OUT = -.7
REP_IN = -1.4
REP_RANGE = REP_OUT - REP_IN
RSP_INCREMENT = .01 # Amount RSP changes. REP is a function of REP so they stay synchronized ## This number is untested

class Demo:
    def __init__(self):
        rospy.init_node("BioTacDemo")
        rospy.Subscriber("biotac_pub", BioTacHand, self.sense)
        self.pub = rospy.Publisher("Maestro/Control", PythonMessage) # Sends position commands
        self.count = 0

        # Command arm to go to start position
        print "Moving arm to start position"
        self.pub.publish("RSP REP", "position position", str(RSP_OUT) + " " + str(REP_OUT), "")
        time.sleep(10) # Sleep to allow arm to move to start position
        print "Starting demo"
        self.rsp = RSP_OUT
        self.rep = REP_OUT

        rospy.spin()

    def sense(self, data):
        btdata = data.bt_data
        pressure = btdata[0].pdc_data
        print pressure

        # Sensor is not being touched. Extend hand outward
        if(pressure < PRESSURE_TOUCHING):
            moveArm(1)
        # Sensor is being pushed. Retract hand
        elif(pressure > PRESSURE_PUSHING):
            moveArm(-1)
        # Else, sensor is being touched but not pushed. Do not move hand

    def moveArm(self, direction):
        rspNew = self.RSP + RSP_INCREMENT*direction
        repNew = REP_RANGE/RSP_RANGE*(self.RSP - RSP_IN) + REP_IN
        if(rspNew > RSP_OUT and rspNew < RSP_IN and repNew < REP_OUT and repNew > REP_IN): # Make sure command positions are within arm in/out bounds
            self.pub.publish("RSP REP", "position position", str(rspNew) + " " + str(repNew), "")
            self.RSP = rspNew
            self.REP = repNew
        

if __name__ == '__main__':
    try:
        reactiveTouchdemo = Demo()
    except KeyboardInterrupt:
        sys.exit()