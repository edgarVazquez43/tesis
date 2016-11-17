#!/usr/bin/env python
import sys
import rospy
import dynamixel_lib as Dynamixel

def main(portName, portBaud):
    ###Communication with dynamixels:
    global dynMan1
    print "Trying to open port on " + portName + " at " + str(portBaud)
    dynMan1 = Dynamixel.DynamixelMan(portName, portBaud)

    dynMan1.SetTorqueEnable(1, 1)
    dynMan1.SetTorqueEnable(5, 1)

    dynMan1.SetMovingSpeed(1, 20)
    dynMan1.SetMovingSpeed(5, 20)

    dynMan1.SetGoalPosition(1, 1500)
    dynMan1.SetGoalPosition(5, 1500)  

    count = 0  

    for x in range(0, 5000):
        Pos_1 = str(dynMan1.GetPresentPosition(1))
        Pos_2 = str(dynMan1.GetPresentPosition(5))
        if Pos_1 == "None" or Pos_2 == "None":
            count=count+1
            print "Count: " + str(count)
        else:
            print "Pos_1: " + str(dynMan1.GetPresentPosition(1)) + "  Pos_2: " + str(dynMan1.GetPresentPosition(5))
            
    print "   "
    print str(count)

if __name__ == '__main__':
    portName = "/dev/justinaHead"
    portBaud = 1000000
    main(portName, portBaud)

