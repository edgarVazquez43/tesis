#!/usr/bin/env python
import sys
import time
import rospy
import dynamixel_lib as Dynamixel

def main(portName, portBaud):
    ###Communication with dynamixels:
    global dynMan1
    print "Trying to open port on " + portName + " at " + str(portBaud)
    dynMan1 = Dynamixel.DynamixelMan(portName, portBaud)

    print "Enable_1: " + str(dynMan1.GetTorqueEnable(1)) + " Enable_2: " + str(dynMan1.GetTorqueEnable(5))

    dynMan1.SetTorqueEnable(1)
    dynMan1.SetTorqueEnable(5)

    dynMan1.SetMovingSpeed(1, 30)
    dynMan1.SetMovingSpeed(5, 30)

    #dynMan1.SetGoalPosition(1, 900)
    #dynMan1.SetGoalPosition(5, 900)  

    count = 0  

    for x in range(0, 2000):

        Pos_1 = str(dynMan1.GetPresentPosition(1))
        Pos_2 = str(dynMan1.GetPresentPosition(5))
 
        if Pos_1 == "None" or Pos_2 == "None":
            count=count+1
            print "Count: " + str(count)
        else:
            print "Pos_1: " + str(Pos_1) + "  Pos_2: " + str(Pos_2)
            
    print "   "
    print str(count)
    
    time.sleep(1)
    
    dynMan1.SetTorqueDisable(1)
    dynMan1.SetTorqueDisable(5)

    time.sleep(1)

    print "Enable_1: " + str(dynMan1.GetTorqueEnable(1)) + " Enable_2: " + str(dynMan1.GetTorqueEnable(5)) 

    dynMan1.Close()

    print "Enable_1: " + str(dynMan1.GetTorqueEnable(1)) + " Enable_2: " + str(dynMan1.GetTorqueEnable(5))

    

if __name__ == '__main__':
    portName = "/dev/justinaHead"
    portBaud = 1000000
    main(portName, portBaud)

