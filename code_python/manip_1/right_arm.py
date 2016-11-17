#!/usr/bin/env python
import sys
import time
import dynamixel_lib as Dynamixel

def main(portName, portBaud):
    data_error = 0
    servo = 6 
    data_str = ""
    headline_str = ""
    file_txt = open("servo[6]_attempt_[9].txt", "w")
    error_txt = open("error_s[6].txt", "a")
    #error_txt.write("Datos perdidos por cada iteracion para el servo #6 \n")
    

    ### Communication with dynamixels:
    print "Trying to open port on " + portName + " at " + str(portBaud)
    dynMan1 = Dynamixel.DynamixelMan(portName, portBaud)
    
    ### Write the head line of text file
    headline_str = "Data's result for reading position of each one Dynamixel Servomotors Right_arm at " + str(portBaud) + " Baudrate for 200 datas transmition \n \n \n"
    file_txt.write(headline_str)

    ### 
    for x in range(0, 200):

        Pos_0 = str(dynMan1.GetPresentPosition(servo))      # Read servo position
 
        if Pos_0 == "None":
            Pos_0 = "Data error transmission "
            data_error=data_error+1
        else:
            print "[0]:" + str(Pos_0)
        
        ### Write into text file
        data_str = "Data[" + str(x) + "]: " + str(Pos_0) + "\n"
        file_txt.write(data_str)
    
    ### Output to console         
    print "   "
    print str(data_error)


    file_txt.write("\n Total data transmission failed: " + str(data_error))
    error_txt.write(str(data_error) + "\n")
    time.sleep(1)                       #Sleep time 1 second

    dynMan1.Close()
    file_txt.close()


if __name__ == '__main__':
    portName = "/dev/justinaRightArm"
    portBaud = 1000000
    main(portName, portBaud)

