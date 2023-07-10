#! /usr/bin/env python

## @package rt2_assignment1
#
# \file user_interface.py
# \brief This file implement the user interface
# \author Roberta Alessi
# \version 0.1
# 
# Description :
# The user is asked if they want to start the robot by pressing an input or if they want to stop the robot by pressing another input.
#

import rospy
import time
from rt2_assignment1.srv import Command

##
#
# \brief Main function
#
#
# Here the node is defined as "user_interface"
#

def main():
    rospy.init_node('user_interface')
    ui_client = rospy.ServiceProxy('/user_interface', Command)
    time.sleep(10)
    rate = rospy.Rate(20)
    x = int(input("\nPress 1 to start the robot "))
    while not rospy.is_shutdown():
        if (x == 1):
            ui_client("start")
            x = int(input("\nPress 0 to stop the robot "))
        else:
            print("Please wait, the robot is going to stop when the position will be reached")
            ui_client("stop")
            x = int(input("\nPress 1 to start the robot "))
            
if __name__ == '__main__':
    main()
