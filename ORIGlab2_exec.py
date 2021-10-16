#!/usr/bin/env python

'''
We get inspirations of Tower of Hanoi algorithm from the website below.
This is also on the lab manual.
Source: https://www.cut-the-knot.org/recurrence/hanoi.shtml
'''

import os
import argparse
import copy
import time
import rospy
import rospkg
import numpy as np
import yaml
import sys
from lab2_header import *

# 20Hz
SPIN_RATE = 50

# UR3 home location
home = np.radians([193.36, -71.39, 85.39, -103.34, -90.31, 103.43])

# UR3 current position, using home position for initialization
current_position = copy.deepcopy(home)

thetas = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

digital_in_0 = 0
analog_in_0 = 0

suction_on = True
suction_off = False

counter = 0

current_io_0 = False
current_position_set = False

# First Column from Top to Bottom
Q01 = np.radians([175.63, -74.24, 107.81, -124.16, -90.09, 85.38])
Q11 = np.radians([175.92, -66.75, 113.98, -137.48, -90.08, 85.80])
Q21 = np.radians([175.92, -60.70, 115.88, -145.43, -90.06, 85.77])
Q31 = np.radians([175.94, -52.10, 116.75, -154.91, -90.04, 85.75])

# Second Column
Q02 = np.radians([192.95, -66.04, 96.33, -120.30, -90.24, 102.71])
Q12 = np.radians([193.06, -58.91, 101.40, -132.78, -90.13, 102.95])
Q22 = np.radians([193.08, -53.73, 103.08, -139.64, -90.12, 102.93])
Q32 = np.radians([193.08, -47.01, 103.77, -147.04, -90.11, 102.92])

# Third Column
Q03 = np.radians([207.80, -52.62, 73.38, -110.35, -90.20, 117.48])
Q13 = np.radians([207.82, -46.00, 79.70, -123.99, -90.17, 117.74])
Q23 = np.radians([207.82, -43.12, 80.77, -127.96, -90.17, 117.73])
Q33 = np.radians([207.84, -36.90, 81.51, -134.90, -90.16, 117.72])


# Matrix of Positions
Q = [[Q01, Q02, Q03], \
    [Q11, Q12, Q13], \
    [Q21, Q22, Q23], \
    [Q31, Q32, Q33]]

############## Your Code Start Here ##############

"""
TODO: define a ROS topic callback funtion for getting the state of suction cup
Whenever ur3/gripper_input publishes info this callback function is called.
"""

def gripper_callback(msg):
    global digital_in_0
    digital_in_0 = msg.DIGIN

############### Your Code End Here ###############
"""
Whenever ur3/position publishes info, this callback function is called.
"""
def position_callback(msg):

    global thetas
    global current_position
    global current_position_set

    thetas[0] = msg.position[0]
    thetas[1] = msg.position[1]
    thetas[2] = msg.position[2]
    thetas[3] = msg.position[3]
    thetas[4] = msg.position[4]
    thetas[5] = msg.position[5]

    current_position[0] = thetas[0]
    current_position[1] = thetas[1]
    current_position[2] = thetas[2]
    current_position[3] = thetas[3]
    current_position[4] = thetas[4]
    current_position[5] = thetas[5]

    current_position_set = True


def gripper(pub_cmd, loop_rate, io_0):

    global SPIN_RATE
    global thetas
    global current_io_0
    global current_position

    error = 0
    spin_count = 0
    at_goal = 0

    current_io_0 = io_0

    driver_msg = command()
    driver_msg.destination = current_position
    driver_msg.v = 1.0
    driver_msg.a = 1.0
    driver_msg.io_0 = io_0
    pub_cmd.publish(driver_msg)

    while(at_goal == 0):

        if( abs(thetas[0]-driver_msg.destination[0]) < 0.0005 and \
            abs(thetas[1]-driver_msg.destination[1]) < 0.0005 and \
            abs(thetas[2]-driver_msg.destination[2]) < 0.0005 and \
            abs(thetas[3]-driver_msg.destination[3]) < 0.0005 and \
            abs(thetas[4]-driver_msg.destination[4]) < 0.0005 and \
            abs(thetas[5]-driver_msg.destination[5]) < 0.0005 ):

            at_goal = 1

        loop_rate.sleep()

        if(spin_count >  SPIN_RATE*5):

            pub_cmd.publish(driver_msg)
            rospy.loginfo("Just published again driver_msg")
            spin_count = 0

        spin_count = spin_count + 1

    return error

def move_arm(pub_cmd, loop_rate, dest, vel, accel):

    global thetas
    global SPIN_RATE

    error = 0
    spin_count = 0
    at_goal = 0

    driver_msg = command()
    driver_msg.destination = dest
    driver_msg.v = vel
    driver_msg.a = accel
    driver_msg.io_0 = current_io_0
    pub_cmd.publish(driver_msg)

    loop_rate.sleep()

    while(at_goal == 0):

        if( abs(thetas[0]-driver_msg.destination[0]) < 0.0005 and \
            abs(thetas[1]-driver_msg.destination[1]) < 0.0005 and \
            abs(thetas[2]-driver_msg.destination[2]) < 0.0005 and \
            abs(thetas[3]-driver_msg.destination[3]) < 0.0005 and \
            abs(thetas[4]-driver_msg.destination[4]) < 0.0005 and \
            abs(thetas[5]-driver_msg.destination[5]) < 0.0005 ):


            at_goal = 1
            #rospy.loginfo("Goal is reached!")

        loop_rate.sleep()

        if(spin_count >  SPIN_RATE*5):

            pub_cmd.publish(driver_msg)
            rospy.loginfo("Just published again driver_msg")
            spin_count = 0

        spin_count = spin_count + 1

    return error

############## Your Code Start Here ##############

def move_block(pub_cmd, loop_rate, start_loc1, start_height, end_loc1, end_height):
    global home
    global Q
    global counter
    global suction_on
    global suction_off
    global digital_in_0


    while (counter == 0):
        move_arm(pub_cmd, loop_rate, home, 1, 1)
        counter = 1

    move_arm(pub_cmd, loop_rate, Q[0][start_loc1], 1, 1)
    move_arm(pub_cmd, loop_rate, Q[start_height][start_loc1], 1, 1)
    gripper(pub_cmd, loop_rate, suction_on)
    time.sleep(0.5)
    if (digital_in_0 == 0):
        gripper(pub_cmd, loop_rate, suction_off)
        print("There is a block missing! Please place it back and run the experiment again.\n")
        sys.exit()
    move_arm(pub_cmd, loop_rate, Q[0][start_loc1], 1, 1)
    move_arm(pub_cmd, loop_rate, Q[0][end_loc1], 1, 1)
    move_arm(pub_cmd, loop_rate, Q[end_height][end_loc1], 1, 1)
    gripper(pub_cmd, loop_rate, suction_off)
    move_arm(pub_cmd, loop_rate, Q[0][end_loc1], 1, 1)



#     driver_msg = command()
#     driver_msg.v = 1
#     driver_msg.a = 1
   
#     while (counter == 0):
#         driver_msg.destination = home
#         pub_cmd.publish(driver_msg)
#         time.sleep(1.5)
#         counter = 1
# #1
#     driver_msg.destination = Q[0][start_loc1]
#     pub_cmd.publish(driver_msg)
#     time.sleep(1.5)
# #2
#     driver_msg.destination = Q[start_height][start_loc1]
#     pub_cmd.publish(driver_msg)
#     time.sleep(1.5)
#     driver_msg.io_0 = True
#     pub_cmd.publish(driver_msg)
#     time.sleep(0.5)
   
#     if(digital_in_0 != 1):
#         driver_msg.io_0 = False
#         pub_cmd.publish(driver_msg)
#         driver_msg.destination = home
#         pub_cmd.publish(driver_msg)
#         time.sleep(1.0)
#         print("There is a block missing! Please place it back and run the experiment again.\n")
#         sys.exit()
# #3
#     driver_msg.destination = Q[0][start_loc1]
#     pub_cmd.publish(driver_msg)
#     time.sleep(1.5)
# #4
#     driver_msg.destination = Q[0][end_loc1]
#     pub_cmd.publish(driver_msg)
#     time.sleep(1.5)
# #5
#     driver_msg.destination = Q[end_height][end_loc1]
#     pub_cmd.publish(driver_msg)
#     time.sleep(1.5)
#     driver_msg.io_0 = False
#     pub_cmd.publish(driver_msg)
#     time.sleep(0.5)
# #6
#     driver_msg.destination = Q[0][end_loc1]
#     pub_cmd.publish(driver_msg)
#     time.sleep(1.5)

############### Your Code End Here ###############

def main():

    global home
    global Q
    global SPIN_RATE

    # Parser
    parser = argparse.ArgumentParser(description='Please specify if using simulator or real robot')
    parser.add_argument('--simulator', type=str, default='True')
    args = parser.parse_args()

    # Initialize ROS node
    rospy.init_node('lab2node')

    # Initialize publisher for ur3/command with buffer size of 10
    pub_command = rospy.Publisher('ur3/command', command, queue_size=10)

    # Initialize subscriber to ur3/position and callback fuction
    # each time data is published
    sub_position = rospy.Subscriber('ur3/position', position, position_callback)

    ############## Your Code Start Here ##############
    # TODO: define a ROS subscriber for ur3/gripper_input message and corresponding callback function

    sub_gripper = rospy.Subscriber('ur3/gripper_input', gripper_input, gripper_callback)

    ############### Your Code End Here ###############


    ############## Your Code Start Here ##############
    # TODO: modify the code below so that program can get user input

    input_done = 0
    loop_count = 0

    while(not input_done):
       
        input_string1 = raw_input("Enter starting location <Either 1 2 3 or 0 to quit> ")
        print("You entered " + input_string1 + "\n")
       
        if(int(input_string1) == 1):
            input_string2 = raw_input("Now enter the ending location <Either 2 3> ")
            print("You entered " + input_string2 + "\n")
            if(int(input_string2) == 2 or int(input_string2) == 3):
                input_done = 1
                loop_count = 1
                start_loc = int(input_string1) - 1
                end_loc = int(input_string2) - 1
            else:
                print("Please just enter 2 or 3 \n\n")
        elif(int(input_string1) == 2):
            input_string2 = raw_input("Now enter the ending location <Either 1 3> ")
            print("You entered " + input_string2 + "\n")
            if(int(input_string2) == 1 or int(input_string2) == 3):
                input_done = 1
                loop_count = 1
                start_loc = int(input_string1) - 1
                end_loc = int(input_string2) - 1
            else:
                print("Please just enter 2 or 3 \n\n")
        elif(int(input_string1) == 3):
            input_string2 = raw_input("Now enter the ending location <Either 1 2> ")
            print("You entered " + input_string2 + "\n")
            if(int(input_string2) == 1 or int(input_string2) == 2):
                input_done = 1
                loop_count = 1
                start_loc = int(input_string1) - 1
                end_loc = int(input_string2) - 1
            else:
                print("Please just enter 2 or 3 \n\n")
        elif (int(input_string1) == 0):
            print("Quitting... ")
            sys.exit()
        else:
            print("Please just enter the character 1 2 3 or 0 to quit \n\n" )

    other_loc = 5 - int(input_string1) - int(input_string2)

    ############### Your Code End Here ###############

    # Check if ROS is ready for operation
    while(rospy.is_shutdown()):

        print("ROS is shutdown!")

#    rospy.loginfo("Sending Goals ...")

    loop_rate = rospy.Rate(SPIN_RATE)

    ############## Your Code Start Here ##############
    # TODO: modify the code so that UR3 can move tower accordingly from user input
    while(loop_count == 1):
#1
        move_block(pub_command, loop_rate, start_loc, 1, end_loc, 3)
#2
        move_block(pub_command, loop_rate, start_loc, 2, other_loc, 3)
#3
        move_block(pub_command, loop_rate, end_loc, 3, other_loc, 2)
#4
        move_block(pub_command, loop_rate, start_loc, 3, end_loc, 3)
#5
        move_block(pub_command, loop_rate, other_loc, 2, start_loc, 3)
#6
        move_block(pub_command, loop_rate, other_loc, 3, end_loc, 2)
#7
        move_block(pub_command, loop_rate, start_loc, 3, end_loc, 1)

        loop_count = loop_count - 1

    ############### Your Code End Here ###############


if __name__ == '__main__':

    try:
        main()
    # When Ctrl+C is executed, it catches the exception
    except rospy.ROSInterruptException:
        pass
