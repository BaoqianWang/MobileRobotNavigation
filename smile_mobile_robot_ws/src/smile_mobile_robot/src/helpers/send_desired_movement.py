#!/usr/bin/env python
'''
Author: David Pierce Walker-Howell<piercedhowell@gmail.com
Date Created: 05/21/2020
Description: This python module is a helping/test module to simply
    send desired odometry to the movement controller. Used primarily for
    testing/tuning the PIDs.
'''
import rospy
from std_msgs.msg import Float32MultiArray
import sys
import argparse

def run():
    '''
    Run continous loop of receiving desired odom from cmd and publishing it
    for the movement_controller to receive.

    Parameters:
        N/A
    Returns:
        N/A
    '''

    parser = argparse.ArgumentParser()

    parser.add_argument("--movement", type=float, nargs=2,
                        help="Input the desired velocity (m/s) and angle (rad) of the vehicle",
                        required=True)
    args = parser.parse_args()
    print(args.movement)
    rospy.init_node('send_desired_movement')

    #topic
    desired_movement_topic = 'smile/desired/movement'

    desired_movement_pub = rospy.Publisher(desired_movement_topic, Float32MultiArray, queue_size=1)
    desired_movement_rate = rospy.Rate(10) #Send at 10Hz

    desired_movement_msg = Float32MultiArray()
    desired_movement_msg.data = args.movement

    try:
        while not rospy.is_shutdown():

            desired_movement_pub.publish(desired_movement_msg)
            desired_movement_rate.sleep()

    except rospy.ROSInterruptException:
        pass
if __name__ == "__main__":
    run()
