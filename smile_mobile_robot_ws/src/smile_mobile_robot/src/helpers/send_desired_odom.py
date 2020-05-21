#!/usr/bin/env python
'''
Author: David Pierce Walker-Howell<piercedhowell@gmail.com
Date Created: 05/21/2020
Description: This python module is a helping/test module to simply
    send desired odometry to the movement controller. Used primarily for
    testing/tuning the PIDs.
'''
import rospy
from nav_msgs.msg import Odometry
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

    parser.add_argument("--odom", type=float, nargs=2,
                        help="Input the desired velocity and angle of the vehicle",
                        required=True)
    args = parser.parse_args()
    print(args.odom)
    rospy.init_node('send_desired_odom')

    odom_pub = rospy.Publisher('/odometry/desired', Odometry, queue_size=1)
    odom_rate = rospy.Rate(10) #Send at 10Hz

    odom_msg = Odometry()

    #Linear velocity and angular position
    odom_msg.twist.twist.linear.x = args.odom[0]
    odom_msg.pose.pose.angular.z = args.odom[1]

    try:
        while not rospy.is_shutdown():
            odom_pub.publish(odom_msg)
            odom_rate.sleep()

    except rospy.ROSInterruptException:
        pass
if __name__ == "__main__":
    run()
