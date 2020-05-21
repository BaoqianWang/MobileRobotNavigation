#!/usr/bin/env python
'''
Author: David Pierce Walker-Howell<piercedhowell@gmail.com
Date Created: 05/21/2020
Description: This module converts the Husky robots /odometry/filtered
            data to the smile robots Odom type message. This is desired
            since the Husky simulator give orientation in quaternions but
            our project requires [roll, pitch, yaw] type orientation.
'''

import rospy
from nav_msgs.msg import Odometry
from smile_mobile_robot.msg import Odom
import tf
import math

class Cvt_Odom_Data:
    """
    Convert the /odometry/filtered topic data from the Husky Robot simulation
    to the /odometry/measured topic for the smile robot
    """

    def __init__(self, node_name='cvt_odom_data'):
        '''
        Initialization
        '''
        rospy.init_node('cvt_odom_data')

        #Subscribe to the filtered odometry data
        rospy.Subscriber('/odometry/filtered', Odometry, self._husky_odom_callback)

        #Publisher to smile odometry measured data
        self.smile_odom_pub = rospy.Publisher('/odom/measured', Odom, queue_size=10)
        self.smile_odom_msg = Odom()

    def _husky_odom_callback(self, husky_odom_msg):
        '''
        Receive the husky odom msgs, convert it to the smile robots odom message,
        and publish it.

        Parameters:
            husky_odom_msg: Odometry message of type nav_msgs/Odometry.
        Returns:
            N/A
        '''
        quaternion = [
            husky_odom_msg.pose.pose.orientation.x,
            husky_odom_msg.pose.pose.orientation.y,
            husky_odom_msg.pose.pose.orientation.z,
            husky_odom_msg.pose.pose.orientation.w]

        [roll, pitch, yaw] = tf.transformations.euler_from_quaternion(quaternion)
        self.smile_odom_msg.header.stamp = rospy.Time.now()
        self.smile_odom_msg.orientation.roll = roll
        self.smile_odom_msg.orientation.pitch = pitch
        self.smile_odom_msg.orientation.yaw = (yaw * 180.0 / math.pi)

        #Velocity converted from m/s to cm/s => PID constants don't have to be as large.
        self.smile_odom_msg.velocity = husky_odom_msg.twist.twist.linear.x * 100.0

        self.smile_odom_pub.publish(self.smile_odom_msg)

    def run(self):
        '''
        Convert the odometry data from Husky simulation to the smile_robots
        format.
        '''
        try:
            rospy.spin()
        except rospy.ROSInterruptException:
            pass

if __name__ == "__main__":
    cvt_odom_data = Cvt_Odom_Data()
    cvt_odom_data.run()
