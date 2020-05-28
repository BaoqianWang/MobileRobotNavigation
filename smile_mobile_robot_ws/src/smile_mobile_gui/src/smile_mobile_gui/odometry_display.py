#!/usr/bin/env python
'''
Author: David Pierce Walker-Howell<piercedhowell@gmail.com
Date Created: 05/28/2020
Description: The python module is a the ROS Qt plugin for the smile robot
                odometry display. Interfaces the odometry display widget
                with the ros odometry data.
'''

import os
import rospy
import rospkg

from qt_gui.plugin import Plugin
from python_qt_binding import loadUi, QtGui, QtCore
from python_qt_binding.QtWidgets import QWidget, QVBoxLayout, QSizePolicy, QApplication
from odometry_display_widget import Odometry_Display_Widget
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion

class Odometry_Display(Plugin):
    '''
    Odoemtry display plugin
    '''

    def __init__(self, context):
        '''
        Initialize the communication with Odometry data.

        Parameters:
            N/A
        Returns:
            N/A
        '''

        #Initialize with qt_gui plugin for ros
        super(Odometry_Display, self).__init__(context)

        self.odometry_display_widget = Odometry_Display_Widget()

        #Display the widget
        context.add_widget(self.odometry_display_widget)

        #Subscribe to the odoemtry data
        odometry_topic = 'smile/raw/odometry'

        #Initialize subscriber for raw odometry
        rospy.Subscriber(odometry_topic, Odometry, self._odom_data_callback)

    def _odom_data_callback(self, odom_msg):
        '''
        Callback function for receive odometry data. Will display data in the
        widget.

        Parameters:
            odom_msg: The odometry message of type nav_msgs/Odometry
        Returns:
            N/A
        '''
        self.odom_msg = odom_msg
        pose_X = self.odom_msg.pose.pose.position.x
        pose_Y = self.odom_msg.pose.pose.position.y
        pose_Z = self.odom_msg.pose.pose.position.z
        self.odometry_display_widget.display_pose(pose_X, pose_Y, pose_Z)


        quaternion = [self.odom_msg.pose.pose.orientation.x,
                      self.odom_msg.pose.pose.orientation.y,
                      self.odom_msg.pose.pose.orientation.z,
                      self.odom_msg.pose.pose.orientation.w]
        [roll, pitch, yaw] = euler_from_quaternion(quaternion)
        self.odometry_display_widget.display_orientation(roll, pitch, yaw)

        lin_vel_x = self.odom_msg.twist.twist.linear.x
        lin_vel_y = self.odom_msg.twist.twist.linear.y
        lin_vel_z = self.odom_msg.twist.twist.linear.z
        self.odometry_display_widget.display_linear_velocity(lin_vel_x, lin_vel_y, lin_vel_z)

        ang_vel_x = self.odom_msg.twist.twist.angular.x
        ang_vel_y = self.odom_msg.twist.twist.angular.y
        ang_vel_z = self.odom_msg.twist.twist.angular.z
        self.odometry_display_widget.display_angular_velocity(ang_vel_x, ang_vel_y, ang_vel_z)
