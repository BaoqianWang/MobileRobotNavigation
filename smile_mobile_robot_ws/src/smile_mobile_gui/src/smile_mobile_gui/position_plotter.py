#!/usr/bin/env python
'''
Author: David Pierce Walker-Howeell<piercedhowell@gmail.com>
Date Created: 05/24/2020
Description: This python module is the ros Qt plugin for plotting the x,y position
            of the robot.
'''
import os
import rospy
import rospkg
from nav_msgs.msg import Odometry

from qt_gui.plugin import Plugin
from python_qt_binding import loadUi
from python_qt_binding.QtWidgets import QWidget
from smile_mobile_gui.position_plotter_widget import Position_Plotter_Widget


class Position_Plotter(Plugin):

    def __init__(self, context):

        #Initialize with qt_gui plugin for ros
        super(Position_Plotter, self).__init__(context)

        # Give QObjects reasonable names
        self.setObjectName('Position_Plotter')

        #Import the position plotting widget
        self.pose_plot_widget = Position_Plotter_Widget()

        # Show _widget.windowTitle on left-top of each plugin (when
        # it's set in _widget). This is useful when you open multiple
        # plugins at once. Also if you open multiple instances of your
        # plugin at once, these lines add number to make it easy to
        # tell from pane to pane.
        if context.serial_number() > 1:
            self._widget.setWindowTitle(self.pose_plot_widget.windowTitle() + (' (%d)' % context.serial_number()))
        # Add widget to the user interface
        context.add_widget(self.pose_plot_widget)

        #Subscriber to the raw odometry data
        rospy.Subscriber('smile/raw/odometry', Odometry, self._raw_odom_callback)

    def _raw_odom_callback(self, raw_odom_msg):
        '''
        Callback function for the raw odometry message subscriber.

        Parameters:
            raw_odom_msg: The raw odometry message of type nav_msgs/Odometry
        Returns:
            N/A
        '''
        x = raw_odom_msg.pose.pose.position.x
        y = raw_odom_msg.pose.pose.position.y
        self.pose_plot_widget.set_point(x, y)

    def shutdown_plugin(self):
        # TODO unregister all publishers here
        pass

    def save_settings(self, plugin_settings, instance_settings):
        # TODO save intrinsic configuration, usually using:
        # instance_settings.set_value(k, v)
        pass

    def restore_settings(self, plugin_settings, instance_settings):
        # TODO restore intrinsic configuration, usually using:
        # v = instance_settings.value(k)
        pass

    #def trigger_configuration(self):
        # Comment in to signal that the plugin has a way to configure
        # This will enable a setting button (gear icon) in each dock widget title bar
        # Usually used to open a modal configuration dialog
