#!/usr/bin/env python
'''
Author: David Pierce Walker-Howell<piercedhowell@gmail.com
Date Created: 05/26/2020
Description: The python module is a pid tuner qt qut plugin for ros to tune
            the pid controllers for the robots movement.
'''
import rospy
import rospkg
from qt_gui.plugin import Plugin
from python_qt_binding import loadUi, QtGui, QtCore
from python_qt_binding.QtWidgets import QWidget, QVBoxLayout, QSizePolicy, QPushButton
from pid_tuner_generic_widget import PID_Tuner_Generic_Widget
from smile_mobile_robot.srv import *

class PID_Tuner(Plugin):
    '''
    PID tunning widget for the velocity and steering PID controllers.
    '''

    def __init__(self, context):
        '''
        Initialie the PID tuner

        Parameters:
            N/A
        Returns:
            N/A
        '''
        #Initialize with qt_gui plugin for ros
        super(PID_Tuner, self).__init__(context)

        self.layout = QVBoxLayout()
        self._widget = QWidget()
        self._widget.setLayout(self.layout)

        #Import a generic pid tunning widget for each pid controller
        self.velocity_pid_tuner_widget = PID_Tuner_Generic_Widget()
        self.steering_pid_tuner_widget = PID_Tuner_Generic_Widget()
        self.pid_tuner_widgets = [self.velocity_pid_tuner_widget, self.steering_pid_tuner_widget]

        #Add button for sending the gains
        self.update_gains_btn = QPushButton("Update Gains")
        self.update_gains_btn.clicked.connect(self._update_gains)

        #Add the pid tuners to a layout
        for pid_tuner_widget in self.pid_tuner_widgets:
            self.layout.addWidget(pid_tuner_widget)

        self.layout.addWidget(self.update_gains_btn)

        context.add_widget(self._widget)

    def _update_gains(self):
        '''
        Update the PID gains on the robot by sending them to the robot through
        the service.

        Parameters:
            N/A
        Returns:
            N/A
        '''
        vel_k_p = self.velocity_pid_tuner_widget.k_p
        vel_k_i = self.velocity_pid_tuner_widget.k_i
        vel_k_d = self.velocity_pid_tuner_widget.k_d
        steer_k_p = self.steering_pid_tuner_widget.k_p
        steer_k_i = self.steering_pid_tuner_widget.k_i
        steer_k_d = self.steering_pid_tuner_widget.k_d


        update_pid_gains = rospy.ServiceProxy('smile/update_pid_gains', pid_gains)
        update_pid_gains(vel_k_p, vel_k_i, vel_k_d, steer_k_p, steer_k_i, steer_k_d)
