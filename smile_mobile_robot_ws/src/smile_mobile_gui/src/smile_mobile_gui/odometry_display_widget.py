#!/usr/bin/env python
'''
Author: David Pierce Walker-Howell<piercedhowell@gmail.com
Date Created: 05/28/2020
Description: The python module is a Qt display widget for the smile mobile robots
            odometry.
'''
import os
import rospy
import rospkg

from qt_gui.plugin import Plugin
from python_qt_binding import loadUi, QtGui, QtCore
from python_qt_binding.QtWidgets import QWidget, QVBoxLayout, QSizePolicy, QApplication

class Odometry_Display_Widget(QWidget):
    '''
    Display the odometry of the smile mobile robot.
    '''
    def __init__(self):
        '''
        Initialize the widget.

        Parameters:
            N/A
        Returns:
            N/A
        '''
        super(QWidget, self).__init__()
        ui_file = os.path.join(rospkg.RosPack().get_path('smile_mobile_gui'), 'resource', 'odometry_display_widget.ui')
        loadUi(ui_file, self)

        #Intialize all the values with zero
        self.display_pose(0.0, 0.0, 0.0)
        self.display_orientation(0.0, 0.0, 0.0)
        self.display_linear_velocity(0.0, 0.0, 0.0)
        self.display_angular_velocity(0.0, 0.0, 0.0)

    def display_pose(self, X, Y, Z):
        '''
        Display the position data (in meters) of the robot

        Parameters:
            X: X position in meters
            Y: Y position in meters
            Z: Z position in meters
        Returns:
            N/A
        '''
        self.pose_x_line_edit.setText(str('{:+.2f}'.format(X)))
        self.pose_y_line_edit.setText(str('{:+.2f}'.format(Y)))
        self.pose_z_line_edit.setText(str('{:+.2f}'.format(Z)))

    def display_linear_velocity(self, x, y, z):
        '''
        Display the linear velocity data (in m/s) of the robot.

        Parameters:
            x: linear x velocity in m/s
            y: linear y velocity in m/s
            z: linear z velocity in m/s
        Returns:
            N/A
        '''
        self.lin_vel_x_line_edit.setText(str('{:+.2f}'.format(x)))
        self.lin_vel_y_line_edit.setText(str('{:+.2f}'.format(y)))
        self.lin_vel_z_line_edit.setText(str('{:+.2f}'.format(z)))

    def display_orientation(self, roll, pitch, yaw):
        '''
        Display the orientation in radians.

        Parameters:
            roll: The angular tilt along the x axis
            pitch: The angular tilt along the y axis
            yaw: The angular position along the z axis
        Returns:
            N/A
        '''
        self.roll_line_edit.setText(str('{:+.2f}'.format(roll)))
        self.pitch_line_edit.setText(str('{:+.2f}'.format(pitch)))
        self.yaw_line_edit.setText(str('{:+.2f}'.format(yaw)))

    def display_angular_velocity(self, x, y, z):
        '''
        Display the angular velocity data (in rad/s) of the robot

        Parameters:
            x: angular x velocity in rad/s
            y: angular y velocity in rad/s
            z: angular z velocity in rad/s
        Returns:
            N/A
        '''
        self.ang_vel_x_line_edit.setText(str('{:+.2f}'.format(x)))
        self.ang_vel_y_line_edit.setText(str('{:+.2f}'.format(y)))
        self.ang_vel_z_line_edit.setText(str('{:+.2f}'.format(z)))

if __name__ == "__main__":
    import sys
    app = QApplication([])
    odometry_display_widget = Odometry_Display_Widget()
    odometry_display_widget.show()
    odometry_display_widget.display_pose(10.1, 11, 124.12)
    odometry_display_widget.display_linear_velocity(-16.454, 1204.12, 0.0)
    odometry_display_widget.display_orientation(-5.64, 320, 491.892)
    odometry_display_widget.display_angular_velocity(-16.454, 1204.12, 0.0)
    sys.exit(app.exec_())
