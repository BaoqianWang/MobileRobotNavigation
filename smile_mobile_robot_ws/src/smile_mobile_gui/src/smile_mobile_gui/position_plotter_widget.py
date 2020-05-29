#!/usr/bin/env python
'''
Author: David Pierce Walker-Howeell<piercedhowell@gmail.com>
Date Created: 05/25/2020
Description: A python widget using Matplotlib to plot the x,y position of a robot.
'''
import os
import rospy
import rospkg

from qt_gui.plugin import Plugin
from python_qt_binding import loadUi, QtGui, QtCore
from python_qt_binding.QtWidgets import QWidget, QVBoxLayout, QSizePolicy
from numpy import arange, sin, pi
from matplotlib.backends.backend_qt4agg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.backends.backend_qt4agg import NavigationToolbar2QT as NavigationToolbar
from matplotlib.figure import Figure
import random

class Matplotlib_Canvas(FigureCanvas):
    '''
    A Matplotlib canvas for defining a basic plot
    '''
    def __init__(self, parent=None, width=5, height=4, dpi=100):
        '''
        Initialize a Canvas for plotting abilities.

        Parameter:
            parent:
            width: The widget of the plotting area.
            height: The height of the plotting area
            dpi:
        Returns:
            N/A
        '''

        fig = Figure(figsize=(width, height), dpi=dpi)

        #Single plot
        self.axes = fig.add_subplot(111)

        self.compute_initial_figure()

        FigureCanvas.__init__(self, fig)
        self.setParent(parent)

        FigureCanvas.setSizePolicy(self,
                                   QSizePolicy.Expanding,
                                   QSizePolicy.Expanding)
        FigureCanvas.updateGeometry(self)


    def compute_initial_figure(self):
        pass


class Position_Plotter_Widget(Matplotlib_Canvas):
    '''
    Qt ROS plugin for plotting x,y, phi position of smile mobile robot
    '''
    def __init__(self, *args, **kwargs):
        '''
        Initialization of the position plotter widget

        Parameter:
            context
        Returns:
            N/A
        '''

        Matplotlib_Canvas.__init__(self, *args, **kwargs)

        self.new_x, self.new_y, self.prev_x, self.prev_y = 0.0, 0.0, 0.0, 0.0

        #Make figure have grid lines
        self.axes.grid(True, which='both')
        self.axes.set_title('Position')
        self.axes.set_xlabel('x (m)')
        self.axes.set_ylabel('y (m)')


        #Update timer for plot (default 0.5 seconds)
        timer = QtCore.QTimer(self)
        timer.timeout.connect(self._update_figure)
        timer.start(500)

    def _update_figure(self):
        '''
        Update the plot every 0.5 seconds if a new position point is
        available.

        Parameter:
            N/A
        Returns:
            N/A
        '''
        #Plot a new point only if it is not the same as the previous point.
        if((self.new_x != self.prev_x) or (self.new_y != self.prev_y)):
            self.axes.scatter(self.new_x, self.new_y, marker="d", c='r')

            #Draw the new point
            self.draw()

    def clear_figure(self):
        '''
        Clear the figure of plot (start fresh!)

        Parameter:
            N/A
        Returns:
            N/A
        '''
        self.axes.cla()

    def set_point(self, x, y):
        '''
        Set a new x, y coordinate to plot:

        Parameters:
            x: The x-coordinate
            y: The y-coordinate
        Returns:
            N/A
        '''
        self.prev_x = self.new_x
        self.prev_y = self.new_y
        self.new_x = x
        self.new_y = y
