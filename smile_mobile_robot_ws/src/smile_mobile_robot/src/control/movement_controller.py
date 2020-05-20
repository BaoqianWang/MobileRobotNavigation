#!/usr/bin/env python
'''
Author: David Pierce Walker-Howeell<piercedhowell@gmail.com>
Date Created: 05/20/2020
Description: This python module controls the movement of the robot using
              pid controllers.
'''
import rospy

class Movement_Controller:
    """
    Control the robot using PID controllers for linear velocity and
    direction orientation.
    """

    def __init__(self, node_name="movement_controller"):
        '''
        Initialize the movement controller as a ros node.

        Parameters:
            N/A
        Returns:
            N/A
        '''
        self.node_name = node_name
        rospy.init_node(node_name)

    def run(self):
        '''
        Run the main loop of the movement controller. Receive desired and
        measured odometry data and control
        '''

        try:
            while not rospy.is_shutdown():
                continue

        except rospy.ROSInterruptException:
            pass
