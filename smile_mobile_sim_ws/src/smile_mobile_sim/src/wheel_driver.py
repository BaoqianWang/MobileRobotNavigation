#!/usr/bin/env python
'''
Author: David Pierce Walker-Howeell<piercedhowell@gmail.com>
Date Created: 05/19/2020
Description: This python module is the wheel driver for the Husky robot used in
            the gazebo simulator with ros. This module receives desired PWM values
            as if real motors, maps them to velocities, and publishes them to the
            simulated robot.
'''
import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Int16MultiArray

class Wheel_Driver:
    """
    This class drives the wheels of the simulated smile mobile robot within
    Gazebo.
    """
    def __init__(self, node_name='wheel_driver'):
        '''
        Initialize the wheel driver.

        Parameters:
            node_name: A string for the name of this ros node. Default: wheel_driver.
        Returns:
            N/A
        '''
        rospy.init_node(node_name)

        #Publish the velocity of the robot with a geometry_msgs/Twist msg
        self.vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

        self.vel_pub_rate = rospy.Rate(100) #100Hz

        #Initialize the message packet
        self.vel_twist_msg = Twist()
        self._zero_velocities()

        #Subscriber for the pwm messages (topic = pwm)
        rospy.Subscriber("pwm", Int16MultiArray, self._pwm_subscriber_callback)
        self.velocity_pwm = 0
        self.angle_pwm = 0


    def _pwm_subscriber_callback(self, pwm_msg):
        '''
        Callback function for the pwm subscriber to receive pwm data

        Parameters:
            pwm_data: A message of type Int32MultiArray
        Returns:
            N/A
        '''
        self.velocity_pwm = pwm_msg.data[0]
        self.angle_pwm = pwm_msg.data[1]

        self.vel_twist_msg.linear.x = self.velocity_pwm
        self.vel_twist_msg.linear.y = 0
        self.vel_twist_msg.linear.z = 0

        self.vel_twist_msg.angular.x = 0
        self.vel_twist_msg.angular.y = 0
        self.vel_twist_msg.angular.z = self.angle_pwm

        return

    def _zero_velocities(self):
        '''
        Zeros-out all the velocities in all directions

        Parameters:
            N/A
        Returns:
            N/A
        '''
        self.vel_twist_msg.linear.x = 0
        self.vel_twist_msg.linear.y = 0
        self.vel_twist_msg.linear.z = 0

        self.vel_twist_msg.angular.x = 0
        self.vel_twist_msg.angular.y = 0
        self.vel_twist_msg.angular.z = 0
        return


    def run(self):
        '''
        The main loop controlling the velocity/direction of the simulated robot.

        Parameters:
            N/A
        Returns:
            N/A
        '''

        #Main loop
        try:
            while not rospy.is_shutdown():

                #Publish the velocity message on topic /cmd_vel
                self.vel_pub.publish(self.vel_twist_msg)

                self.vel_pub_rate.sleep()
        except rospy.ROSInterruptException:
            print("[ERROR]: Interruption Occured in Wheel_Driver")

if __name__ == "__main__":

    wheel_driver = Wheel_Driver()
    wheel_driver.run()
