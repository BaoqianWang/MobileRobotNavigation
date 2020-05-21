#!/usr/bin/env python
'''
Author: David Pierce Walker-Howeell<piercedhowell@gmail.com>
Date Created: 05/20/2020
Description: This python module controls the movement of the robot using
              pid controllers.
'''
import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64, Int16MultiArray

import time

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

        #Initialize subscriber for measured_odometry
        rospy.Subscriber("/odometry/filtered", Odometry, self._measured_odom_data_callback)

        #Initialize subscriber for desired_odometry
        rospy.Subscriber("/odometry/desired", Odometry, self._desired_odom_data_callback)

        self._initialize_pid_controlers()

        self.pwm_pub = rospy.Publisher("/pwm", Int16MultiArray, queue_size=10)
        self.pwm_msg = Int16MultiArray()

        self.pid_timer = rospy.Rate(100) #100Hz

        #Initialize publisher for writing PWM
        self.measured_vel_x = 0.0
        self.desired_vel_x = 0.0
        self.measured_orientation = 0.0
        self.desired_orientation = 0.0
        self.vel_control = 0.0
        self.steering_control = 0.0

    def _initialize_pid_controlers(self):
        '''
        Initialize communication with the pid controllers.

        Parameters:
            N/A
        Returns:
            N/A
        '''
        #Inialize the communication with the velocity pid controller
        #Setpoint is the desired valued
        self.vel_setpoint_pub = rospy.Publisher('/velocity/setpoint', Float64, queue_size=10)
        self.steering_setpoint_pub = rospy.Publisher('/steering/setpoint', Float64, queue_size=10)

        #State is the measured value
        self.vel_state_pub = rospy.Publisher('/velocity/state', Float64, queue_size=10)
        self.steering_state_pub = rospy.Publisher('/steering/state', Float64, queue_size=10)

        #Subscribe to the control output from the PID controllers
        rospy.Subscriber('/velocity/control_effort', Float64, self._velocity_control_callback)
        rospy.Subscriber('/steering/control_effort', Float64, self._steering_control_callback)

    def _velocity_control_callback(self, control_effort_msg):
        '''
        Callback for the velocity control output of the pid.

        Parameters:
            control_effort_msg: Message of type Float64
        Returns:
            N/A
        '''
        self.vel_control = control_effort_msg.data

    def _steering_control_callback(self, control_effort_msg):
        '''
        Callback for the steering control output of the pid.

        Parameters:
            control_effort_msg: Message of type Float64
        Returns:
            N/A
        '''
        self.steering_control = control_effort_msg.data


    def _measured_odom_data_callback(self, measured_odom_msg):
        '''
        Callback function for the measured odometry data estimated.
        Unpack the data

        Parameters:
            measured_odom_msg: Odometry data message type geometry_msgs/Twist
        Returns:
            N/A
        '''
        self.measured_odom = measured_odom_msg
        self.measured_vel_x = self.measured_odom.twist.twist.linear.x

        #Orientation is the direction the robot faces
        self.measured_orientation = self.measured_odom.twist.twist.angular.z


    def _desired_odom_data_callback(self, measured_odom_msg):
        '''
        Callback function for the desired odometry for the robot to hold.
        Unpack the data

        Parameters:
            desired_odom_msg: Odometry data message type geometry_msgs/Twist
        Returns:
            N/A
        '''
        self.desired_odom = measured_odom_msg
        self.desired_vel_x = self.desired_odom.twist.twist.linear.x

        #Orientation is the direction the robot faces
        self.desired_orientation = self.desired_odom.twist.twist.angular.z

    def map_control_efforts_to_pwms(self, vel_control_effort, steering_control_effort):
        '''
        Maps the control efforts for velocity and steering to individual motor
        PWMS.

        Parameters:
            vel_control_effort: Velocity Control effort output from PID.
            steering_control_effort: sterring control effor output from PID
        Returns:
            pwms: [pwm_1, pwm_2, pwm_3, pwm_4]
        '''
        vel_control = self.vel_control
        steering_control = self.steering_control

        pwm_1 = vel_control - steering_control
        pwm_2 = vel_control + steering_control
        pwm_3 = vel_control + steering_control
        pwm_4 = vel_control - steering_control

        return [pwm_1, pwm_2, pwm_3, pwm_4]
    def run(self):
        '''
        Run the main loop of the movement controller. Receive desired and
        measured odometry data and control.

        Parameters:
            N/A
        Returns:
            N/A
        '''

        try:
            while not rospy.is_shutdown():

                #Publish to the PID Controllers
                self.vel_setpoint_pub.publish(self.desired_vel_x)
                self.vel_state_pub.publish(self.measured_vel_x)

                self.steering_setpoint_pub.publish(self.desired_orientation)
                self.steering_state_pub.publish(self.measured_orientation)

                #Take the control effort output from PID controller and map to
                #PID's of individual motors
                motor_pwms = self.map_control_efforts_to_pwms(self.vel_control, self.steering_control)

                self.pwm_msg.data = motor_pwms
                self.pwm_pub.publish(self.pwm_msg)

                #100Hz
                self.pid_timer.sleep()

        except rospy.ROSInterruptException:
            pass

if __name__ == "__main__":
    movement_controller = Movement_Controller()
    movement_controller.run()
