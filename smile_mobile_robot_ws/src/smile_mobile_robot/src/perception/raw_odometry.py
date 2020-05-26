#!/usr/bin/env python
'''
Author: David Pierce Walker-Howell<piercedhowell@gmail.com>
Date Created: 05/25/2020
Description: Use the encoders and magnetometer to get a raw odometery estimation.
'''
import rospy
from sensor_msgs.msg import Imu
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import numpy as np
from tf.transformations import quaternion_from_euler, euler_from_quaternion

class Raw_Odometry:
    """
    Subscribe to the imu data and GPS data to be able to determine the
    odometry of the vehicle
    """

    def __init__(self, node_name="raw_odometry"):
        '''
        Initialize the estimate of odometry node.

        Parameters:
            node_name: Name of rosnode. Default: estimate_odometry

        Returns:
            N/A
        '''
        self.node_name = node_name
        rospy.init_node(node_name)

        #TOPICS - This format of name space is given for the ability to simulate
        #multiple robots
        imu_data_topic = rospy.get_namespace() + "imu"
        encoder_data_topic = rospy.get_namespace() + "encoders"
        raw_odom_topic = rospy.get_namespace() + "raw/odometry"

        #Subscriber to imu
        rospy.Subscriber(imu_data_topic, Imu, self._imu_data_subscriber_callback)

        #Subscriber to the encoders
        rospy.Subscriber(encoder_data_topic, Float32MultiArray, self._encoders_subscriber_callback)

        #Publisher of estimated odometry data
        self.raw_odom_pub = rospy.Publisher(raw_odom_topic, Odometry, queue_size=10)
        self.raw_odom_msg = Odometry()
        self.loop_frequency = 100
        self.dt = 1.0 / self.loop_frequency
        self.raw_odom_pub_rate = rospy.Rate(self.loop_frequency)

        self.imu_data = Imu()
        self.encoders_data = [0.0, 0.0, 0.0, 0.0]

        #Get the vehicle parameters from the parameter server
        self.vehicle_params = rospy.get_param("/vehicle_params")
        self.wheel_radius = self.vehicle_params['wheel_radius']
        self.vehicle_width = self.vehicle_params['vehicle_width']

        #Initialize odometry Parameters
        self.velocity = 0.0
        self.prev_pose = np.array([0.0, 0.0])
        self.curr_pose = np.array([0.0, 0,0])
        self.phi = 0.0

    def _imu_data_subscriber_callback(self, imu_data_msg):
        '''
        Callback function for the imu data comming from the robot.
        Unpack the data.

        Parameters:
            imu_data_msg: imu data of message type sensor_msgs/Imu
        Returns:
            N/A
        '''

        self.imu_data = imu_data_msg
        [x, y, z, w] = [self.imu_data.orientation.x, self.imu_data.orientation.y, self.imu_data.orientation.z, self.imu_data.orientation.w]
        [roll, pitch, yaw] = euler_from_quaternion([x, y, z, w])
        self.phi = yaw


    def _encoders_subscriber_callback(self, encoders_data_msg):
        '''
        Callback function for the encoders data coming from each wheel
        of the robot

        Parameters:
            encoders_data_msg: Encoders data for each motor. Type std_msgs/Float32MultiArray
        Returns:
            N/A
        '''
        #Extrac the enocers data.
        for i in range(4):
            self.encoders_data[i] = encoders_data_msg.data[i]

    def _estimate_odometry_encoders(self, encoders, wheel_radius, phi, prev_pose, dt):
        '''
        Estimates the odometry of the robot with the encoders and magnetometer.
        Provides the vehicles local forwar velocity, global orientation and global
        position.

        Parameters:
            encoders: A list of the raw encoder reading in order of [motor_1, motor_2, motor_3, motor_4]
            wheel_radius: The radius of the wheel.
            phi: The global angle orientation (aka: yaw)
            prev_pose: The previous global position: np.array([prev_x_pose, prev_y_pose])
            dt: The time since the last calculation.
        Returns:
            velocity: The instaneous velocity of the vehicle. Pointing forward.
            curr_pose: The current position from dead reckoning algorithm. np.array([curr_x_pose, curr_y_pose])
        '''
        #Estimate the instaneous velocity of the vehicle from the encoders

        velocity_left = ((encoders[0] + encoders[3]) / 2.0) * wheel_radius
        velocity_right = ((encoders[1] + encoders[2]) / 2.0) * wheel_radius

        #The linear velocity is the average of the left sides velocity
        #and right sides velocity.
        velocity = (velocity_left + velocity_right) / 2.0

        #curr_pose = prev_pose + (velocity * np.array((np.cos(phi), np.sin(phi))) * dt)
        curr_pose = np.array([0.0, 0.0])
        curr_pose[0] = prev_pose[0] + (velocity * np.cos(phi) * dt)
        curr_pose[1] = prev_pose[1] + (velocity * np.sin(phi) * dt )
        return velocity, curr_pose

    def run(self):
        '''
        Run the main loop for determining odometry.

        Parameters:
            N/A
        Returns:
            N/A
        '''
        timer_end = 0.0
        timer_start = 0.0
        try:
            while not rospy.is_shutdown():

                #Calculate the current velocity and the current updated position. Dead Reckoning.
                self.velocity, self.curr_pose = self._estimate_odometry_encoders(self.encoders_data,
                                                    self.wheel_radius, self.phi, self.prev_pose, self.dt)
                self.prev_pose = np.copy(self.curr_pose)

                #Pack and publish the estimated odometry data
                self.raw_odom_msg.pose.pose.position.x = self.curr_pose[0]
                self.raw_odom_msg.pose.pose.position.y = self.curr_pose[1]
                self.raw_odom_msg.pose.pose.position.z = 0.0

                #Convert euler positions to quaternion
                quaternion = quaternion_from_euler(0.0, 0.0, self.phi)

                self.raw_odom_msg.pose.pose.orientation.x = quaternion[0]
                self.raw_odom_msg.pose.pose.orientation.y = quaternion[1]
                self.raw_odom_msg.pose.pose.orientation.z = quaternion[2]
                self.raw_odom_msg.pose.pose.orientation.w = quaternion[3]

                self.raw_odom_msg.twist.twist.linear.x = self.velocity

                #TODO: Set Covariances

                #Publish the data to the raw odometry topic
                self.raw_odom_pub.publish(self.raw_odom_msg)
                self.raw_odom_pub_rate.sleep()

        except rospy.ROSInterruptException:
            pass

if __name__ == "__main__":
    raw_odom = Raw_Odometry()
    raw_odom.run()
