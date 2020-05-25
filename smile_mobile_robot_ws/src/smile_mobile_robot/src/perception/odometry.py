#!/usr/bin/env python
'''
Author: David Pierce Walker-Howell<piercedhowell@gmail.com>
Date Created: 05/20/2020
Description: Uses the IMU and GPS to determine the odometry of the robot.
'''
import rospy
from sensor_msgs.msg import Imu
from std_msgs.msg import Int16MultiArray
from geometry_msgs.msg import Twist

class Odometry:
    """
    Subscribe to the imu data and GPS data to be able to determine the
    odometry of the vehicle
    """

    def __init__(self, node_name="estimate_odometry"):
        '''
        Initialize the estimate of odometry node.

        Parameters:
            node_name: Name of rosnode. Default: estimate_odometry

        Returns:
            N/A
        '''
        self.node_name = node_name
        rospy.init_node(node_name)

        #Subscriber to imu
        rospy.Subscriber("/imu/data", Imu, self._imu_data_subscriber_callback)

        #Publisher of estimated odometry data
        self.odom_pub = rospy.Publisher('/measured_odom', Twist, queue_size=10)
        self.odom_msg = Twist()
        self.odom_pub_rate = rospy.Rate(100)

        self.imu_data = Imu()
        self.linear_accel = [0, 0, 0]
        self.orientation = [0, 0, 0]
        self.gyro = [0, 0, 0]
        self.imu_timer_start = 0
        self.imu_timer_end = 0

        #Previous and current calculated x velocity
        self.prev_vel_x = 0
        self.curr_vel_x = 0

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
        #Accelerometer
        self.linear_accel = [self.imu_data.linear_acceleration.x,
                             self.imu_data.linear_acceleration.y,
                             self.imu_data.linear_acceleration.z]

        #Magnetometer readings
        self.orientation = [self.imu_data.orientation.x,
                            self.imu_data.orientation.y,
                            self.imu_data.orientation.z]

        #Gyroscope readings
        self.gyro = [self.imu_data.angular_velocity.x,
                     self.imu_data.angular_velocity.y,
                     self.imu_data.angular_velocity.z]

        #Get distance in-between velocity calculations
        #Get time in nano seconds
        self.imu_timer_end = (rospy.Time.now()).nsecs
        imu_dt = (self.imu_timer_end - self.imu_timer_start) / (1.0e9)

        #Estimate the linear velocity by integrating the linear acceleration
        self.curr_vel_x = \
                    self._estimate_velocity(self.linear_accel[0], \
                        self.prev_vel_x, imu_dt)

        self.prev_vel_x = self.curr_vel_x
        #Restart timer
        self.imu_timer_start = (rospy.Time.now()).nsecs


    def _estimate_velocity(self, linear_accel_x, prev_vel_x, dt):
        '''
        Estimate instaneous linear x velocity from the acceleration measured in
        the linear x direction.

        Parameters:
            linear_accel_x: The linear acceleration in the x-direction
            dt: The time interval for updating the velocity
        Returns:
            linear_vel_x_estimate: The velocity estimate
        '''
        return prev_vel_x + (linear_accel_x * dt)

    def run(self):
        '''
        Run the main loop for determining odometry.

        Parameters:
            N/A
        Returns:
            N/A
        '''

        try:
            while not rospy.is_shutdown():
                #Pack up the odometry message
                self.odom_msg.linear.x = self.curr_vel_x
                self.odom_msg.linear.y = 0
                self.odom_msg.linear.y = 0
                self.odom_msg.angular.x = 0
                self.odom_msg.angular.y = 0
                self.odom_msg.angular.z = 0

                #Publish the odometry message
                self.odom_pub.publish(self.odom_msg)

                self.odom_pub_rate.sleep()

        except rospy.ROSInterruptException:
            pass

if __name__ == "__main__":
    odom = Odometry()
    odom.run()
