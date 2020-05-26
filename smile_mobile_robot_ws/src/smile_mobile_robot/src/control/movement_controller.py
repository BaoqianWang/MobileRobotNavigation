#!/usr/bin/env python
'''
Author: David Pierce Walker-Howeell<piercedhowell@gmail.com>
Date Created: 05/20/2020
Description: This python module controls the movement of the robot using
              pid controllers.
'''
import rospy
import rosparam
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64, Int16MultiArray, Float32MultiArray
from nav_msgs.msg import Odometry
from tf.transformations import quaternion_from_euler, euler_from_quaternion
import time
from pid_controller import PID_Controller
from smile_mobile_robot.srv import pid_gains, pid_gainsResponse

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

        #TOPICS - This format of name space is given for ability to simulate multiple
        #by providing different names

        measured_odom_topic = rospy.get_namespace() + "raw/odometry"
        pwm_topic = rospy.get_namespace() + "pwm"
        desired_movement_topic = rospy.get_namespace() + "desired/movement"

        #Initialize subscriber for raw odometry
        rospy.Subscriber(measured_odom_topic, Odometry, self._odom_data_callback)

        #Initialize subscriber for desired_odometry
        rospy.Subscriber(desired_movement_topic, Float32MultiArray, self._desired_movement_callback)

        #Initialize PWM controllers
        self._initialize_pid_controlers()

        self.pwm_pub = rospy.Publisher(pwm_topic, Int16MultiArray, queue_size=10)
        self.pwm_msg = Int16MultiArray()

        self.pid_timer = rospy.Rate(100) #100Hz

        #Initialize publisher for writing PWM
        self.measured_velocity = 0.0
        self.desired_velocity= 0.0
        self.measured_orientation = 0.0
        self.desired_orientation = 0.0
        self.velocity_control = 0.0
        self.steering_control = 0.0

    def _initialize_pid_controlers(self):
        '''
        Initialize communication with the pid controllers.

        Parameters:
            N/A
        Returns:
            N/A
        '''

        pid_params = rospy.get_param("/pid")
        velocity_pid_params = pid_params['velocity']
        steering_pid_params = pid_params['steering']

        velocity_k_p = velocity_pid_params['k_p']
        velocity_k_i = velocity_pid_params['k_i']
        velocity_k_d = velocity_pid_params['k_d']

        self.velocity_pid_controller = PID_Controller(k_p=velocity_k_p,
                                                      k_i=velocity_k_i,
                                                      k_d=velocity_k_d,
                                                      max_control_effort=255,
                                                      min_control_effort=-255)

        steering_k_p = steering_pid_params['k_p']
        steering_k_i = steering_pid_params['k_i']
        steering_k_d = steering_pid_params['k_d']
        self.steering_pid_controller = PID_Controller(k_p=steering_k_p,
                                                      k_i=steering_k_i,
                                                      k_d=steering_k_d,
                                                      max_control_effort=100,
                                                      min_control_effort=-100,
                                                      angle_error=True)
        #Initialize the service for updating the pid controller gains
        rospy.Service('update_pid_gains', pid_gains, self.handle_pid_gains_update)

        return

    def handle_pid_gains_update(self, req):
        '''
        Handler function for when a PID gain upate is requested. This will update
        the PIDS in the parameter server.

        Parameters:
            reg: The request message containing the desired gains
        Returns:
            N/A
        '''
        self.velocity_pid_controller.set_gains(req.velocity_k_p, req.velocity_k_i, req.velocity_k_d)
        self.steering_pid_controller.set_gains(req.steering_k_p, req.steering_k_i, req.steering_k_d)

        #Update the parameter server on the correct pid values
        rospy.set_param("/pid/velocity", {'k_p': req.velocity_k_p, 'k_i': req.velocity_k_i, 'k_d': req.velocity_k_d})
        rospy.set_param("/pid/steering", {'k_p': req.steering_k_p, 'k_i': req.steering_k_i, 'k_d': req.steering_k_d})


        return pid_gainsResponse()
        
    def _odom_data_callback(self, odom_msg):
        '''
        Callback function for the measured odometry data estimated.
        Unpack the data

        Parameters:
            measured_odom_msg: Odometry data message type nav_msgs/Odometry
        Returns:
            N/A
        '''
        self.measured_odom = odom_msg
        self.measured_velocity= self.measured_odom.twist.twist.linear.x

        #Orientation is the direction the robot faces
        #Convert from quaternion to euler
        quaternion = [self.measured_odom.pose.pose.orientation.x,
                      self.measured_odom.pose.pose.orientation.y,
                      self.measured_odom.pose.pose.orientation.z,
                      self.measured_odom.pose.pose.orientation.w]
        [roll, pitch, yaw] = euler_from_quaternion(quaternion)

        self.measured_orientation = yaw


    def _desired_movement_callback(self, desired_movement_msg):
        '''
        Callback function for the desired movement for the robot to hold. The
        message contains the desired velocity and orientation.

        Parameters:
            desired_movement_msg: The desired velocity (m/s) and orientation (rad/s).
                                  The message is of type std_msgs/Float32MultiArray.
                                  [desired_velocity, desired_orientation]
        Returns:
            N/A
        '''
        self.desired_velocity = desired_movement_msg.data[0]

        #Orientation is the direction the robot faces
        self.desired_orientation = desired_movement_msg.data[1]

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
        vel_control = vel_control_effort
        steering_control = steering_control_effort

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

                #Take the control effort output from PID controller and map to
                #PID's of individual motor
                self.velocity_control, _ = self.velocity_pid_controller.update(self.desired_velocity, self.measured_velocity)
                self.steering_control, _ = self.steering_pid_controller.update(self.desired_orientation, self.measured_orientation)
                motor_pwms = self.map_control_efforts_to_pwms(self.velocity_control, self.steering_control)

                self.pwm_msg.data = motor_pwms
                self.pwm_pub.publish(self.pwm_msg)

                #100Hz
                self.pid_timer.sleep()

        except rospy.ROSInterruptException:
            pass

if __name__ == "__main__":
    movement_controller = Movement_Controller()
    movement_controller.run()
