#!/usr/bin/env python
'''
Author: David Pierce Walker-Howell<piercedhowell@gmail.com>
Date Created:06/06/2020
Description: Control the robot to follow the simple lanes. Receives lane
             detections from the lane detector node.
'''
import rospy
from std_msgs.msg import Float32MultiArray
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
from pid_controller import PID_Controller
import math
from smile_mobile_robot.srv import bool_call, bool_callResponse

class Lane_Follower:
    '''
    Control the robut to follow simple lanes.
    '''

    def __init__(self, node_name="lane_follower"):
        '''
        Initialize the lane follower.

        Parameters:
            node_name: The name of the lane follower node
        Returns:
            N/A
        '''
        rospy.init_node(node_name)

        #TOPICS
        lane_tracking_topic = rospy.get_namespace() + "lane_tracking"
        odom_topic = rospy.get_namespace() + "raw/odometry"
        desired_movement_topic = rospy.get_namespace() + "desired/movement"

        #Lane tracking data from subscriber. Data comes from the lane detection.
        #node.
        rospy.Subscriber(lane_tracking_topic, Float32MultiArray,
                            self._lane_tracking_callback)

        #Subscriber to the odometry data
        rospy.Subscriber(odom_topic, Odometry, self._odom_data_callback)

        #Publisher to the movement controller api
        self.desired_movement_pub = rospy.Publisher(desired_movement_topic,
                                                    Float32MultiArray, queue_size=1)


        #Initialize the pid controller for tracking the lane
        #TODO: Add parameters for lane tracking pid to config file
        param_path = '/vision_params/lane_follower/pid/'

        #Get the pid controller parameters from the parameter server
        k_p = rospy.get_param(param_path + 'k_p')
        k_i = rospy.get_param(param_path + 'k_i')
        k_d = rospy.get_param(param_path + 'k_d')
        integral_min = rospy.get_param(param_path + 'integral_min')
        integral_max = rospy.get_param(param_path + 'integral_max')
        max_control_effort = rospy.get_param(param_path + 'max_control_effort')
        min_control_effort = rospy.get_param(param_path + 'min_control_effort')

        self.lane_tracking_pid_controller = PID_Controller(k_p=k_p,
                                                           k_i=k_i,
                                                           k_d=k_d,
                                                           integral_min=integral_min,
                                                           integral_max=integral_max,
                                                           max_control_effort=max_control_effort,
                                                           min_control_effort=min_control_effort)

        #Service the enable or disable line following.
        #This can be called to enable or disable the control of the robot based
        #on line following
        rospy.Service('enable_line_following', bool_call, self._line_following_state)
        self.rate = rospy.Rate(10)

        #Initialize variables
        self.lane_detected = False
        self.camera_center_point = 0.0
        self.lane_center_point = 0.0

        self.lane_follow_enabled = True

        self.measured_velocity = 0.0
        self.measured_orientation = 0.0

        #Base velocity
        self.desired_velocity = 0.5

    def _line_following_state(self, req):
        '''
        The service callback to enable or disable line following.

        Parameters:
            req: The request message
        Returns:
            N/A
        '''
        self.lane_follow_enabled = req.val

        #Set the robots velocity to zero and hold the current orientation
        self.desired_movement_msg.data = [0.0, self.measured_orientation]
        self.desired_movement_pub.publish(self.desired_movement_msg)
        return bool_callResponse()

    def _lane_tracking_callback(self, lane_tracking_msg):
        '''
        Receive the lane tracking data msg. Unpack it

        Parameters:
            lane_tracking_msg. The lane tracking data message from the lane detection
                        node.
                        Format: [lane_detected_bool, set_point, measured_point]
        Returns:
            N/A
        '''
        ret = lane_tracking_msg.data[0]
        if(ret == 1.0):
            self.lane_detected = True

        else:
            self.lane_detected = False

        self.camera_center_point = lane_tracking_msg.data[1]
        self.lane_center_point = lane_tracking_msg.data[2]

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


    def run(self):
        '''
        Main loop for running the lane following.

        Parameters:
            N/A
        Returns:
            N/A
        '''
        self.desired_movement_msg = Float32MultiArray()
        try:

            while not rospy.is_shutdown():

                if(self.lane_follow_enabled):
                    #print(self.camera_center_point, self.lane_center_point)
                    #Get the pid control effort the determine how much to steer
                    control_effort, error = self.lane_tracking_pid_controller.update(\
                                    self.camera_center_point, self.lane_center_point)

                    #Translate the control effort into a yaw angle for the robot to
                    #face
                    adjustment_angle = self.measured_orientation + control_effort
                    if(adjustment_angle < -1*math.pi):
                        self.desired_orientation = 2.0*math.pi + adjustment_angle

                    elif(adjustment_angle > math.pi):
                        self.desired_orientation = adjustment_angle - 2.0*math.pi

                    else:
                        self.desired_orientation = adjustment_angle


                    #Write the velocity and orientation to the robot.
                    velocity = self.desired_velocity - abs(0.005 * error)
                    self.desired_movement_msg.data = [velocity, self.desired_orientation]
                    self.desired_movement_pub.publish(self.desired_movement_msg)
                else:
                    self.desired_movement_msg.data = [0.0, self.desired_orientation]



                self.rate.sleep()

        except rospy.ROSInterruptException:
            pass

if __name__ == "__main__":
    lane_follower = Lane_Follower()
    lane_follower.run()
