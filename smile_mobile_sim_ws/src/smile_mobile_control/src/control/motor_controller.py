#!/usr/bin/env python
'''
Author: David Pierce Walker-Howell<piercedhowell@gmail.com>
Date Created: 05/24/2020
Description: This python module controls the motors of the simulated smile-mobile
        robot within Gazebo.
'''
import rospy
from std_msgs.msg import Float64, Int16MultiArray

class Motor_Controller:
    """
    Controls the simulated smile-mobile robot's motors given PWM values for each
    motor.
    """

    def __init__(self, node_name="motor_controller"):
        '''
        Initialize the motor controller publishers/subscribers.

        Parameters:
            node_name: A string for the name of this ros node. Default: motor_controller
        Returns:
            N/A
        '''
        rospy.init_node(node_name)

        #TOPICS - This format of name space is given for the ability to simulate
        #multiple robots
        motor_1_command_topic = rospy.get_namespace() + "motor_1/command"
        motor_2_command_topic = rospy.get_namespace() + "motor_2/command"
        motor_3_command_topic = rospy.get_namespace() + "motor_3/command"
        motor_4_command_topic = rospy.get_namespace() + "motor_4/command"

        #Maximum motor rotation speed (radians per second)
        self.max_motor_speed = 30.0

        #Publishers for each individual motor.
        #motor_1 => left_front_motor
        #motor_2 => right_front_motor
        #motor_3 => right_back_motor
        #motor_4 => left_back_motor

        self.motor_1_pub = rospy.Publisher(motor_1_command_topic, Float64, queue_size=1)
        self.motor_2_pub = rospy.Publisher(motor_2_command_topic, Float64, queue_size=1)
        self.motor_3_pub = rospy.Publisher(motor_3_command_topic, Float64, queue_size=1)
        self.motor_4_pub = rospy.Publisher(motor_4_command_topic, Float64, queue_size=1)

        self.motor_pub_rate = rospy.Rate(50) #50 Hz

        #Motor command msg to publish to each motor
        self.motor_msgs = [Float64() for i in range(4)]

        #Subscriber for the pwm messages (topic = pwm)
        rospy.Subscriber("/smile/pwm", Int16MultiArray, self._pwm_subscriber_callback)


    def _pwm_subscriber_callback(self, pwm_msg):
        '''
        Callback function for the pwm subscriber to receive pwm data. Convert
        the PWM signal to the simulated motor speed.

        Parameters:
            pwm_data: A message of type Int32MultiArray
        Returns:
            N/A
        '''
        #Convert the pwm messages to approximate motor radians per second
        for i in range(4):
            self.motor_msgs[i].data = (pwm_msg.data[i] / 255.0) * self.max_motor_speed

        return

    def run(self):
        '''
        Main loop controlling the simulated motors. Sends the joint velocity to
        the simulated gazebo robot

        Parameters:
            N/A
        Returns:
            N/A
        '''

        #Main loop
        try:
            while not rospy.is_shutdown():

                #Publish the motor controll command messages to each motor
                self.motor_1_pub.publish(self.motor_msgs[0])
                self.motor_2_pub.publish(self.motor_msgs[1])
                self.motor_3_pub.publish(self.motor_msgs[2])
                self.motor_4_pub.publish(self.motor_msgs[3])
                self.motor_pub_rate.sleep()

        except rospy.ROSInterruptException:
            pass

if __name__ == "__main__":
    motor_controller = Motor_Controller()
    motor_controller.run()
