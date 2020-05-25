#!/usr/bin/env python
'''
Author: David Pierce Walker-Howell<piercedhowell@gmail.com>
Date Created: 05/24/2020
Description: The python module mimics the encoder sensors of for the simulated
        smile-mobile robot.
'''
import rospy
from std_msgs.msg import Float64, Float32MultiArray
from sensor_msgs.msg import JointState

class Encoders:
    """
    Reads the joint states of the motors and publishes the angular frequency
    of each motor.
    """

    def __init__(self, node_name="encoders"):
        '''
        Initialize the communication with the simulated smile-robot to get joint
        states.

        Parameters:
            node_name: A string for the name of this ros node. Default: encoders
        '''
        rospy.init_node(node_name)

        #TOPICS - This format of name space is given for the ability to simulate
        #multiple robots
        joint_states_topic = rospy.get_namespace() + "joint_states"
        encoders_topic = rospy.get_namespace() + "encoders"

        #Subscriber to the motor joint states of the smile-mobile robot sim.
        rospy.Subscriber(joint_states_topic, JointState, self._joint_state_callback)

        #Encoder data measured in rad / s
        #[motor_1, motor_2, motor_3, motor_4]
        self.encoders_msg = Float32MultiArray()
        self.encoders_msg.data = [0.0, 0.0, 0.0, 0.0]

        self.encoders_pub_rate = rospy.Rate(50)

        #Publisher to publish the motor rotary encoder data
        self.encoders_pub = rospy.Publisher(encoders_topic, Float32MultiArray, queue_size=10)


    def _joint_state_callback(self, joint_state_msg):
        '''
        Callback function for the motor joint states of the simulated smile robot.

        Parameters:
            joint_state_msg: A message of type sensor_msgs/JointState.

        Returns:
            N/A
        '''
        for i in range(4):
            self.encoders_msg.data[i] = joint_state_msg.velocity[i]

        self.encoders_msg.data[0] = joint_state_msg.velocity[1]
        self.encoders_msg.data[1] = joint_state_msg.velocity[3]
        self.encoders_msg.data[2] = joint_state_msg.velocity[2]
        self.encoders_msg.data[3] = joint_state_msg.velocity[0]

    def run(self):
        '''
        The main loop to run for the encoder module.

        Parameters:
            N/A
        Returns:
            N/A
        '''

        try:

            while not rospy.is_shutdown():

                #Publih the encoder data
                self.encoders_pub.publish(self.encoders_msg)
                self.encoders_pub_rate.sleep()

        except rospy.ROSInterruptException:
            pass

if __name__ == "__main__":
    encoders = Encoders()
    encoders.run()
