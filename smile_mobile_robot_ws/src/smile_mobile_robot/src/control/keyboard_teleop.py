#! /usr/bin/env python
'''
Author: David Pierce Walker-Howell<piercedhowell@gmail.com>
Date Created: 06/03/2020
Description: Provide an interface for controlling the smile mobile robot with keyboard
            teleop.
'''
import rospy
from std_msgs.msg import Int16MultiArray
from pynput import keyboard
import sys
import numpy as np

class Keyboard_Teleop:
    '''
    Interface command to the robot for controlling the PWM of each motor 
    with keyboard commands.
    '''
    def __init__(self, node_name="keyboard_teleop"):
        '''
        Initialize the keyboard teleop control.

        Parameters:
            node_name: The name of the keyboard teleop node. Default: keyboard_teleop

        Returns:
            N/A
        '''
        self.node_name = node_name
        rospy.init_node(node_name)

        #TOPICS - This formate of name space is given for ability to simulate multiple robots
        #by providing different names.

        pwm_topic = rospy.get_namespace() + "pwm"

        #Initialize the publisher to the pwm topic

        self.pwm_pub = rospy.Publisher(pwm_topic, Int16MultiArray, queue_size=1)
        self.pwm_msg = Int16MultiArray()
        
        self.pub_timer = rospy.Rate(10)
        
        #Initialize the keyboard listener
        self.listener = keyboard.Listener(
                            on_press=self.on_press,
                            on_release=self.on_release)

        self.linear_pwm = 0
        self.angular_pwm = 0
        self.linear_go = 0
        self.angular_go = 0

        #Preset values (min = 0, max = 255)!
        self.set_linear_pwm_perc = 0.1
        self.set_angular_pwm_perc = 0.4
        self.set_linear_pwm = int(self.set_linear_pwm_perc * 255)
        self.set_angular_pwm = int(self.set_angular_pwm_perc * 255)


    def on_press(self, key):
        '''
        Callback function for the keyboard listener to identify which key was
        pressed.

        Parameters:
            key:
        Returns:
            N/A
        '''
        sys.stdout.write("\r    ")
        sys.stdout.flush()


        #Ensure w and s aren't read at the same time
        if(key.char == 'w'): #Forward movement
            self.linear_go = 1
        elif(key.char == 's'):
            self.linear_go = -1

        if(key.char == 'a'):#Left turns
            self.angular_go = 1

        elif(key.char == 'd'): #Right turns
            self.angular_go = -1
        
        if(key.char == 'i'): #Increase the throttle
            if(self.set_linear_pwm_perc < 0.9):
                self.set_linear_pwm_perc += 0.1
                self.set_linear_pwm = int(self.set_linear_pwm_perc * 255)

        elif(key.char == 'j'): #Decrease the throttle
            if(self.set_linear_pwm_perc > 0.1):
                self.set_linear_pwm_perc -= 0.1
                self.set_linear_pwm = int(self.set_linear_pwm_perc * 255)
                
    
    def on_release(self, key):
        '''
        Callback function for the keyboard listener to identify which key was
        released.

        Parameters:
            key:
        Returns:
            N/A
        '''
        if(key.char == 'w' or key.char == 's'):
            self.linear_go = 0

        elif(key.char == 'a' or key.char == 'd'):
            self.angular_go = 0
        
    
    def run(self):
        '''
        Main loop to read the keyboard inputs and publish them as pwm values.

        Parameters:
            N/A
        Returns:
            N/A
        '''
        self.listener.start()
        angular_mean_filter = np.array([0, 0, 0, 0, 0, 0, 0, 0, 0, 0])
        linear_mean_filter = np.array([0, 0, 0, 0, 0, 0, 0, 0, 0, 0])


        try:
            while not rospy.is_shutdown():
                
                #mix the linear and angular pwm's to get the pwm of each motor
                if(self.linear_go == -1):
                    self.linear_pwm = -1 * self.set_linear_pwm
                elif(self.linear_go == 1):
                    self.linear_pwm = self.set_linear_pwm
                else:
                    self.linear_pwm = 0
               
                if(self.angular_go == -1):
                    self.angular_pwm = -1 * self.set_angular_pwm
                elif(self.angular_go == 1):
                    self.angular_pwm = self.set_angular_pwm
                else:
                    self.angular_pwm = 0
                
                #Run the mean average filter to ensure values don't change too fast
                angular_mean_filter = np.delete(angular_mean_filter, 0); 
                angular_mean_filter = np.append(angular_mean_filter, self.angular_pwm)
                linear_mean_filter = np.delete(linear_mean_filter, 0); 
                linear_mean_filter = np.append(linear_mean_filter, self.linear_pwm)
                angular_pwm_filt = np.mean(angular_mean_filter)
                linear_pwm_filt = np.mean(linear_mean_filter)

                pwm_1 = linear_pwm_filt - angular_pwm_filt
                pwm_2 = linear_pwm_filt + angular_pwm_filt
                pwm_3 = linear_pwm_filt + angular_pwm_filt
                pwm_4 = linear_pwm_filt - angular_pwm_filt

                pwms = [pwm_1, pwm_2, pwm_3, pwm_4]
                
                #Bound the pwms if necessary
                for i in range(4):
                    if(pwms[i] < -255):
                        pwms[i] = -255
                    elif(pwms[i] > 255):
                        pwms[i] = 255
                
                #Publish the pwm to the pwm topic
                self.pwm_msg.data = pwms
                self.pwm_pub.publish(self.pwm_msg)
                
                self.pub_timer.sleep()

        except rospy.ROSInterruptException:
            pass

if __name__ == "__main__":


    keyboard_teleop = Keyboard_Teleop()
    keyboard_teleop.run()


