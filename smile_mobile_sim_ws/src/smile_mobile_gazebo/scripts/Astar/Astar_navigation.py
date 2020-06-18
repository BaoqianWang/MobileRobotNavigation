'''
Author: BaoqianWang<wbq7758258@gmail.com>
Date Created: 05/26/2020
Description: This python module implements Astar navigation algorithm
'''

import rospy, tf
import geometry_msgs.msg, nav_msgs.msg
from math import *
import numpy as np
from time import sleep
import PlannerNavigation
import time
import json
from std_msgs.msg import Float32MultiArray
#import pyrr


class AstarNavigation:
    '''
    AstarNavigation algorithm
    '''

    def __init__(self, initial_pos, node_name="astar_navigation"):
        rospy.init_node(node_name)
        self.pose= [initial_pos[0], initial_pos[1], initial_pos[2]]
        self.ini_pos=initial_pos

    def huskyOdomCallback(self,message):
        # Implementation of proportional position control

        # Generate a simplified pose
        pos = message.pose.pose
        quat = pos.orientation
        # From quaternion to Euler
        angles = tf.transformations.euler_from_quaternion((quat.x,quat.y,
                                                   quat.z,quat.w))
        theta = angles[2]


        self.pose = [self.ini_pos[0]+pos.position.x, self.ini_pos[1]+pos.position.y, theta]  # X, Y, Theta

        # Reporting
        #print('huskyOdomCallback: x=%4.1f,y=%4.1f dist=%4.2f, cmd.v=%4.2f, cmd.w=%4.2f'%(self.pose[0],self.pose[1]))


    def run(self):


        #start = np.array([0, 0, 0])
        #goal = np.array([15, 8.0, 0.0])
        #path=self.planning_path(map_fname,start,goal)

        cmdmsg = Float32MultiArray()
        cmdpub = rospy.Publisher('/smile/desired/movement',Float32MultiArray, queue_size=10)
        rospy.Subscriber('/smile/raw/odometry',nav_msgs.msg.Odometry,self.huskyOdomCallback)
        filename='path_database.txt'
        with open(filename, 'r') as fd:
            database=json.load(fd)
        path=database[0]
        print('The planned path is', path)
        # Tunable parameters
        wgain = 1.8 # Gain for the angular velocity [rad/s / rad]
        vconst = 1 # Linear velocity when far away [m/s]
        distThresh = 1# Distance treshold [m]

        # Proportional Controller


        for goal_x, goal_y in reversed(zip(path['x'],path['y'])):
            v = 0 # default linear velocity
            u = 0 # default angluar velocity
            print('Goal is', goal_x, goal_y)
            print('Current position: x=%4.1f,y=%4.1f'%(self.pose[0],self.pose[1]))
            distance = sqrt((self.pose[0]-goal_x)**2+(self.pose[1]-goal_y)**2)
            while distance>distThresh:
                #print('Current position: x=%4.1f,y=%4.1f'%(self.pose[0],self.pose[1]))
                distance = sqrt((self.pose[0]-goal_x)**2+(self.pose[1]-goal_y)**2)
                v = vconst
                desireYaw = atan2(goal_y-self.pose[1],goal_x-self.pose[0])
                u = desireYaw#-self.pose[2]
                bound = atan2(sin(u),cos(u))
                # Publish
                cmdmsg.data = [v, u]
                cmdpub.publish(cmdmsg)
                print('Pose&Control: x=%4.1f,y=%4.1f dist=%4.2f, cmd.v=%4.2f, cmd.u=%4.2f'%(self.pose[0],self.pose[1],distance,v,u))
        print('Third', v,u)
        cmdmsg.data = [0, u]
        cmdpub.publish(cmdmsg)
        rospy.spin()

if __name__ == "__main__":
    ini_pos=[-10, 22, 0]
    astar_navigation = AstarNavigation(ini_pos)
    astar_navigation.run()
