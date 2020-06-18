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
#import pyrr


class AstarNavigation:
    '''
    AstarNavigation algorithm
    '''

    def __init__(self, node_name="astar_navigation"):
        rospy.init_node(node_name)
        self.pose= [0, 0, 0]

    def tic(self):
      return time.time()

    def toc(self,tstart, nm=""):
      print('%s took: %s sec.\n' % (nm,(time.time() - tstart)))


    def load_map(self,fname):
      '''
      Loads the bounady and blocks from map file fname.

      boundary = [['xmin', 'ymin', 'zmin', 'xmax', 'ymax', 'zmax','r','g','b']]

      blocks = [['xmin', 'ymin', 'zmin', 'xmax', 'ymax', 'zmax','r','g','b'],
                ...,
                ['xmin', 'ymin', 'zmin', 'xmax', 'ymax', 'zmax','r','g','b']]
      '''
      mapdata = np.loadtxt(fname,dtype={'names': ('type', 'xmin', 'ymin', 'zmin', 'xmax', 'ymax', 'zmax','r','g','b'),\
                                        'formats': ('S8','f', 'f', 'f', 'f', 'f', 'f', 'f','f','f')})
      blockIdx = mapdata['type'] == b'block'
      boundary = mapdata[~blockIdx][['xmin', 'ymin', 'zmin', 'xmax', 'ymax', 'zmax','r','g','b']]#.view('<f4').reshape(-1,11)[:,2:]
      blocks = mapdata[blockIdx][['xmin', 'ymin', 'zmin', 'xmax', 'ymax', 'zmax','r','g','b']]#.view('<f4').reshape(-1,11)[:,2:]
      return boundary, blocks

    def planning_path(self,mapfile,start,goal):
        resolution=0.5
        boundary, blocks = self.load_map(mapfile)
        MP = PlannerNavigation.MyPlanner(boundary, blocks, resolution) # TODO: replace this with your own planner implementation

        # Call the motion planner
        t0 = self.tic()
        rx, ry, rz = MP.plan(start, goal)
        self.toc(t0,"Planning")

        #Concatenate the path
        path=zip(rx,ry)

        return path

    def huskyOdomCallback(self,message):
        # Implementation of proportional position control

        # Generate a simplified pose
        pos = message.pose.pose
        quat = pos.orientation
        # From quaternion to Euler
        angles = tf.transformations.euler_from_quaternion((quat.x,quat.y,
                                                   quat.z,quat.w))
        theta = angles[2]


        self.pose = [pos.position.x, pos.position.y, theta]  # X, Y, Theta

        # Reporting
        #print('huskyOdomCallback: x=%4.1f,y=%4.1f dist=%4.2f, cmd.v=%4.2f, cmd.w=%4.2f'%(self.pose[0],self.pose[1]))


    def run(self):

        map_fname="./maps/boundary_boxes.txt"
        start = np.array([0, 0, 0])
        goal = np.array([15, 8.0, 0.0])
        #path=self.planning_path(map_fname,start,goal)

        cmdmsg = geometry_msgs.msg.Twist()
        cmdpub = rospy.Publisher('/husky_velocity_controller/cmd_vel',geometry_msgs.msg.Twist, queue_size=10)
        rospy.Subscriber('/odometry/filtered',nav_msgs.msg.Odometry,self.huskyOdomCallback)
        filename='path_database.txt'
        with open(filename, 'r') as fd:
            database=json.load(fd)
        path=database[0]
        print('The planned path is', path)
        # Tunable parameters
        wgain = 1.8 # Gain for the angular velocity [rad/s / rad]
        vconst = 1.3 # Linear velocity when far away [m/s]
        distThresh = 0.4 # Distance treshold [m]

        # Proportional Controller
        v = 0 # default linear velocity
        w = 0 # default angluar velocity

        for goal_x, goal_y in reversed(zip(path['x'],path['y'])):
            print('Goal is', goal_x, goal_y)
            #print('Current position: x=%4.1f,y=%4.1f'%(self.pose[0],self.pose[1]))
            distance = sqrt((self.pose[0]-goal_x)**2+(self.pose[1]-goal_y)**2)
            while distance>distThresh:
                #print('Current position: x=%4.1f,y=%4.1f'%(self.pose[0],self.pose[1]))
                distance = sqrt((self.pose[0]-goal_x)**2+(self.pose[1]-goal_y)**2)
                v = vconst
                desireYaw = atan2(goal_y-self.pose[1],goal_x-self.pose[0])
                u = desireYaw-self.pose[2]
                bound = atan2(sin(u),cos(u))
                w = min(1.3 , max(-1.3, wgain*bound))
                # Publish
                cmdmsg.linear.x = v
                cmdmsg.angular.z = w
                #print(u)
                #print('Current control:', v,w)
                cmdpub.publish(cmdmsg)

        rospy.spin()

if __name__ == "__main__":
    astar_navigation = AstarNavigation()
    astar_navigation.run()
