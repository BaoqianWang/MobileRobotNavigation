#!/usr/bin/env python

import rospy, tf
import geometry_msgs.msg, nav_msgs.msg
from math import *
from time import sleep
import agent



########################################
# Main Script
# Initialize our node
rospy.init_node('husky_dynamic_collision_avoidance', anonymous=True)

#Agents name
agents_name=['husky_alpha','husky_beta','husky_beta']

#
# Set waypoint for Husky to drive to
goal = [10, 10]  # Goal

# Setup publisher
cmdmsg = geometry_msgs.msg.Twist()
cmdpub = rospy.Publisher('/sim/husky_alpha/husky_velocity_controller/cmd_vel', geometry_msgs.msg.Twist, queue_size=10)

msg = geometry_msgs.msg.Twist()
msg.linear.x = 1
msg.angular.z = 1
cmdpub.publish(msg)

# Setup subscription - which implemets our controller.
# We pass the publisher, the message to publish and the goal as
# additional parameters to the callback function.
#rospy.Subscriber('odometry/filtered', nav_msgs.msg.Odometry, huskyOdomCallback,
 #                (cmdpub, cmdmsg, goal))
# spin() simply keeps python from exiting until this node is stopped
rospy.spin()