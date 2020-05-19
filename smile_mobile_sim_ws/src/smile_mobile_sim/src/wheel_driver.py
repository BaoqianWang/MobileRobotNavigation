#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist

def run():
    
    #Initialize the wheel driver node (simulation)
    rospy.init_node('wheel_driver')
    
    #Create a publisher to write to the wheel links
    vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    
    #rate to publish
    rate = rospy.Rate(10)
    
    #Initialize the twist message
    vel_twist_msg = Twist()
    vel_twist_msg.linear.x = 1
    vel_twist_msg.linear.y = 0
    vel_twist_msg.linear.z = 0
    
    vel_twist_msg.angular.x = 0
    vel_twist_msg.angular.y = 0
    vel_twist_msg.angular.z = 0
    
    while (not rospy.is_shutdown()):
        vel_pub.publish(vel_twist_msg)
        
        rate.sleep()
        
if __name__ == "__main__":
    try:
        run()
            
    except rospy.RosInterruptException:
        print("Failed")
        
        
        
        
        
    