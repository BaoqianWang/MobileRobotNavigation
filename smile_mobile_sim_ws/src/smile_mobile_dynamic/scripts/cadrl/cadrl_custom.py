#!/usr/bin/env python
import rospy, tf
import geometry_msgs.msg, nav_msgs.msg
from math import *
import numpy as np

import agent
import network
import util
import numpy as np
import matplotlib.pyplot as plt
import copy
import json
from std_msgs.msg import Float32MultiArray



def load_map(fname):
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
  boundary = mapdata[~blockIdx][['xmin', 'ymin', 'zmin', 'xmax', 'ymax', 'zmax','r','g','b']].view('<f4').reshape(-1,11)[:,2:]
  blocks = mapdata[blockIdx][['xmin', 'ymin', 'zmin', 'xmax', 'ymax', 'zmax','r','g','b']].view('<f4').reshape(-1,11)[:,2:]


  #print(blocks)
  return blocks


class HuskyRosSimulator():
    def __init__(self,goal_x,goal_y,radius,pref_speed, index,network,other_agent_name,actions,initial,path_name):
        #self.husky_agent=agent_instance # agent.Agent(initial_position[0], initial_position[1], goal_position[0], goal_position[1], radius=radius, id=index)
        self.pose=[initial[0],initial[1],initial[2]] # Need to change to real initial poses
        self.init=[initial[0],initial[1],initial[2]]
        self.vel=0
        self.index=index
        self.radius=radius
        self.pref_speed=pref_speed
        self.nn=network
        self.waypoint=0
        self.actions=actions
        self.other_agent_name=other_agent_name
        self.desired_action=[0,0]
        self.reach_goal=0
        self.goal_x=goal_x
        self.goal_y=goal_y
        self.load_path(path_name)
        self.static_obstacles= load_map('/home/smile/smile-mobile/smile_mobile_sim_ws/src/smile_mobile_dynamic/scripts/Astar/maps/smile_world_pure_trees.txt')
        self.cmdpub = rospy.Publisher('/smile/desired/movement', Float32MultiArray, queue_size=10)
        self.pose_subscriber=rospy.Subscriber('/smile/raw/odometry', nav_msgs.msg.Odometry,self.update_pose)
        self.pose_subscriber_other=rospy.Subscriber('/husky_%s/odometry/filtered' %other_agent_name, nav_msgs.msg.Odometry,self.observe_other_agents)
        self.control_timer = rospy.Timer(rospy.Duration(0.01),self.husky_control)
        #self.observe=rospy.Timer(rospy.Duration(0.1),self.observe_environments)
        self.nn_timer = rospy.Timer(rospy.Duration(0.05),self.husky_compute_actions)
        self.goal_update=rospy.Timer(rospy.Duration(0.05),self.set_goal)

    def load_path(self,filename):
        with open(filename,'r') as fd:
            database=json.load(fd)
        start=[self.init[0], self.init[1],0.2]
        goal=[self.goal_x,self.goal_y,0.2]
        distance_list=[]
        for path in database:
            path_start=path['start']
            path_end=path['goal']
            dist = np.linalg.norm(np.asarray(path_start)-np.asarray(start))+np.linalg.norm(np.asarray(path_end)-np.asarray(goal))
            distance_list.append(dist)

        ind = np.argmin(distance_list)
        path=database[ind]
        goals=zip(path['x'],path['y'])
        goals=goals[::-1]
        self.goals=goals



    def set_goal(self,event):
        goal=self.goals[self.waypoint]
        print(goal)
        distance = sqrt((self.pose[0]-goal[0])**2+(self.pose[1]-goal[1])**2)
        self.goal_x=goal[0]
        self.goal_y=goal[1]
        if(distance<1.3 and self.waypoint+1<len(self.goals)):
            self.waypoint+=1

            print(self.waypoint)
        #print('goal is:',self.goal_x, self.goal_y, 'num_way is', self.waypoint,'distance', distance,'current_pose',self.pose[0], self.pose[1])


    def update_pose(self,message):
        pos = message.pose.pose
        quat = pos.orientation
        vel=message.twist.twist.linear
        # From quaternion to Euler
        angles = tf.transformations.euler_from_quaternion((quat.x,quat.y,quat.z,quat.w))
        theta = angles[2]
        self.pose = [self.init[0]+pos.position.x, self.init[1]+pos.position.y, theta]  # X, Y, Theta
        self.vel=vel.x
        return

    def observe_other_agents(self,message):
        other_agents=[]
        other_pos=message.pose.pose
        quat=other_pos.orientation
        vel=message.twist.twist.linear.x
        angles = tf.transformations.euler_from_quaternion((quat.x,quat.y,quat.z,quat.w))
        theta = angles[2]

        x= other_pos.position.x-27.7
        y= other_pos.position.y+4.26
        #print('other agent position', x, y)
        heading_angle=theta
        goal_x=-16 #Need to be defined manully
        goal_y=4.26
        radius=1.5
        pref_speed=1.5
        index=1
        vx=vel*np.cos(heading_angle)
        vy=vel*np.sin(heading_angle)

        agent_instance=agent.Agent(x, y, goal_x, goal_y, radius, pref_speed, heading_angle, index)
        agent_instance.vel_global_frame = np.array([vx, vy])
        print('Other agent', x,y)

        self.other_agent=[agent_instance]
        for i,block in enumerate(self.static_obstacles):
            agent_instance=agent.Agent(block[0],block[1],block[0],block[1],1.414*block[3]/2,0,0,2+i)
            agent_instance.vel_global_frame=np.array([0,0])
            self.other_agent.append(agent_instance)
        return


    def husky_compute_actions(self, event):
        x= self.pose[0]
        y= self.pose[1]
        heading_angle=self.pose[2]
        goal_x=self.goal_x
        goal_y=self.goal_y
        radius=self.radius
        pref_speed=self.pref_speed
        distance=sqrt((self.pose[0]-self.goals[-1][0])**2+(self.pose[1]-self.goals[-1][1])**2)
        if distance < 2:
            pref_speed=0.3
        index=self.index
        vx=self.vel*np.cos(heading_angle)
        vy=self.vel*np.sin(heading_angle)

        host_agent = agent.Agent(x, y, goal_x, goal_y, radius, pref_speed, heading_angle, index)
        host_agent.vel_global_frame = np.array([vx, vy])
        # host_agent.print_agent_info()

        other_agents_state = copy.deepcopy(self.other_agent)
        obs = host_agent.observe(other_agents_state)[1:]
        obs = np.expand_dims(obs, axis=0)
        # print "obs:", obs
        predictions = self.nn.predict_p(obs)[0]
        # print "predictions:", predictions
        # print "best action index:", np.argmax(predictions)
        raw_action = copy.deepcopy(self.actions[np.argmax(predictions)])
        action = np.array([pref_speed*raw_action[0], util.wrap(raw_action[1] + self.pose[2])])


        # if close to goal
        kp_v = 0.5
        kp_r = 1

        distance=sqrt((self.pose[0]-self.goals[-1][0])**2+(self.pose[1]-self.goals[-1][1])**2)
        if distance < 0.5:
            action=[0,0]
            rospy.signal_shutdown('Done')
        # if host_agent.dist_to_goal < 2: # and self.percentComplete>=0.9:
        #     print "somewhat close to goal"
        #     pref_speed = max(min(kp_v * (host_agent.dist_to_goal-0.1), pref_speed), 0.0)
        #     action[0] = min(raw_action[0], pref_speed)
        #     turn_amount = max(min(kp_r * (host_agent.dist_to_goal-0.1), 1.0), 0.0) * raw_action[1]
        #     action[1] = util.wrap(turn_amount + self.pose[2])
        self.desired_action=action
        # if host_agent.dist_to_goal < 1:
        #     self.reach_goal=1
        #     action=[0,0]
        #     self.desired_action=action
        # if self.reach_goal==1:
        #     action=[0,0]
        #     self.desired_action=action
        #self.update_action(action)

        #print('desired action', self.desired_action,'current_pose',self.pose[0], self.pose[1])
        return


    def husky_control(self,event):
        desired_yaw=self.desired_action[1]
        vx=self.desired_action[0]
        cmdmsg = Float32MultiArray()
        cmdmsg.data = [vx, desired_yaw]
        # print(vx,desired_yaw)
        distance=sqrt((self.pose[0]-self.goals[-1][0])**2+(self.pose[1]-self.goals[-1][1])**2)
        if distance < 0.5:
            cmdmsg.data=[0,0]
            print('shuting down')
            self.cmdpub.publish(cmdmsg)
            rospy.signal_shutdown('Done')
        # if distance < 1:
        #     cmdmsg.data=[0,0]
            #self.cmdpub.publish(cmdmsg)
        self.cmdpub.publish(cmdmsg)
        #print('desired action', vx, desired_yaw,'current_pose',self.pose[0], self.pose[1])
        return





class MultiAgentDCA():
    def __init__(self,node_name="multi_agent_dynamic_collision_avodiance"):
        rospy.init_node(node_name)

    def run(self):
        husky_robot_names=['beta']
        possible_actions = network.Actions()
        actions=possible_actions.actions
        num_actions = possible_actions.num_actions
        nn = network.NetworkVP_rnn(network.Config.DEVICE, 'network', num_actions)
        nn.simple_load('/home/smile/smile-mobile/smile_mobile_sim_ws/src/smile_mobile_dynamic/scripts/cadrl/checkpoints/network_01900000')

        goal_x=-30
        goal_y=13
        radius=1
        pref_speed=4.4
        initial=[-6.87, 0,0]
        path_name='/home/smile/smile-mobile/smile_mobile_sim_ws/src/smile_mobile_dynamic/scripts/Astar/path_database.txt'
        #name=husky_robot_names[0]
        index=0
        other_agent_name=husky_robot_names[0]
        husky_robot=HuskyRosSimulator(goal_x,goal_y,radius,pref_speed, index,nn,other_agent_name,actions,initial,path_name)
        rospy.spin()

if __name__ == "__main__":
    multi_agent_dca = MultiAgentDCA()
    multi_agent_dca.run()
