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
  return boundary, blocks


class HuskyRosSimulator():
    def __init__(self,goal_x,goal_y,radius,pref_speed, name,index,network,other_agent_name,actions):
        #self.husky_agent=agent_instance # agent.Agent(initial_position[0], initial_position[1], goal_position[0], goal_position[1], radius=radius, id=index)
        self.pose=[-10,22,0] # Need to change to real initial poses
        self.init=[-10,22,0]
        self.vel=0
        self.index=index
        self.radius=radius
        self.goal_x=goal_x
        self.goal_y=goal_y
        self.pref_speed=pref_speed
        self.nn=network
        self.actions=actions
        self.other_agent_name=other_agent_name
        self.desired_action=[0,0]
        # cmdmsg = geometry_msgs.msg.Twist()
        #self.other_agents=[]
        self.cmdpub = rospy.Publisher('/smile/desired/movement' %name, geometry_msgs.msg.Twist, queue_size=10)
        self.pose_subscriber=rospy.Subscriber('/smile/raw/odometry' %name, nav_msgs.msg.Odometry,self.update_pose)
        self.pose_subscriber_other=rospy.Subscriber('/husky_%s/odometry/filtered' %other_agent_name, nav_msgs.msg.Odometry,self.observe_other_agents)
        self.control_timer = rospy.Timer(rospy.Duration(0.01),self.husky_control)
        #self.observe=rospy.Timer(rospy.Duration(0.1),self.observe_environments)
        self.nn_timer = rospy.Timer(rospy.Duration(0.1),self.husky_compute_actions)


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

        x= other_pos.position.x+3
        y= other_pos.position.y
        heading_angle=theta
        goal_x=10+3 #Need to be defined manully
        goal_y=10
        radius=0.6
        pref_speed=4
        index=1
        vx=vel*np.cos(heading_angle)
        vy=vel*np.sin(heading_angle)

        agent_instance=agent.Agent(x, y, goal_x, goal_y, radius, pref_speed, heading_angle, index)
        agent_instance.vel_global_frame = np.array([vx, vy])


        self.other_agent=[agent_instance]

        boundary, blocks = load_map('../Astar/maps/smile_world_tree.txt')

        for block in blocks
           # other_agents.append(agent_instance)
           #
           #
           # for i, agent_ros in enumerate(other_agents_ros_simu)
           #     x= agent_ros.pose[0]
           #     y= agent_ros.pose[1]
           #     heading_angle=agent_ros.pose[2]
           #     goal_x=agent_ros.goal_x
           #     goal_y=agent_ros.goal_y
           #     radius=agent_ros.radius
           #     pref_speed=agent_ros.pref_speed
           #     index=agent_ros.index
           #     vx=agent_ros.vel*np.cos(heading_angle)
           #     vy=agent_ros.vel*np.sin(heading_angle)
           #     agent_instance=agent.Agent(x, y, goal_x, goal_y, radius, pref_speed, heading_angle, index)
           #     agent_instace.vel_global_frame = np.array([v_x, v_y])
           #     other_agents.append(agent_instance)

    def husky_compute_actions(self, event):
        x= self.pose[0]
        y= self.pose[1]
        heading_angle=self.pose[2]
        goal_x=self.goal_x
        goal_y=self.goal_y
        radius=self.radius
        pref_speed=self.pref_speed
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


        if host_agent.dist_to_goal < 2.0: # and self.percentComplete>=0.9:
            # print "somewhat close to goal"
            pref_speed = max(min(kp_v * (host_agent.dist_to_goal-0.1), pref_speed), 0.0)
            action[0] = min(raw_action[0], pref_speed)
            turn_amount = max(min(kp_r * (host_agent.dist_to_goal-0.1), 1.0), 0.0) * raw_action[1]
            action[1] = util.wrap(turn_amount + self.pose[2])

        #self.update_action(action)
        self.desired_action=action
        return


    def husky_control(self,event):
        desired_yaw=self.desired_action[1]
        yaw_error=desired_yaw-self.pose[2]
        if abs(yaw_error)>np.pi:
            yaw_error-=np.sign(yaw_error)*2*np.pi

        gain=5
        vw=gain*yaw_error
        #use_d_min=False
        vx=self.desired_action[0]
        twist=geometry_msgs.msg.Twist()
        twist.angular.z=vw
        twist.linear.x=vx
        self.cmdpub.publish(twist)
        return





class MultiAgentDCA():
    def __init__(self,node_name="multi_agent_dynamic_collision_avodiance"):
        rospy.init_node(node_name)

    def run(self):
        husky_robot_names=['beta','gamma']
        possible_actions = network.Actions()
        actions=possible_actions.actions
        num_actions = possible_actions.num_actions
        nn = network.NetworkVP_rnn(network.Config.DEVICE, 'network', num_actions)
        nn.simple_load('../checkpoints/network_01900000')

        goal_x=10
        goal_y=5
        radius=0.6
        pref_speed=2
        name=husky_robot_names[0]
        index=0
        other_agent_name=husky_robot_names[1]
        husky_robot=HuskyRosSimulator(goal_x,goal_y,radius,pref_speed, name,index,nn,other_agent_name,actions)
        rospy.spin()

if __name__ == "__main__":
    multi_agent_dca = MultiAgentDCA()
    multi_agent_dca.run()

        # agents_goal_x=[-1, 12]
        # agents_goal_y=[4, 10]
        # agents_x = [0,-1] #Initial x
        # agents_y = [0,-1] #Initial y
        # agents_r = [0.2,0.2]
        # agents_vx = [0,0.0]
        # agents_vy = [0,0.0]
        # num_other_agents = len(agents_x)
        # radius=0.5
        # pref_speed=2
        # # Create Agent objects for each observed dynamic obstacle
        # ros_agents=[]
        # for i in range(num_other_agents):
        #     x = agents_x[i]
        #     y = agents_y[i]
        #     v_x = agents_vx[i]
        #     v_y = agents_vy[i]
        #     radius = agents_r[i]
        #     ros_agent_instance=HuskyRosSimulator(agents_goal_x[i], agents_goal_y[i], radius, pref_speed, husky_robot_names[i],i,nn)
        #     ros_agents.append(ros_agent_instance)
        #
        #
        # #Start navigating
        # while (1):
        #     allAtGoal=0
        #     for i in range(len(agents)):
        #         otherAgents=[agentInstance for j,agentInstance in enumerate(agents) if j!=i]
        #         currentAgent=agents[i]
        #         obs = currentAgent.observe(otherAgents)[1:]
        #         obs = np.expand_dims(obs, axis=0)
        #         predictions = nn.predict_p(obs)[0]
        #         action = possible_actions.actions[np.argmax(predictions)]
        #         cmdmsg = geometry_msgs.msg.Twist()
        #         cmdmsg.linear.x = action[0]*currentAgent.pref_speed
        #         cmdmsg.angular.z = action[1]
        #         currentAgent.update_state(action, .2)
        #         ros_agents[i].control_in_ros(cmdmsg)
        #         allAtGoal+=currentAgent.is_at_goal
        #         print(allAtGoal)
        #     if(allAtGoal==len(agents)):
        #         print('Done')
        #         break
