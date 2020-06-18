import agent
import network
import util
import numpy as np
import matplotlib.pyplot as plt

#Load network configurations
possible_actions = network.Actions()
num_actions = possible_actions.num_actions
nn = network.NetworkVP_rnn(network.Config.DEVICE, 'network', num_actions)
nn.simple_load('../checkpoints/network_01900000')

start_x = 0
start_y = 0
goal_x = 5
goal_y = 5
radius = 0.5
pref_speed = 1.2
heading_angle = 0.
index = 0
v_x = 0
v_y = 0

#Define the agents
#Parameters
host_agent = agent.Agent(start_x, start_y, goal_x, goal_y, 
                         radius=radius, pref_speed=pref_speed, initial_heading=heading_angle,
                         id=index)
host_agent.vel_global_frame = np.array([v_x, v_y])

# Sample observation data in a format easily generated from sensors
other_agents_x = [1,3,5]
other_agents_y = [2,2,2]
other_agents_r = [0.2, 0.2, 0.2]
other_agents_vx = [0.0, 0.0, 0]
other_agents_vy = [0.0, 0, 0]
num_other_agents = len(other_agents_x)

# Create Agent objects for each observed dynamic obstacle
other_agents = []
for i in range(num_other_agents):
    x = other_agents_x[i]; y = other_agents_y[i]
    v_x = other_agents_vx[i]; v_y = other_agents_vy[i]
    radius = other_agents_r[i]
    
    other_agent = agent.Agent(x, y, goal_x, goal_y, radius=radius, id=i+1)
    other_agent.vel_global_frame = np.array([v_x, v_y])
    other_agents.append(other_agent)

obs = host_agent.observe(other_agents)[1:]
obs = np.expand_dims(obs, axis=0)


i=0
#plt.scatter(start_x, start_y, c='yellow',marker='*',s=250)
#plt.scatter(goal_x, goal_y, c='green',marker='*',s=250)

#Create a figure
fig = plt.figure(figsize=(15, 6))
ax = fig.add_subplot(1, 2, 1)
ax.plot(start_x,start_y,c=[0.8500, 0.3250, 0.0980],marker='*', markersize=20)
ax.plot(goal_x,goal_y,c=[0.0, 0.4470, 0.7410],marker='*', markersize=20)
plt.ion()
plt.show()

while(1):
    circ1 = plt.Circle((host_agent.pos_global_frame[0], host_agent.pos_global_frame[1]), radius=radius, fc='w', ec=[0.9290, 0.6940, 0.1250])
    ax.add_patch(circ1)
    #plt.plot(host_agent.pos_global_frame[0], host_agent.pos_global_frame[1], "xc")
    # for stopping simulation with the esc key.
    #plt.gcf().canvas.mpl_connect('key_release_event',lambda event: [exit(0) if event.key == 'escape' else None])
    i=i+1	
    predictions = nn.predict_p(obs)[0]
    action = possible_actions.actions[np.argmax(predictions)]
	#action = np.array([host_agent.pref_speed*raw_action[0], util.wrap(raw_action[1] + host_agent.heading_global_frame)])
    host_agent.update_state(action, .5)
	#plt.scatter(host_agent.pos_global_frame[0], host_agent.pos_global_frame[1], c='red',marker='o')
	#plt.arrow(host_agent.pos_global_frame[0], host_agent.pos_global_frame[1],host_agent.vel_global_frame[0],host_agent.vel_global_frame[1])
    for otherAgent in other_agents:
	plt.plot(otherAgent.pos_global_frame[0], otherAgent.pos_global_frame[1],marker='o')
	#plt.arrow(otherAgent.pos_global_frame[0], otherAgent.pos_global_frame[1],otherAgent.vel_global_frame[0],otherAgent.vel_global_frame[1])
	randomAction=np.random.random(2)
	#randomAction[1]=0
	otherAgent.update_state(randomAction,.5)
	obs = host_agent.observe(other_agents)[1:]
	obs = np.expand_dims(obs, axis=0)
    plt.draw()
    plt.pause(0.2)
    if(host_agent.is_at_goal):
        util.plot_current_state_ego_frame(obs)
	print('done')
	break
    if(np.remainder(i,200)==0):
	plt.show()










