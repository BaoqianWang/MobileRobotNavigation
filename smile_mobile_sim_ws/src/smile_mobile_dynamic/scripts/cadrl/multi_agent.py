import agent
import network
import util
import numpy as np
import matplotlib.pyplot as plt

# Load network configurations
possible_actions = network.Actions()
num_actions = possible_actions.num_actions
nn = network.NetworkVP_rnn(network.Config.DEVICE, 'network', num_actions)
nn.simple_load('../checkpoints/network_01900000')

#Define color
plt_colors = []
plt_colors.append([0.8500, 0.3250, 0.0980])  # red
plt_colors.append([0.0, 0.4470, 0.7410])  # blue
plt_colors.append([0.4660, 0.6740, 0.1880])  # green
plt_colors.append([0.4940, 0.1840, 0.5560])  # purple
plt_colors.append([0.9290, 0.6940, 0.1250])  # orange
plt_colors.append([0.3010, 0.7450, 0.9330])  # cyan
plt_colors.append([0.6350, 0.0780, 0.1840])  # chocolate

# Define the agents
# Parameters start_x, start_y, goal_x, goal_y, radius, pref_speed, initial_heading, index
# host_agent = agent.Agent(0, 0, 5, 5,0,0,radius=0.5, pref_speed=1.2, initial_heading=0,id=0)
# host_agent.vel_global_frame = np.array([v_x, v_y])

# Sample observation data in a format easily generated from sensors
# Define agents
agents_goal_x=[-1, 12, 8, 4]
agents_goal_y=[4, 10, 6, 8]
agents_x = [0,-1, -1, 10] #Initial x
agents_y = [0,-1, 2, -2] #Initial y
agents_r = [0.2,0.2, 0.2, 0.2]
agents_vx = [0,0.0, 0.0, 0]
agents_vy = [0,0.0, 0, 0]
num_other_agents = len(agents_x)

#Create a figure
fig = plt.figure(figsize=(15, 6))
ax = fig.add_subplot(1, 2, 1)

# Create Agent objects for each observed dynamic obstacle
agents = []
for i in range(num_other_agents):
    x = agents_x[i]
    y = agents_y[i]
    v_x = agents_vx[i]
    v_y = agents_vy[i]
    radius = agents_r[i]
    agentInstance = agent.Agent(x, y, agents_goal_x[i], agents_goal_y[i], radius=radius, id=i + 1)
    agentInstance.vel_global_frame = np.array([v_x, v_y])
    agents.append(agentInstance)
    ax.plot(x, y, marker='*', markersize=20,c=plt_colors[i])
    ax.plot(agents_goal_x[i], agents_goal_y[i],  marker='*', markersize=20,c=plt_colors[i])
    plt.ion()
    plt.show()



#Start navigating

while (1):
    allAtGoal=0
    for i in range(len(agents)):
        otherAgents=[agentInstance for j,agentInstance in enumerate(agents) if j!=i]
        currentAgent=agents[i]
        obs = currentAgent.observe(otherAgents)[1:]
        obs = np.expand_dims(obs, axis=0)
        circ1 = plt.Circle((currentAgent.pos_global_frame[0], currentAgent.pos_global_frame[1]), radius=agents_r[i], fc='w', ec=plt_colors[i])
        ax.add_patch(circ1)
        predictions = nn.predict_p(obs)[0]
        action = possible_actions.actions[np.argmax(predictions)]
        currentAgent.update_state(action, .2)
        allAtGoal+=currentAgent.is_at_goal
    plt.draw()
    plt.pause(0.2)
    if(allAtGoal==len(agents)):
        print('Done')
        break









