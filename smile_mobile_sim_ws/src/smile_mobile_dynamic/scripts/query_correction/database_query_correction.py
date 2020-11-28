import numpy as np
import json
import Planner
import pyrr
from nose.tools import assert_raises
from dijkstra import dijkstra, shortest_path, make_graph
import matplotlib.pyplot as plt

waypoints={'a': [-6.87, 0],
'b': [-6.75, -21.36],
'c': [0.04, 1.28],
'd': [0.18, -20.81],
'e': [20.393, 13.95],
'f': [20.393, 1.3968],
'g': [15.78, -20.69],
'h': [33.46, 1.20],
'i': [33.00, 13.87],
'j': [7.77, 13.48],
'k': [2.52, 11.23],
'l': [32.57, -11.1],
'm': [28.56, -14.08],
'o': [25.03, -10.39],
'p': [20.89, -5.48],
'q': [15.53, -7.08]
}


graph = {'a': {'c':7.2},
       'b': {'d':7.2},
       'c': {'a':7.2,'k':11,'f':20},
       'd': {'g':15},
       'e': {'j':13,'f':12,'i':13},
       'f': {'h':13,'c':20,'e':12},
       'g': {'q':13,'d':15},
       'h': {'l':12,'i':12,'f':13},
       'i': {'h':12,'e':13},
       'j': {'k':5.5,'e':13},
       'k': {'j':5.5,'c':10},
       'l': {'h':12,'m':5.2},
       'm': {'o':5.2,'l':5.2},
       'o': {'m':5.2,'p':5.2},
       'p': {'o':5.2,'q':5.2},
       'q': {'p':5.2,'g':13}
       }


def load_map(fname):
  mapdata = np.loadtxt(fname,dtype={'names': ('type', 'xmin', 'ymin', 'zmin', 'xmax', 'ymax', 'zmax','r','g','b'),\
                                    'formats': ('S8','f', 'f', 'f', 'f', 'f', 'f', 'f','f','f')})
  blockIdx = mapdata['type'] == b'block'
  boundary = mapdata[~blockIdx][['xmin', 'ymin', 'zmin', 'xmax', 'ymax', 'zmax','r','g','b']].view('<f4').reshape(-1,11)[:,2:]
  blocks = mapdata[blockIdx][['xmin', 'ymin', 'zmin', 'xmax', 'ymax', 'zmax','r','g','b']].view('<f4').reshape(-1,11)[:,2:]
  for block in blocks:
      block[0]=block[0]-block[3]/2
      block[1]=block[1]-block[4]/2
      block[2]=block[2]-block[5]/2
      block[3]=block[0]+block[3]
      block[4]=block[1]+block[4]
      block[5]=block[2]+block[5]
  return boundary, blocks

class PathQueryCorrection():
    def __init__(self,initial, goal, database_path):
        self.initial=initial
        self.goal=goal
        self.filename=database_path
        self.boundary, self.blocks = load_map('/home/smile/smile-mobile/smile_mobile_sim_ws/src/smile_mobile_dynamic/scripts/Astar/maps/smile_world_pure_trees.txt')

    def unpaved_path_planning(self,start, goal, resolution):
        #Unpaved road path planning

        MP = Planner.MyPlanner(self.boundary, self.blocks, resolution) # TODO: replace this with your own planner implementation
        rx, ry = MP.plan(start, goal)
        return rx,ry

    def paved_path_planning(self,startNode,endNode):
        path=shortest_path(graph, startNode, endNode)
        return path


    def path_query(self):
        with open(self.filename,'r') as fd:
            database=json.load(fd)
        start=[self.initial[0], self.initial[1]]
        goal=[self.goal[0],self.goal[1]]
        distance_list=[]
        for path in database:
            path_start=path['start']
            path_end=path['goal']
            dist = np.linalg.norm(np.asarray(path_start)-np.asarray(start))+np.linalg.norm(np.asarray(path_end)-np.asarray(goal))
            distance_list.append(dist)


        ind = np.argmin(distance_list)
        path=database[ind]

        self.goal_x=list(reversed(path['x']))
        self.goal_y=list(reversed(path['y']))

        plt.figure()
        ax=plt.subplot(1,1,1)

        plt.plot(self.goal_x,self.goal_y,label='Path')
        plt.plot(self.goal_x,self.goal_y,'.')
        ax.scatter(goal[0], goal[1],  marker='*',c='r',label='Goal')
        ax.scatter(initial[0], initial[1],  marker='d', c='r',label='Start')
        plt.xlabel('x (m)')
        plt.ylabel('y (m)')
        plt.legend()

        for block in self.blocks:
            circ1 = plt.Rectangle((block[0], block[1]), width=3.2, height=3.2,fc='tab:gray')
            ax.add_patch(circ1)
        axes = plt.gca()
        axes.set_xlim([-35,38])
        axes.set_ylim([-25,25])
        plt.savefig('query_database%d.png')


    def path_correction(self):

        print('extracted path', self.goal_x,self.goal_y)
        #Replan for the target
        start_unpaved=[self.goal_x[-1],self.goal_y[-1]]
        rx,ry = self.unpaved_path_planning(start_unpaved,np.asarray([self.goal[0],self.goal[1],0.2]),4)
        print('unpaved scenario',rx,ry)

        self.goal_x=self.goal_x+list(reversed(rx))
        self.goal_y=self.goal_y+list(reversed(ry))

        #Replan for the initial
        closet_start = min(waypoints.values(), key=lambda k: np.sqrt((k[0]-self.initial[0])**2+(k[1]-self.initial[1])**2))
        waypoint_list=[waypoint for waypoint, value in waypoints.items() if value == closet_start]
        target_waypoint=[waypoint for waypoint, value in waypoints.items() if value == [self.goal_x[0],self.goal_y[0]]]

        path_paved = self.paved_path_planning(waypoint_list[0],target_waypoint[0])
        for node in reversed(path_paved):
            self.goal_x.insert(0,waypoints[node][0])
            self.goal_y.insert(0,waypoints[node][1])
            print('paved scenario',waypoints[node][0],waypoints[node][1])

        final_path=dict()
        final_path['x']=self.goal_x
        final_path['y']=self.goal_y

        with open('final_path.txt', 'w') as fd:
            fd.write(json.dumps(final_path))

        plt.figure()
        ax=plt.subplot(1,1,1)

        self.goal_x.insert(0,self.initial[0])#+self.goal_x+self.goal[0]
        self.goal_y.insert(0,self.initial[1])#+self.goal_y+self.goal[1]

        self.goal_x.append(self.goal[0])#+self.goal_x+self.goal[0]
        self.goal_y.append(self.goal[1])#+self.goal_y+self.goal[1]


        plt.plot(self.goal_x,self.goal_y,label='Path')
        plt.plot(self.goal_x,self.goal_y,'.')
        ax.scatter(self.goal[0], self.goal[1],  marker='*',c='r',label='Goal')
        ax.scatter(self.initial[0], self.initial[1],  marker='d', c='r',label='Start')
        plt.xlabel('x (m)')
        plt.ylabel('y (m)')
        plt.legend()

        for block in self.blocks:
            circ1 = plt.Rectangle((block[0], block[1]), width=3.2, height=3.2,fc='tab:gray')
            ax.add_patch(circ1)
        axes = plt.gca()
        axes.set_xlim([-35,38])
        axes.set_ylim([-25,25])
        plt.savefig('correction_database%d.png')

    def run(self):
        self.path_query()
        self.path_correction()

if __name__=="__main__":

    with open('../parameters/parameters_configurations.json') as parameter_file:
        parameters=json.load(parameter_file)

    #Load parameters of host agent
    goal=parameters['host_goal']
    initial=parameters['host_initial']
    path_name='/home/smile/smile-mobile/smile_mobile_sim_ws/src/smile_mobile_dynamic/scripts/Astar/path_database.txt'
    path_query_correction=PathQueryCorrection(initial, goal, path_name)
    path_query_correction.run()
