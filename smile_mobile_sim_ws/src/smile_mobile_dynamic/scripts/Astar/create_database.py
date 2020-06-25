import numpy as np
import time
import matplotlib.pyplot as plt; plt.ion()
from mpl_toolkits.mplot3d import Axes3D
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
import Planner
import pyrr
import json
from nose.tools import assert_raises
from dijkstra import dijkstra, shortest_path, make_graph


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


def paved_shortest_path(startNode, endNode):
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
    path=shortest_path(graph, startNode, endNode)
    cost=0
    for i, node in enumerate(path[:-1]):
        cost+=graph[path[i]][path[i+1]]
    return cost, path


def tic():
  return time.time()

def toc(tstart, nm=""):
  print('%s took: %s sec.\n' % (nm,(time.time() - tstart)))

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


def draw_block_list(ax,blocks):
  '''
  Subroutine used by draw_map() to display the environment blocks
  '''
  v = np.array([[0,0,0],[1,0,0],[1,1,0],[0,1,0],[0,0,1],[1,0,1],[1,1,1],[0,1,1]],dtype='float')
  f = np.array([[0,1,5,4],[1,2,6,5],[2,3,7,6],[3,0,4,7],[0,1,2,3],[4,5,6,7]])
  clr = blocks[:,6:]/255
  n = blocks.shape[0]
  d = blocks[:,3:6] - blocks[:,:3]
  vl = np.zeros((8*n,3))
  fl = np.zeros((6*n,4),dtype='int64')
  fcl = np.zeros((6*n,3))
  for k in range(n):
    vl[k*8:(k+1)*8,:] = v * d[k] + blocks[k,:3]
    fl[k*6:(k+1)*6,:] = f + k*8
    fcl[k*6:(k+1)*6,:] = clr[k,:]

  if type(ax) is Poly3DCollection:
    ax.set_verts(vl[fl])
  else:
    pc = Poly3DCollection(vl[fl], alpha=0.25, linewidths=1, edgecolors='k')
    pc.set_facecolor(fcl)
    h = ax.add_collection3d(pc)
    return h


def runtest(mapfile, start, goal, resolution, verbose = True):
  '''
  This function:
   * load the provided mapfile
   * creates a motion planner
   * plans a path from start to goal
   * checks whether the path is collision free and reaches the goal
   * computes the path length as a sum of the Euclidean norm of the path segments
  '''
  # Load a map and instantiate a motion planner

  boundary, blocks = load_map(mapfile)
  MP = Planner.MyPlanner(boundary, blocks, resolution) # TODO: replace this with your own planner implementation


  # Call the motion planner
  t0 = tic()
  rx, ry, rz = MP.plan(start, goal)
  path=dict()
  path['goal']=goal
  path['start']=start
  path['x']=rx[:-1]
  path['y']=ry[:-1]
  #print(path)
  toc(t0,"Planning")
  return rx,ry,rz

def generate_database(start, map_file, resolution):
    database=[]

    #Path paved scenario
    for waypoint, position in waypoints.items():
        print('The waypoint is', waypoint)
        start=[position[0],position[1],0.2]
        costa, patha = paved_shortest_path(waypoint,'a')
        print('The path is',patha)
        goals=[(i,j,0.2) for i in np.arange(-33,-10,4) for j in np.arange(-23,22,4)]
        start_unpaved=[waypoints['a'][0],waypoints['a'][1],0.2]

        for i in range(-33,-10,4):
            for j in range(-23,22,4):
                goal=[i,j,0.2]
                path=dict()
                rx,ry,rz = runtest(map_file,start_unpaved,np.asarray(goal),resolution)
                for node in reversed(patha):
                    print(node)
                    rx.append(waypoints[node][0])
                    ry.append(waypoints[node][1])
                length=len(rx)
                boundary, blocks=load_map(map_file)
                #path['start']=start
                path['goal']=goal#[goal[0],goal[1],goal[2]]
                path['x']=rx
                path['y']=ry
                path['start']=start
                database.append(path)
    with open('path_database.txt', 'w') as fd:
        fd.write(json.dumps(database))

    return

if __name__=="__main__":

  start = np.array([-10, 22, 0.2])
  #goal = np.array([9.0, 7.0, 1.5])
  generate_database(start,"./maps/smile_world_pure_trees.txt",4)
