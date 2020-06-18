import numpy as np
import time
import matplotlib.pyplot as plt; plt.ion()
from mpl_toolkits.mplot3d import Axes3D
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
import Planner
import pyrr
import json


def tic():
  return time.time()

def toc(tstart, nm=""):
  print('%s took: %s sec.\n' % (nm,(time.time() - tstart)))

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
  for block in blocks:
      block[0]=block[0]-block[3]/2
      block[1]=block[1]-block[4]/2
      block[2]=block[2]-block[5]/2
      block[3]=block[0]+block[3]
      block[4]=block[1]+block[4]
      block[5]=block[2]+block[5]

  print(blocks)
  return boundary, blocks



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
  toc(t0,"Planning")

  #Concatenate the path
  #path=np.vstack((np.asarray(rx),np.asarray(ry),np.asarray(rz))).T

  #Draw the environment and planned path

  return rx,ry,rz

def generate_database(start, map_file, resolution):
    database=[]
    for i in range(1):
        goal = [-25.0, 2, 0.2]
        #database=[]
        path=dict()
        rx,ry,rz = runtest(map_file,start,np.asarray(goal),resolution)
        boundary, blocks=load_map(map_file)
        #path['start']=start
        path['goal']=goal
        path['x']=rx
        path['y']=ry
        database.append(path)
        print(path)
        #my_dict = {'foo': [1,2], 'bar':[3,4]}

        # create list of strings
        #list_of_strings = [ f'{key} : {path[key]}' for key in path ]
        #list_of_strings= ["{}: {}".format(key,path[key]) for key in path]
        # write string one by one adding newline
        #with open('path.txt', 'w') as my_file:
            #[ my_file.write(st+"\n") for st in list_of_strings ]
    with open('path_database.txt', 'w') as fd:
        fd.write(json.dumps(database))
    return

if __name__=="__main__":
  #test_single_cube()
  #test_maze()
  #test_flappy_bird()
  #test_monza()
  #test_window()
  #test_tower()
  #test_room()
  start = np.array([-10, 22, 0.2])
  #goal = np.array([9.0, 7.0, 1.5])
  generate_database(start,"./maps/smile_world_tree.txt",1)
