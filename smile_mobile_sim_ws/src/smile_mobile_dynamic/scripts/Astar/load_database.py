import json
import numpy as np

filename='path_database.txt'
with open(filename, 'r') as fd:
    database=json.load(fd)
    #print(json.load(fd))
start=[-10,22,0.2]
goal=[-25,6,0.2]
distance_list=[]
for path in database:
    path_start=path['start']
    path_end=path['goal']
    dist = np.linalg.norm(np.asarray(path_start)-np.asarray(start))+np.linalg.norm(np.asarray(path_end)-np.asarray(goal))
    distance_list.append(dist)

ind = np.argmin(distance_list)
print(database[ind])
