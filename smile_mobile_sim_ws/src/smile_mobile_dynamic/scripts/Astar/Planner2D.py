import numpy as np
import math
show_animation = True
import matplotlib.pyplot as plt
import pyrr


paved_waypoints={'a': [-6.87, 0],
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

paved_graph = {'a': {'c':7.2},
       'b': {'d':7.2},
       'c': {'a':7.2,'k':11,'f':20},
       'd': {'g':15},
       'e': {'j':13,'f':12,'i':13},
       'f': {'h':13,'c':20},
       'g': {'q':13,'d':15},
       'h': {'l':12,'i':12},
       'i': {'h':12,'e':13},
       'j': {'k':5.5,'e':13},
       'k': {'j':5.5,'c':10},
       'l': {'h':12,'m':5.2},
       'm': {'o':5.2,'l':5.2},
       'o': {'m':5.2,'p':5.2},
       'p': {'o':5.2,'q':5.2},
       'q': {'p':5.2,'g':13}
       }

entry_point=['a','b']


class MyPlanner:

  #Only consider two dimensional
  def __init__(self, boundary, blocks, res):
    self.boundary = boundary
    self.blocks = blocks
    self.resolution=res
    self.xmin = round(boundary[0,0])
    self.ymin = round(boundary[0,1])


    self.xmax = round(boundary[0,2])
    self.ymax = round(boundary[0,3])


    self.xwidth = round((self.xmax - self.xmin) / self.resolution)
    self.ywidth = round((self.ymax - self.ymin) / self.resolution)

    #Define the motion of the node
    self.motion = self.motion_model()

  #Define the discrete node
  class Node:
      def __init__(self, x, y, cost, pind):
          self.x=x
          self.y=y
          self.cost = cost
          self.pind = pind


  #Discretize the continous position to discrete node index
  def discretize(self,position,minPos):
      return round((position-minPos)/self.resolution)

  #Create a index of each node
  def index_node(self,node):
      return (node.y - self.ymin) * self.xwidth + (node.x - self.xmin)

  #Get the position of the node
  def pos_node(self, index, minp):
      pos = index * self.resolution + minp
      return pos

  #Calculate the heuristic function
  def calculate_heuristic(self,node1,node2):
      w = 1.0  # weight of heuristic
      d = w * np.sqrt((node1.x - node2.x)**2+(node1.y - node2.y)**2)
      return d

  #Define the motion model
  def motion_model(self):
      # Consider the horizontal, vertical, diagonal movement
      # dx, dy, cost
      motion = [[1, 0,1],
                [0, 1,1],
                [-1, 0,1],
                [0, -1,1],
                [-1, -1, math.sqrt(2)],
                [1, 1,math.sqrt(2)]]

      return motion

  def collision_checking(self,startPoint, endPoint, box):
      # Check the collision between a line segement defined by a start point and end point and a axis aligned box.
      line1 = pyrr.line.create_from_points(startPoint, endPoint)
      line2 = pyrr.line.create_from_points(endPoint, startPoint)

      ray1 = pyrr.ray.create_from_line(line1)
      ray2 = pyrr.ray.create_from_line(line2)

      result1 = pyrr.geometric_tests.ray_intersect_aabb(ray1, box)
      result2 = pyrr.geometric_tests.ray_intersect_aabb(ray2, box)
      #if endPoint[0]>box[0] and endPoint[0]<box[3] and endPoint[1]>box[1] and endPoint[1]<box[4] and endPoint[2]>box[2] and endPoint[2]<box[5]:
          #return 1
      if (result1 is None or result2 is None):
          return 0
      return 1

  #Check if a node is valid
  def verify_node(self,node1, node2):
      px2 = self.pos_node(node2.x, self.xmin)
      py2 = self.pos_node(node2.y, self.ymin)

      px1 = self.pos_node(node1.x, self.xmin)
      py1 = self.pos_node(node1.y, self.ymin)


      startPoint=np.array([px1, py1])
      endPoint=np.array([px2,py2])

      #Boundary checking
      if px2 < self.xmin or py2 < self.ymin  or px2 >= self.xmax or py2 >= self.ymax:
        return False

      #Collision check
      for k in range(self.blocks.shape[0]):
         # Convert box to configuration space by adding the robot radius
        box=np.array([[self.blocks[k, 0]-0.5,self.blocks[k, 1]-0.5],[self.blocks[k, 3]+0.5,self.blocks[k, 4]+0.5]])

        if self.collision_checking(startPoint, endPoint, box):
            return False
      return True

  #Generate the final path
  def final_path(self, nodeGoal, closedSet):
      rx, ry = [self.pos_node(nodeGoal.x, self.xmin)], [
          self.pos_node(nodeGoal.y, self.ymin)]
      pind = nodeGoal.pind
      while pind != -1:
          n = closedSet[pind]
          rx.append(self.pos_node(n.x, self.xmin))
          ry.append(self.pos_node(n.y, self.ymin))
          pind = n.pind

      return rx, ry

  #Search the environment and do planning
  def plan(self,start,goal):

     # Start is the waypoint 'a', 'b', 'c'
     # Goal is the  two dimensional position
    nodeStart=self.Node(waypoint['start'][0],waypoint['start'][1],0.0,-1)
    nodeGoal=self.Node(self.discretize(goal[0],self.xmin),self.discretize(goal[1],self.ymin),0.0,-1)

    openSet, closedSet =dict(), dict()
    openSet[self.index_node(nodeStart)] = nodeStart
    expandedNode=0
    while 1:
      expandedNode+=1
      if len(openSet)==0:
        print("Open set is empty..")
        break
      expandIndex = min(openSet, key=lambda o: openSet[o].cost + self.calculate_heuristic(openSet[o],nodeGoal))
      currentNode = openSet[expandIndex]

      if currentNode.x == nodeGoal.x and currentNode.y == nodeGoal.y and currentNode.z==nodeGoal.z:
        nodeGoal.pind = currentNode.pind
        nodeGoal.cost = currentNode.cost
        break

      # Remove the item from the open set
      del openSet[expandIndex]

      # Add it to the closed set
      closedSet[expandIndex] = currentNode

      # expand_grid search grid based on motion model
      for i, _ in enumerate(self.motion):
        node = self.Node(currentNode.x + self.motion[i][0],
                         currentNode.y + self.motion[i][1],
                         currentNode.z + self.motion[i][2],
                         currentNode.cost + self.motion[i][3], expandIndex)
        nodeID = self.index_node(node)

        # If the node is not safe, do nothing
        if not self.verify_node(currentNode,node):
          continue

        if nodeID in closedSet:
          continue

        if nodeID not in openSet:
          openSet[nodeID] = node  # discovered a new node
        else:
          if openSet[nodeID].cost > node.cost:
            # This path is the best until now. record it
            openSet[nodeID] = node
    print('Total expanded Nodes:', expandedNode)
    rx, ry, rz= self.final_path(nodeGoal, closedSet)
    return rx, ry, rz
