import numpy as np
import math
show_animation = True
import matplotlib.pyplot as plt
import pyrr

class MyPlanner:
  #__slots__ = ['boundary', 'blocks']

  def __init__(self, boundary, blocks, res):
    self.boundary = boundary
    self.blocks = blocks
    self.resolution=res
    self.xmin = round(boundary[0,0])
    self.ymin = round(boundary[0,1])
    self.zmin = round(boundary[0,2])

    self.xmax = round(boundary[0,3])
    self.ymax = round(boundary[0,4])
    self.zmax = round(boundary[0,5])

    self.xwidth = round((self.xmax - self.xmin) / self.resolution)
    self.ywidth = round((self.ymax - self.ymin) / self.resolution)
    self.zwidth = round((self.zmax - self.zmin) / self.resolution)
    #Define the motion of the node
    self.motion = self.motion_model()

  #Define the discrete node
  class Node:
      def __init__(self, x, y ,z, cost, pind):
          self.x=x
          self.y=y
          self.z=z
          self.cost = cost
          self.pind = pind

      def __str__(self):
          return str(self.x) + "," + str(self.y) + "," + str(self.z) + ","+ str(
                self.cost) + "," + str(self.pind)

  #Discretize the continous position to discrete node index
  def discretize(self,position,minPos):
      return round((position-minPos)/self.resolution)

  #Create a index of each node
  def index_node(self,node):
      return (node.z-self.zmin)*self.ywidth*self.xwidth+(node.y - self.ymin) * self.xwidth + (node.x - self.xmin)

  #Get the position of the node
  def pos_node(self, index, minp):
      pos = index * self.resolution + minp
      return pos

  #Calculate the heuristic function
  def calculate_heuristic(self,node1,node2):
      w = 1.0  # weight of heuristic
      d = w * np.sqrt((node1.x - node2.x)**2+(node1.y - node2.y)**2+(node1.z-node2.z)**2)
      return d

  #Define the motion model
  def motion_model(self):
      # Consider the horizontal, vertical, diagonal movement
      # dx, dy, dz, cost
      motion = [[1, 0,0,1],
                [0, 1,0,1],
                [-1, 0,0,1],
                [0, -1,0,1],
                [0, 0, -1, 1],
                [0, 0, 1, 1],
                [-1, -1, -1,math.sqrt(3)],
                [-1, 1,-1, math.sqrt(3)],
                [1, -1,-1, math.sqrt(3)],
                [1, 1,-1, math.sqrt(3)],
                [-1, -1,1, math.sqrt(3)],
                [-1, 1, 1,math.sqrt(3)],
                [1, -1, 1,math.sqrt(3)],
                [1, 1, 1,math.sqrt(3)]]

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
      pz2 = self.pos_node(node2.z, self.zmin)

      px1 = self.pos_node(node1.x, self.xmin)
      py1 = self.pos_node(node1.y, self.ymin)
      pz1 = self.pos_node(node1.z, self.zmin)

      startPoint=np.array([px1, py1, pz1])
      endPoint=np.array([px2,py2,pz2])

      #Boundary checking
      if px2 < self.xmin or py2 < self.ymin or pz2 < self.zmin or px2 >= self.xmax or py2 >= self.ymax or pz2 >= self.zmax:
        return False

      #Collision check
      for k in range(self.blocks.shape[0]):
         # Convert box to configuration space by adding the robot radius
        box=np.array([[self.blocks[k, 0]-0.6,self.blocks[k, 1]-0.6,self.blocks[k, 2]],[self.blocks[k, 3]+0.6,self.blocks[k, 4]+0.6,self.blocks[k, 5]]])

        if self.collision_checking(startPoint, endPoint, box):
            return False
      return True

  #Generate the final path
  def final_path(self, nodeGoal, closedSet):
      rx, ry, rz = [self.pos_node(nodeGoal.x, self.xmin)], [
          self.pos_node(nodeGoal.y, self.ymin)], [self.pos_node(nodeGoal.z, self.zmin)]
      pind = nodeGoal.pind
      while pind != -1:
          n = closedSet[pind]
          rx.append(self.pos_node(n.x, self.xmin))
          ry.append(self.pos_node(n.y, self.ymin))
          rz.append(self.pos_node(n.z, self.zmin))
          pind = n.pind

      return rx, ry, rz

  #Search the environment and do planning
  def plan(self,start,goal):
    # ax is used to draw the point
    nodeStart=self.Node(self.discretize(start[0],self.xmin),self.discretize(start[1],self.ymin),self.discretize(start[2],self.zmin),0.0,-1)
    nodeGoal=self.Node(self.discretize(goal[0],self.xmin),self.discretize(goal[1],self.ymin),self.discretize(goal[2],self.zmin),0.0,-1)
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
