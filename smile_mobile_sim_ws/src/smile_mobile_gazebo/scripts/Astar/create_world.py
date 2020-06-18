#!/usr/bin/python3
'''
Author: BaoqianWang<wbq7758258@gmail.com>
Date Created: 05/26/2020
Description: This python module create objects in world.
'''

import rospy, tf
from gazebo_msgs.srv import DeleteModel, SpawnModel
from geometry_msgs.msg import *
import numpy as np
from std_srvs.srv import Empty as empty
from std_msgs.msg import Empty

SIZE=0.8 #This is predefined but should be extracted from model file, which is left to future work


class CreateWorldModel:
    """
    Create world model by adding objects
    """

    def __init__(self, model_file, map_file, node_name="spawn_custom_objects",):
        rospy.init_node(node_name)
        rospy.wait_for_service("/gazebo/delete_model")
        rospy.wait_for_service("/gazebo/spawn_sdf_model")
        rospy.wait_for_service('gazebo/reset_world')
        self.reset_simulation = rospy.ServiceProxy('/gazebo/reset_world', empty)
        print("Got it.")
        self.delete_model = rospy.ServiceProxy("/gazebo/delete_model", DeleteModel)
        self.spawn_model = rospy.ServiceProxy("/gazebo/spawn_sdf_model", SpawnModel)
        self.model_fname=model_file
        self.map_fname=map_file

    def load_map(self):
        '''
        Loads the bounady and blocks from map file fname.

        boundary = [['xmin', 'ymin', 'zmin', 'xmax', 'ymax', 'zmax','r','g','b']]

        blocks = [['xmin', 'ymin', 'zmin', 'xmax', 'ymax', 'zmax','r','g','b'],
                ...,
                ['xmin', 'ymin', 'zmin', 'xmax', 'ymax', 'zmax','r','g','b']]
        '''
        mapdata = np.loadtxt(self.map_fname,dtype={'names': ('type', 'xmin', 'ymin', 'zmin', 'xmax', 'ymax', 'zmax','r','g','b'),\
                                        'formats': ('S8','f', 'f', 'f', 'f', 'f', 'f', 'f','f','f')})

        blockIdx = mapdata['type'] == b'block'
        print(mapdata)
        print(blockIdx)
        print(mapdata[~blockIdx][['xmin', 'ymin', 'zmin', 'xmax', 'ymax', 'zmax','r','g','b']])
        boundary = mapdata[~blockIdx][['xmin', 'ymin', 'zmin', 'xmax', 'ymax', 'zmax','r','g','b']]#.view('<f4').reshape(-1,11)[:,2:]

        blocks = mapdata[blockIdx][['xmin', 'ymin', 'zmin', 'xmax', 'ymax', 'zmax','r','g','b']]#.view('<f4').reshape(-1,11)[:,2:]

        return boundary, blocks


    def run(self):
        #self.reset_simulation()
        q=tf.transformations.quaternion_from_euler(0,0,0)
        orient = Quaternion(x=q[0],y=q[1],z=q[2],w=q[3])

        with open(self.model_fname, "r") as f:
            product_xml = f.read()

        boundary, blocks=self.load_map()


        # for num in xrange(0,12):
        #     item_name = "product_{0}_0".format(num)
        #     print("Deleting model:%s", item_name)
        #     delete_model(item_name)
        for i, block in enumerate(blocks):
            px=block[0]
            py=block[1]
            pz=block[2]
            item_name   =   "box{0}".format(i)
            print("Spawning model:%s", item_name)
            item_pose   =   Pose(Point(x=px, y=py,  z=pz),   orient)
            self.spawn_model(item_name, product_xml, "", item_pose, "world")


        #     blocks = [['xmin', 'ymin', 'zmin', 'xmax', 'ymax', 'zmax','r','g','b']
        #
        # for num in xrange(0,12):
        #     bin_y   =   2.8 *   (num    /   6)  -   1.4
        #     bin_x   =   0.5 *   (num    %   6)  -   1.5
        #     item_name   =   "product_{0}_0".format(num)
        #     print("Spawning model:%s", item_name)
        #     item_pose   =   Pose(Point(x=bin_x, y=bin_y,    z=0.4),   orient)
        #     spawn_model(item_name, product_xml, "", item_pose, "world")


if __name__ == "__main__":
    model_file="/home/baoqian/smile-mobile/smile_mobile_sim_ws/src/smile_mobile_gazebo/worlds/box.sdf"
    map_file="./maps/smile_world.txt"
    world_model = CreateWorldModel(model_file, map_file)
    world_model.run()

'''
if __name__ == '__main__':
    print("Waiting for gazebo services...")
    rospy.init_node("spawn_products_in_bins")
    rospy.wait_for_service("/gazebo/delete_model")
    rospy.wait_for_service("/gazebo/spawn_sdf_model")
    print("Got it.")
    delete_model = rospy.ServiceProxy("/gazebo/delete_model", DeleteModel)
    spawn_model = rospy.ServiceProxy("/gazebo/spawn_sdf_model", SpawnModel)

    with open("/home/baoqian/husky/src/husky/husky_gazebo/worlds/box.sdf", "r") as f:
        product_xml = f.read()

    q=tf.transformations.quaternion_from_euler(0,0,0)
    orient = Quaternion(x=q[0],y=q[1],z=q[2],w=q[3])

    for num in xrange(0,12):
        item_name = "product_{0}_0".format(num)
        print("Deleting model:%s", item_name)
        delete_model(item_name)

    for num in xrange(0,12):
        bin_y   =   2.8 *   (num    /   6)  -   1.4
        bin_x   =   0.5 *   (num    %   6)  -   1.5
        item_name   =   "product_{0}_0".format(num)
        print("Spawning model:%s", item_name)
        item_pose   =   Pose(Point(x=bin_x, y=bin_y,    z=0),   orient)
        spawn_model(item_name, product_xml, "", item_pose, "world")
'''
