import numpy as np
import math
import matplotlib.pyplot as plt
import pyrr





class AstarPlanningPavedUnpaved:

    def __init__(self, unpaved_boundary, static_obstacles, resolution):

        self.unpaved_boundary=unpaved_boundary
        self.static_obstacles=static_obstacles
        self.res=resolution
        self.xmin=round(boundary[0,0])
        self.ymin=round(boundary[0,1])
        self.xmax=round(boundary[0,2])
        self.ymax=round(boundary[0,3])

        return

    def construct_graph_unpaved_environment(self):
        graph=dict()
        x=[x_value for x_value in np.arrange(self.xmin, self.xmax, self.res)]
        y=[y_value for y_value in np.arrange(self.ymin, self.ymax, self.res)]

        node_index=0
        for x_value in x:
            for y_value in y:
                graph['%d' %node_index]



        #Consider bounary nodes first







        pass
