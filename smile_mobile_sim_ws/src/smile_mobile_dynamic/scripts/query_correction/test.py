import numpy as np
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

graph = {'a': {'c':7.2},
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

path=shortest_path(graph, 'm', 'a')
print(path)
