import numpy as np
import matplotlib.pyplot as plt

import skgeom as sg
from skgeom import minkowski
from skgeom.draw import draw

from planner import dijkstra
from obstacles import obstaclelist
from map import map as mapper
from helpers import *

from kinematics import holonomic

class RRT:
    """
    This classes creates the RRT on a given map for a given start, and end position.
    Required inputs for instantiation:
    - map: Instance of map class, with obstacles pre-loaded
    - startpos/endpos = (x, y) coordinates for start/end pos respectively. (Must be within the map grid)
    """
    def __init__(self, map, startpos, endpos, x_=(0, 10), y_=(-5, 15)):
        self.map = map
        self.map.create_graph(startpos, endpos, x_, y_)

    # After the tree is created, it returns the computed graph
    def create_tree(self, n_iter=200, stepSize=0.7):
        for _ in range(n_iter):
            self.map.draw(save=True, name=_)
            randvex = self.map.randomPosition()

            if self.map.isInObstacle(randvex):
                continue
            
            nearvex, nearidx = self.map.nearest(randvex)
            if nearvex is None:
                continue

            newvex = newVertex(randvex, nearvex, stepSize)
            print(randvex)

            newidx = self.map.G.add_vex(newvex)
            dist = distance(newvex, nearvex)
            self.map.G.add_edge(newidx, nearidx, dist)

            dist = distance(newvex, self.map.G.endpos)

            if dist < stepSize:
                endidx = self.map.G.add_vex(self.map.G.endpos)
                self.map.G.add_edge(newidx, endidx, dist)
                self.map.G.success = True
                print("success")

        return self.map.G        

    def get_graph(self):    
        return self.map.G

if __name__ == '__main__':

    startpos = (0., 5.)
    endpos = (18., 5.)
    n_iter = 400
    stepSize = 1.5

    robot = holonomic.SynchroSquare(size=1)

    map = mapper.Map(dimx=20, dimy=18)

    # Basic map
    # map.add_obstacle("circle", {'x': 3.5, 'y': 3.5, 'r': 1.0, 'colour': 'teal', 'type': 'circle'})
    # map.add_obstacle("rect", {'x': 3, 'y': 0, 'dimx': 2, 'dimy': 2, 'colour': 'teal', 'type': 'rect'})
    # map.add_obstacle("circle", {'x': 2, 'y': 2, 'r': 0.5, 'colour': 'teal', 'type': 'circle'})
    # map.add_obstacle("irregular", {'x': [1, 2, 1, 0], 'y': [1, 2, 4, 2], 'colour': 'teal', 'type': 'irregular'})
    # map.add_obstacle("irregular", {'x': [-2, -2, -1, -1], 'y': [0, 4, 4, 0], 'colour': 'teal', 'type': 'irregular'})
    
    
    # Complex map
    map.add_obstacle("rect", {'x': 3, 'y': 2, 'dimx': 3, 'dimy': 7, 'colour': 'dodgerblue', 'type': 'rect'}) 
    map.add_obstacle("irregular", {'x': [10, 11, 12, 11], 'y': [5, 4, 5, 6], 'colour': 'dodgerblue', 'type': 'irregular'})
    map.add_obstacle("circle", {'x': 15, 'y': 3, 'r': 1.5, 'colour': 'darkgreen', 'type': 'circle'})
    map.add_obstacle("circle", {'x': 15, 'y': 12, 'r': 1.5, 'colour': 'darkgreen', 'type': 'circle'})

    map.growObstacles(robot)

    # startpos = (0., 0.)
    # endpos = (500., 500.)
    # n_iter = 1000
    # stepSize = 15

    # map = mapper.Map("map/imgs/hyperdrive.jpg")
    # map.add_obstacle("circle", {'x': 600, 'y': 600, 'r': 50, 'colour': 'teal', 'type': 'circle'})
    # map.add_obstacle("circle", {'x': 200, 'y': 200, 'r': 100, 'colour': 'teal', 'type': 'circle'})

    # map.add_obstacle("rect", {'x': 0, 'y': 0, 'dimx': 20, 'dimy': 20, 'colour': 'azure', 'type': 'rect'})

    planner = RRT(map, startpos, endpos, (0, 19), (-2, 10))
    G = planner.create_tree(n_iter, stepSize)
    map.load_graph(G)

    if G.success:
        path = dijkstra(G)
        wheel_paths = robot.generate_wheel_paths(path)
        map.draw(path)

        map.clear_lines()
        map.add_kinematics(wheel_paths)
        map.draw(path)

        map.record_kinematics()
    else:
        map.draw()