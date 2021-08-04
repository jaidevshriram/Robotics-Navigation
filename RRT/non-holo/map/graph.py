import numpy as np
from random import random
    

# Defining Node class with related parameters
class Node():
    def __init__(self,x,y):
        self.x=x
        self.y=y
        self.p=None
        self.pt1_x=None
        self.pt1_y=None
        self.pt2_x=None
        self.pt2_y=None
        self.Ti=None
        self.S=None
        self.coord1=None
        self.coord2=None
        self.coord3=None
        self.coord4=None
        self.theta11=None
        self.theta12=None
        self.theta13=None
        self.theta14=None

class Graph:
    def __init__(self, startpos, endpos, x_, y_):
        self.startpos = startpos
        self.endpos = endpos

        self.vertices = [startpos]
        self.edges = []
        self.success = False

        self.vex2idx = {startpos:0}
        self.neighbors = {0:[]}
        self.distances = {0:0.}

        self.sx = endpos[0] - startpos[0]
        self.sy = endpos[1] - startpos[1]

        self.grid_min_max_x = x_
        self.grid_min_max_y = y_

    def add_vex(self, pos):
        try:
            idx = self.vex2idx[pos]
        except:
            idx = len(self.vertices)
            self.vertices.append(pos)
            self.vex2idx[pos] = idx
            self.neighbors[idx] = []
        return idx

    def add_edge(self, idx1, idx2, cost):
        self.edges.append((idx1, idx2))
        self.neighbors[idx1].append((idx2, cost))
        self.neighbors[idx2].append((idx1, cost))

    def randomPosition(self):
        rx = random()
        ry = random()

        posx = self.grid_min_max_x[0] + rx * self.grid_min_max_x[1]
        posy = self.grid_min_max_y[0] + ry * self.grid_min_max_y[1]
        return posx, posy