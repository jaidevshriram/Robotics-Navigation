import numpy as np
import matplotlib.pyplot as plt
import matplotlib.image as mpimg
from matplotlib import collections  as mc

from copy import deepcopy

from obstacles import obstaclelist as ObstacleList
from .graph import Graph
from .line import Line
from helpers import *

class Map:
    """
    This class represents the world map.
    """
    def __init__(self, img="", dimx=10, dimy=10):
        self.dimx = dimx
        self.dimy = dimy
        self.img = None
        self.G = None

        if img != "":
            self.img = mpimg.imread(img)
            self.dimx = self.img.shape[1]
            self.dimy = self.img.shape[0]

        self.obstaclelist = ObstacleList.ObstacleList()
        self.kinematics = None

    def create_graph(self, startpos, endpos, x_, y_):
        self.G = Graph(startpos, endpos, x_, y_)
        return self.G

    def load_graph(self, G):
        self.G = deepcopy(G)

    def clear_lines(self):
        self.G.edges = []
        self.G.vertices = []

    def add_obstacle(self, type, cfg):
        self.obstaclelist.add_obstacle(type, cfg)

    def isInObstacle(self, randvex):
        return self.obstaclelist.isInObstacle(randvex)

    def isThruObstacle(self, line):
        return self.obstaclelist.isThruObstacle(line)

    def growObstacles(self, robot):
        self.obstaclelist.grow(robot)

    def randomPosition(self):
        return self.G.randomPosition()

    def nearest(self, vex):
        Nvex = None
        Nidx = None
        minDist = float("inf")

        for idx, v in enumerate(self.G.vertices):
            line = Line(v, vex)

            if self.isThruObstacle(line):
                continue

            dist = distance(v, vex)
            if dist < minDist:
                minDist = dist
                Nidx = idx
                Nvex = v

        return Nvex, Nidx

    def draw(self, path=None, save=False, name=0):

        fig, ax = plt.subplots()

        if self.G is not None:

            px = [x for x, y in self.G.vertices]
            py = [y for x, y in self.G.vertices]

            marker_size = 50

            ax.scatter(px, py, marker='o', c='cyan')
            ax.scatter(self.G.startpos[0], self.G.startpos[1], c='green', marker='*', s=marker_size)
            ax.scatter(self.G.endpos[0], self.G.endpos[1], c='red', marker='*', s=marker_size)

            lines = [(self.G.vertices[edge[0]], self.G.vertices[edge[1]]) for edge in self.G.edges]
            lc = mc.LineCollection(lines, colors='green', linewidths=2)
            ax.add_collection(lc)

        if self.kinematics is not None:
                for i in range(len(self.kinematics)):
                    ax.plot(self.kinematics[i][0], self.kinematics[i][1])

        if path is not None:
            paths = [(path[i], path[i+1]) for i in range(len(path)-1)]
            lc2 = mc.LineCollection(paths, colors='blue', linewidths=3)
            ax.add_collection(lc2)

        self.obstaclelist.draw(ax)

        if self.img is not None:
            plt.imshow(self.img)

        plt.axis([-5, self.dimx, -5, self.dimy])
        plt.gca().set_aspect('equal', adjustable='box')

        ax.set_xticks([])
        ax.set_yticks([])

        plt.title('Holonomic Robot')

        if save:
            plt.savefig(f"map/out/{name}.jpg")
        else:
            plt.show()

        plt.close()

    def add_kinematics(self, kinematics):
        self.kinematics = kinematics

    def record_kinematics(self, path=None):

        for time in range(len(self.kinematics[0][0])):

            fig, ax = plt.subplots()

            if self.G is not None:

                px = [x for x, y in self.G.vertices]
                py = [y for x, y in self.G.vertices]

                marker_size = 50

                ax.scatter(px, py, marker='o', c='cyan')
                ax.scatter(self.G.startpos[0], self.G.startpos[1], c='green', marker='*', s=marker_size)
                ax.scatter(self.G.endpos[0], self.G.endpos[1], c='red', marker='*', s=marker_size)

                lines = [(self.G.vertices[edge[0]], self.G.vertices[edge[1]]) for edge in self.G.edges]
                lc = mc.LineCollection(lines, colors='green', linewidths=2)
                ax.add_collection(lc)

            if self.kinematics is not None:
                for i in range(len(self.kinematics)):
                    ax.plot(self.kinematics[i][0][:time], self.kinematics[i][1][:time])

            if path is not None:
                paths = [(path[i], path[i+1]) for i in range(len(path)-1)]
                lc2 = mc.LineCollection(paths, colors='blue', linewidths=3)
                ax.add_collection(lc2)

            self.obstaclelist.draw(ax)

            if self.img is not None:
                plt.imshow(self.img)

            plt.axis([-5, self.dimx, -5, self.dimy])
            plt.gca().set_aspect('equal', adjustable='box')

            ax.set_xticks([])
            ax.set_yticks([])

            plt.title('Holonomic Robot')

            plt.savefig(f"map/kin_out/{time}.jpg")
            plt.close()