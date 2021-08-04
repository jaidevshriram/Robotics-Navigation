import numpy as np
import matplotlib.pyplot as plt

from .obstacle import CircleObstacle, RectangleObstacle, RandomPolygon

class ObstacleList:
    """
    This class stores a list of objects
    """
    def __init__(self):
        self.obstacles = []

    def add_obstacle(self,  type, cfg):
        
        if type == "circle":
            self.obstacles.append(CircleObstacle(cfg))
        elif type == "rect":
            self.obstacles.append(RectangleObstacle(cfg))
        elif type == 'irregular':
            self.obstacles.append(RandomPolygon(cfg))
        else:
            raise RuntimeError("Obstacle shape not supported")

    def draw(self, subplot):
        for i, obstacle in enumerate(self.obstacles):
            shape = obstacle.draw()
            subplot.add_artist(shape)

            expanded = obstacle.draw_expanded()
            subplot.add_artist(expanded)

    def isInObstacle(self, point):
        for i, obstacle in enumerate(self.obstacles):
            if obstacle.isInside(point):
                return True
        return False

    def isThruObstacle(self, line):
        for i, obstacle in enumerate(self.obstacles):
            if obstacle.intersect(line):
                return True
        return False

    def minObstacleDist(self, point):
        min_val = np.inf
        for i, obstacle in enumerate(self.obstacles):
            min_val = min(obstacle.distFromShape(point), min_val)
        return min_val

    def grow(self, robot):
        for i, obstacle in enumerate(self.obstacles):
            obstacle.grow(robot)
