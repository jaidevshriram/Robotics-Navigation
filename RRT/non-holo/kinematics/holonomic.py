import numpy as np
import matplotlib.pyplot as plt
import skgeom as sg
import shapely.geometry

from skgeom.draw import draw
from matplotlib import collections  as mc

from helpers import *

class Synchro:
    """
    This is a synchro drive robot. Such robots are free to move in any direction without any rotation required.
    Hence, the body frame of this robot remains the same throughout, with the wheels changing orientation to change direction.

    Here, the wheels are at four corners - defined as corner_transform.
    These are computed with respect to the origin.
    """
    def __init__(self, origin=(0, 0)):
        self.type = "NA"
        self.origin = origin
        self.corner_transform = np.array([
            [1, 1],
            [1, -1],
            [-1, -1],
            [-1, 1]
        ], dtype=np.float64)
        self.wheel_paths = [[[], []] for wheel in range(len(self.corner_transform))]
        self.shape = sg.Circle2(sg.Point2(0, 0), sg.Sign(5))

    def generate_wheel_paths(self, path):
        for i in range(1, len(path)):
            start = path[i-1]
            end = path[i]

            SMALLEST_STEP = 0.1
            n = int(distance(end, start) // SMALLEST_STEP) + 1

            STEP = (np.array(end) - np.array(start))/n

            for j in range(1, n+1):
                for wheel_ind in range(len(self.wheel_paths)):
                    size, y = np.array(start) + j*STEP + self.corner_transform[wheel_ind]
                    self.wheel_paths[wheel_ind][0].append(size)
                    self.wheel_paths[wheel_ind][1].append(y)

        return self.wheel_paths

    def get_shape(self):
        return self.shape

class SynchroSquare(Synchro):
    def __init__(self, origin=(0, 0), size=0.25):
        super().__init__(origin)
        self.type = "Square"
        self.shape = sg.Polygon([sg.Point2(-size, -size), sg.Point2(size, -size), sg.Point2(size, size), sg.Point2(-size, size)])
        self.corner_transform *= size
        self.max_radius = np.sqrt(size**2 + size**2)

class SynchroCircle(Synchro):
    def __init__(self, origin=(0, 0), size=0.25):
        super().__init__(origin)
        self.type = "Circle"
        self.shape = sg.Circle2(sg.Point2(origin[0], origin[1]), sg.Sign(1))
        self.r = size
        self.corner_transform *= size

if __name__ == '__main__':
    hol = SynchroCircle((0, 0)) 
    print(type(hol.shape))
    print(hol.r)
