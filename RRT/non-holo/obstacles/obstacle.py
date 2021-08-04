import numpy as np
import matplotlib.pyplot as plt
import shapely.geometry
import shapely.ops
import descartes
import skgeom as sg
import matplotlib.patches as patches

from matplotlib.path import Path
from skgeom import minkowski
from skgeom import boolean_set

from helpers import *

class Obstacle:
    """
    Base class to represent obstacles.
    """
    def __init__(self, x, y):
        self.x = x
        self.y = y

    def draw(self):
        raise RuntimeError("Not Implemented!")

    def draw_expanded(self):
        raise RuntimeError("Not Implemented!")

    def intersect(line):
        raise RuntimeError("Not Implemented!")

    def isInside(self, point):
        raise RuntimeError("Not Implemented!")

    def distFromShape(self, point):
        raise RuntimeError("Not Implemented!")

    def grow(self, robot):
        raise RuntimeError("Not Implemented!")


class CircleObstacle(Obstacle):
    """
    The circle is represented by (Specified in cfg dict):
    - x, y: Center of circle
    - r: Radius of circle
    - colour (optional): Matplotlib colour
    """
    def __init__(self, cfg):

        try:
            x = cfg['x']
            y = cfg['y']
            self.r = cfg['r']
        except:
            raise RuntimeError("Circle Coordinates not correctly specified")

        if 'colour' in cfg.keys():
            self.colour = cfg['colour']
        else:
            self.colour = 'black'

        super().__init__(x, y)
        self.baseshape = shapely.geometry.Point(self.x, self.y).buffer(self.r)
        self.shape = self.baseshape

    def draw(self):
        return descartes.PolygonPatch(self.baseshape, fc=self.colour, ec=self.colour)

    def draw_expanded(self):
        return descartes.PolygonPatch(self.shape, fc=self.colour, ec=self.colour, alpha=0.5)

    def isInside(self, point):
        if distance((self.x, self.y), point) <= self.r:
            return True
        else:
            return False

    def intersect(self, line):
        line = shapely.geometry.LineString([line.p0, line.p1])
        return line.intersects(self.shape)

    def distFromShape(self, point):
        return distance((self.x, self.y), point) - self.r

    def grow(self, robot):

        radius = 0
        if robot.type == "Circle":
            radius = robot.r
        elif robot.type == "Square":
            radius = robot.max_radius
        else:
            raise RuntimeError(f"Robot shape {robot.type} not supported!")

        self.r += radius
        self.shape = self.baseshape.buffer(radius)

class RandomPolygon(Obstacle):
    def __init__(self, cfg):

        try:
            self.xs = cfg['x']
            self.ys = cfg['y']
            assert(type(self.xs) == list)
            assert(type(self.ys) == list)
        except:
            raise RuntimeError("Polygon Coordinates not correctly specified")

        if 'colour' in cfg.keys():
            self.colour = cfg['colour']
        else:
            self.colour = 'black'

        points = np.array(list(zip(self.xs, self.ys)))
        self.baseshape = sg.Polygon(points)
        vertices = to_list_of_tuples(self.baseshape.vertices)
        self.shape = shapely.geometry.Polygon(vertices)

        super().__init__(-1, -1)

    def draw(self):

        if isinstance(self.baseshape, sg.Polygon):
            vertices = to_list_of_tuples(self.baseshape.vertices) + [(0, 0)]
            codes = [Path.MOVETO] + [Path.LINETO] * (len(vertices) - 2) + [Path.CLOSEPOLY]
            path = Path(vertices, codes)
            return patches.PathPatch(path, facecolor=self.colour, ec=self.colour)
        else:
            return descartes.PolygonPatch(self.baseshape, fc=self.colour, ec=self.colour)

    def draw_expanded(self):
        return descartes.PolygonPatch(self.shape, fc=self.colour, ec=self.colour, alpha=0.5) 

    def isInside(self, point):
        point = shapely.geometry.Point(point[0], point[1])
        return point.within(self.shape)

    def intersect(self, line):
        line = shapely.geometry.LineString([line.p0, line.p1])
        return line.intersects(self.shape)

    def distFromShape(self, point):
        point = shapely.geometry.Point(point[0], point[1])
        closest_point, _ = shapely.ops.nearest_points(self.shape, point)
        return distance(point, closest_point)

    def grow(self, robot):

        if robot.type == "Circle":
            self.baseshape = shapely.geometry.Polygon(zip(self.xs, self.ys))
            self.shape = self.baseshape.buffer(robot.r)
        elif robot.type == "Square":
            temp = minkowski.minkowski_sum(robot.shape, self.baseshape)
            polygon = temp.outer_boundary()
            polygon_with_holes = temp
            plot_vertices = False    

            vertices = to_list_of_tuples(polygon.vertices)
            self.shape = shapely.geometry.Polygon(vertices)
        else:
            raise RuntimeError("Robot shape not supported!")

class RectangleObstacle(RandomPolygon):
    """
    The rectangle is represented by (Specified in cfg dict):
    - x, y: Bottom Left corner
    - dimx, dimy: Width and height of the rectangle respectively
    - colour (optional): Matplotlib colour
    """
    def __init__(self, cfg):

        try:
            x = cfg['x']
            y = cfg['y']
            self.dimx = cfg['dimx']
            self.dimy = cfg['dimy']
        except:
            raise RuntimeError("Rectangle Coordinates not correctly specified")

        if 'colour' in cfg.keys():
            self.colour = cfg['colour']
        else:
            self.colour = 'black'

        cfg['x'] = [x, x+self.dimx, x+self.dimx, x]
        cfg['y'] = [y, y, y+self.dimy, y+self.dimy]

        super().__init__(cfg)